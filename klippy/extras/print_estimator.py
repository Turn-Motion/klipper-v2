# Klipper extra: print_estimator
#
# Runs inside the persistent live klippy process and exposes a single API
# endpoint:
#
#   method: "estimate_file"
#   params: { "filename": "myprint.gcode", "include_moves": 1 }
#
# On request it spawns an isolated batch klippy subprocess with a temporary
# config that appends [move_capture] to the sim config. The subprocess runs
# the full Klipper motion planner (LookAheadQueue + trapq) at CPU speed,
# captures every move's trapezoid data, then exits. The parent reads the
# results and returns structured JSON — no live printer state is touched.
#
# Response:
#   {
#     "print_time":      142.3,         # seconds
#     "move_count":      4821,
#     "filament_used_mm": 8341.2,
#     "moves": [                         # empty if include_moves=0
#       {
#         "start_pos":  [x,y,z,a,b],
#         "end_pos":    [x,y,z,a,b],
#         "start_v":    0.0,             # mm/s
#         "cruise_v":   180.5,
#         "end_v":      12.3,
#         "accel_t":    0.031,           # seconds
#         "cruise_t":   0.18,
#         "decel_t":    0.028,
#         "move_d":     12.4,            # mm (in move_d space)
#         "is_kinematic": true,
#         "extrude_d":  0.042            # mm, signed
#       }, ...
#     ]
#   }
import os, sys, json, subprocess, tempfile, threading, logging

# Absolute path of klippy.py relative to this file (extras/ → klippy/)
_KLIPPY_PY = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'klippy.py')

class PrintEstimator:
    def __init__(self, config):
        self.printer    = config.get_printer()
        self.dict_path  = config.get('dictionary_path',
                                     '/klipper/out/klipper.dict')
        self.gcode_dir  = config.get('gcode_dir', '/gcode')
        self._lock      = threading.Lock()
        self._cfg_file  = self.printer.get_start_args()['config_file']
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect)

    def _handle_connect(self):
        # In batch mode (-i flag) there is no API server; skip registration.
        webhooks = self.printer.lookup_object('webhooks', None)
        if webhooks is None:
            return
        webhooks.register_endpoint('estimate_file', self._handle_estimate)
        logging.info("print_estimator: API endpoint 'estimate_file' registered")

    # ── API handler ──────────────────────────────────────────────────────────

    def _handle_estimate(self, web_request):
        filename = web_request.get_str('filename')
        # Security: reject any path traversal attempt
        if os.sep in filename or '/' in filename or '..' in filename:
            raise web_request.error("filename must be a plain filename, not a path")
        gcode_path = os.path.join(self.gcode_dir, filename)
        if not os.path.isfile(gcode_path):
            raise web_request.error("File not found: %s" % filename)
        include_moves = int(web_request.get('include_moves', 1))
        with self._lock:
            result = self._run(gcode_path, include_moves=bool(include_moves))
        web_request.send(result)

    # ── Estimation logic ─────────────────────────────────────────────────────

    def _run(self, gcode_path, include_moves=True):
        moves_out  = tempfile.mktemp(prefix='klipper_moves_',  suffix='.jsonl')
        log_out    = tempfile.mktemp(prefix='klipper_estlog_', suffix='.log')
        serial_out = tempfile.mktemp(prefix='klipper_serial_', suffix='.out')

        # Write a temp config = sim config + [move_capture] section
        with tempfile.NamedTemporaryFile(mode='w', suffix='.cfg',
                                         prefix='klipper_estcfg_',
                                         delete=False) as cf:
            with open(self._cfg_file) as src:
                cf.write(src.read())
            cf.write('\n\n[move_capture]\noutput_path = %s\n' % moves_out)
            tmp_cfg = cf.name

        try:
            logging.info("print_estimator: running batch estimation for %s",
                         gcode_path)
            subprocess.run(
                [sys.executable, _KLIPPY_PY,
                 tmp_cfg,
                 '-i', gcode_path,
                 '-o', serial_out,
                 '-d', self.dict_path,
                 '-l', log_out],
                timeout=300,
                check=False,
            )
            return self._parse(log_out, moves_out, include_moves)
        finally:
            for p in [tmp_cfg, log_out, serial_out]:
                try:
                    os.unlink(p)
                except OSError:
                    pass

    def _parse(self, log_path, moves_path, include_moves):
        print_time   = 0.0
        move_count   = 0
        filament_mm  = 0.0
        moves        = []

        # Pull print_time / move_count from the batch log line
        # e.g.: "print_time=142.368  move_count=4821  print_stall=0"
        if os.path.isfile(log_path):
            with open(log_path) as f:
                for line in f:
                    if 'print_time=' not in line:
                        continue
                    for tok in line.split():
                        if tok.startswith('print_time='):
                            try:
                                print_time = float(tok.split('=', 1)[1])
                            except ValueError:
                                pass
                        elif tok.startswith('move_count='):
                            try:
                                move_count = int(tok.split('=', 1)[1])
                            except ValueError:
                                pass

        # Parse per-move JSONL written by move_capture.py
        if os.path.isfile(moves_path):
            with open(moves_path) as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        m = json.loads(line)
                        filament_mm += abs(m.get('ed', 0.))
                        if include_moves:
                            moves.append({
                                'start_pos':    m['sp'],
                                'end_pos':      m['ep'],
                                'start_v':      m['sv'],
                                'cruise_v':     m['cv'],
                                'end_v':        m['ev'],
                                'accel_t':      m['at'],
                                'cruise_t':     m['ct'],
                                'decel_t':      m['dt'],
                                'move_d':       m['md'],
                                'is_kinematic': m['kin'],
                                'extrude_d':    m['ed'],
                            })
                    except (json.JSONDecodeError, KeyError) as e:
                        logging.warning("print_estimator: bad move line: %s", e)
            try:
                os.unlink(moves_path)
            except OSError:
                pass

        return {
            'print_time':       round(print_time, 3),
            'move_count':       move_count,
            'filament_used_mm': round(filament_mm, 3),
            'moves':            moves,
        }

def load_config(config):
    return PrintEstimator(config)
