# Klipper extra: move_capture
#
# Loaded ONLY in the batch estimation subprocess (not in the live klippy).
# Wraps the toolhead's lookahead flush to write one JSON line per move to
# a file as the moves leave the lookahead queue. The parent
# print_estimator.py reads that file and returns the data via the API.
#
# Each line in the output JSONL is a compact dict:
#   sp  start_pos [x,y,z,a,b]
#   ep  end_pos   [x,y,z,a,b]
#   sv  start velocity  (mm/s)
#   cv  cruise velocity (mm/s)
#   ev  end velocity    (mm/s)
#   at  accel time      (s)
#   ct  cruise time     (s)
#   dt  decel time      (s)
#   md  move distance   (mm)
#   kin True = kinematic (XY/Z/AB) move, False = extrude-only
#   ed  extruder distance (mm, signed)
import json, logging

class MoveCapture:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.output_path = config.get('output_path', '/tmp/klipper_moves.jsonl')
        self._file = None
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect)
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect)

    def _handle_connect(self):
        toolhead = self.printer.lookup_object('toolhead')
        lookahead = toolhead.lookahead
        orig_flush = lookahead.flush
        f = open(self.output_path, 'w')
        self._file = f

        def capture(lazy=False):
            moves = orig_flush(lazy=lazy)
            for move in moves:
                try:
                    f.write(json.dumps({
                        'sp': [round(v, 4) for v in move.start_pos[:5]],
                        'ep': [round(v, 4) for v in move.end_pos[:5]],
                        'sv': round(move.start_v,   4),
                        'cv': round(move.cruise_v,  4),
                        'ev': round(move.end_v,     4),
                        'at': round(move.accel_t,   6),
                        'ct': round(move.cruise_t,  6),
                        'dt': round(move.decel_t,   6),
                        'md': round(move.move_d,    4),
                        'kin': move.is_kinematic_move,
                        'ed': round(
                            move.axes_d[5] if len(move.axes_d) > 5 else 0., 5),
                    }) + '\n')
                except Exception as e:
                    logging.warning(
                        "move_capture: failed to capture move: %s", e)
            f.flush()
            return moves

        lookahead.flush = capture
        logging.info("move_capture: writing to %s", self.output_path)

    def _handle_disconnect(self):
        if self._file:
            self._file.close()
            self._file = None

def load_config(config):
    return MoveCapture(config)
