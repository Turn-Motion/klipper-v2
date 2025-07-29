# ADS1220 Support
#
# Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import time
from . import bulk_sensor, bus

#
# Constants
#
BYTES_PER_SAMPLE = 4  # samples are 4 byte wide unsigned integers
MAX_SAMPLES_PER_MESSAGE = bulk_sensor.MAX_BULK_MSG_SIZE // BYTES_PER_SAMPLE
UPDATE_INTERVAL = 0.10
RESET_CMD = 0x06
START_SYNC_CMD = 0x08
RREG_CMD = 0x20
WREG_CMD = 0x40
NOOP_CMD = 0x0
RESET_STATE = bytearray([0x0, 0x0, 0x0, 0x0])

# turn bytearrays into pretty hex strings: [0xff, 0x1]
def hexify(byte_array):
    return "[%s]" % (", ".join([hex(b) for b in byte_array]))


class ADS1220:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.last_error_count = 0
        self.consecutive_fails = 0
        # Debug option - enabled by default
        self.debug = config.getboolean('debug', True)
        if self.debug:
            logging.info("ADS1220 %s: Debug mode enabled", self.name)
        # Hardware test mode - disabled by default
        self.hw_test_mode = config.getboolean('hw_test_mode', True)
        if self.hw_test_mode:
            logging.info("ADS1220 %s: Hardware test mode enabled", self.name)
            self.register_test_command()
        # Pin test on startup if requested
        self.pin_test_on_startup = config.getboolean('pin_test_on_startup', True)
        # Register this instance in the global registry
        ADS1220_INSTANCES[self.name] = self
        # Register global test command if this is the first instance
        if len(ADS1220_INSTANCES) == 1:
            gcode = self.printer.lookup_object('gcode')
            gcode.register_command('ADS1220TESTANY', self.cmd_ADS1220_TEST_ANY,
                                  desc="Test any ADS1220 sensor by name")
        # Chip options
        # Gain
        self.gain_options = {'1': 0x0, '2': 0x1, '4': 0x2, '8': 0x3, '16': 0x4,
                             '32': 0x5, '64': 0x6, '128': 0x7}
        self.gain = config.getchoice('gain', self.gain_options, default='128')
        # Sample rate
        self.sps_normal = {'20': 20, '45': 45, '90': 90, '175': 175,
                           '330': 330, '600': 600, '1000': 1000}
        self.sps_turbo = {'40': 40, '90': 90, '180': 180, '350': 350,
                          '660': 660, '1200': 1200, '2000': 2000}
        self.sps_options = self.sps_normal.copy()
        self.sps_options.update(self.sps_turbo)
        self.sps = config.getchoice('sample_rate', self.sps_options,
                                    default='600')
        self.is_turbo = str(self.sps) in self.sps_turbo
        # Input multiplexer: AINP and AINN
        mux_options = {'AIN0_AIN1': 0b0000, 'AIN0_AIN2': 0b0001,
                       'AIN0_AIN3': 0b0010, 'AIN1_AIN2': 0b0011,
                       'AIN1_AIN3': 0b0100, 'AIN2_AIN3': 0b0101,
                       'AIN1_AIN0': 0b0110, 'AIN3_AIN2': 0b0111,
                       'AIN0_AVSS': 0b1000, 'AIN1_AVSS': 0b1001,
                       'AIN2_AVSS': 0b1010, 'AIN3_AVSS': 0b1011}
        self.mux = config.getchoice('input_mux', mux_options,
                                    default='AIN0_AIN1')
        # PGA Bypass
        self.pga_bypass = config.getboolean('pga_bypass', default=False)
        # bypass PGA when AVSS is the negative input
        force_pga_bypass = self.mux >= 0b1000
        self.pga_bypass = force_pga_bypass or self.pga_bypass
        # Voltage Reference
        self.vref_options = {'internal': 0b0, 'REF0': 0b01, 'REF1': 0b10,
                             'analog_supply': 0b11}
        self.vref = config.getchoice('vref', self.vref_options,
                                     default='internal')
        # check for conflict between REF1 and AIN0/AIN3
        mux_conflict = [0b0000, 0b0001, 0b0010, 0b0100, 0b0101, 0b0110, 0b0111,
                        0b1000, 0b1011]
        if self.vref == 0b10 and self.mux in mux_conflict:
            raise config.error("ADS1220 config error: AIN0/REFP1 and AIN3/REFN1"
                               " cant be used as a voltage reference and"
                               " an input at the same time")
        # SPI Setup
        # Use a more conservative SPI speed for better reliability
        # Original: spi_speed = 512000 if self.is_turbo else 256000
        spi_speed = config.getint('spi_speed', 256000)  # Default to 100kHz for stability
        logging.info("ADS1220: Using SPI speed of %d Hz", spi_speed)
        
        # Try SPI mode 0 (CPOL=0, CPHA=0) for ADS1220
        # The datasheet specifies CPOL=0, and some ADS1220 implementations work better with mode 0
        spi_mode = config.getint('spi_mode', 0)  # Allow configuring the mode
        logging.info("ADS1220 %s: Using SPI mode %d", self.name, spi_mode)
        self.spi = bus.MCU_SPI_from_config(config, spi_mode, default_speed=spi_speed)
        self.mcu = mcu = self.spi.get_mcu()
        self.oid = mcu.create_oid()
        # Data Ready (DRDY) Pin
        drdy_pin = config.get('data_ready_pin')
        ppins = printer.lookup_object('pins')
        drdy_ppin = ppins.lookup_pin(drdy_pin)
        self.data_ready_pin = drdy_ppin['pin']
        drdy_pin_mcu = drdy_ppin['chip']
        if drdy_pin_mcu != self.mcu:
            raise config.error("ADS1220 config error: SPI communication and"
                               " data_ready_pin must be on the same MCU")
        # Bulk Sensor Setup
        self.bulk_queue = bulk_sensor.BulkDataQueue(self.mcu, oid=self.oid)
        # Clock tracking
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        # Measurement conversion
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth, "<i")
        # Process messages in batches
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch, self._start_measurements,
            self._finish_measurements, UPDATE_INTERVAL)
        # Command Configuration
        self.attach_probe_cmd = None
        mcu.add_config_cmd(
            "config_ads1220 oid=%d spi_oid=%d data_ready_pin=%s"
            % (self.oid, self.spi.get_oid(), self.data_ready_pin))
        mcu.add_config_cmd("query_ads1220 oid=%d rest_ticks=0"
                           % (self.oid,), on_restart=True)
        mcu.register_config_callback(self._build_config)
        self.query_ads1220_cmd = None

    def _build_config(self):
        # Run pin test if requested
        if self.pin_test_on_startup:
            self.test_pins()
        cmdqueue = self.spi.get_command_queue()
        self.query_ads1220_cmd = self.mcu.lookup_command(
            "query_ads1220 oid=%c rest_ticks=%u", cq=cmdqueue)
        self.attach_probe_cmd = self.mcu.lookup_command(
            "ads1220_attach_load_cell_probe oid=%c load_cell_probe_oid=%c")
        self.ffreader.setup_query_command("query_ads1220_status oid=%c",
                                          oid=self.oid, cq=cmdqueue)

    def get_mcu(self):
        return self.mcu

    def get_samples_per_second(self):
        return self.sps

    # returns a tuple of the minimum and maximum value of the sensor, used to
    # detect if a data value is saturated
    def get_range(self):
        return -0x800000, 0x7FFFFF

    # add_client interface, direct pass through to bulk_sensor API
    def add_client(self, callback):
        self.batch_bulk.add_client(callback)

    def attach_load_cell_probe(self, load_cell_probe_oid):
        self.attach_probe_cmd.send([self.oid, load_cell_probe_oid])

    # Measurement decoding
    def _convert_samples(self, samples):
        adc_factor = 1. / (1 << 23)
        count = 0
        for ptime, val in samples:
            samples[count] = (round(ptime, 6), val, round(val * adc_factor, 9))
            count += 1
        del samples[count:]

    # Start, stop, and process message batches
    def _start_measurements(self):
        self.last_error_count = 0
        self.consecutive_fails = 0
        # Start bulk reading
        self.reset_chip()
        self.setup_chip()
        rest_ticks = self.mcu.seconds_to_clock(1. / (10. * self.sps))
        self.query_ads1220_cmd.send([self.oid, rest_ticks])
        logging.info("ADS1220 starting '%s' measurements", self.name)
        # Initialize clock tracking
        self.ffreader.note_start()

    def _finish_measurements(self):
        # don't use serial connection after shutdown
        if self.printer.is_shutdown():
            return
        # Halt bulk reading
        self.query_ads1220_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info("ADS1220 finished '%s' measurements", self.name)

    def _process_batch(self, eventtime):
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        return {'data': samples, 'errors': self.last_error_count,
                'overflows': self.ffreader.get_last_overflows()}

    def reset_chip(self):
        # the reset command takes 50us to complete
        # Try reset up to 3 times
        if self.debug:
            logging.info("ADS1220 %s: Starting chip reset", self.name)
        
        for retry in range(3):
            if self.debug:
                logging.info("ADS1220 %s: Reset attempt %d", self.name, retry + 1)
            
            try:
                self.send_command(RESET_CMD)
                # Add a delay to ensure reset completes (50us minimum, use 5ms to be safe)
                time.sleep(0.005)
                
                # read startup register state and validate
                if self.debug:
                    logging.info("ADS1220 %s: Reading registers after reset", self.name)
                val = self.read_reg(0x0, 4)
                
                if self.debug:
                    logging.info("ADS1220 %s: Reset state: got %s, expected %s", 
                                 self.name, hexify(val), hexify(RESET_STATE))
                
                if val == RESET_STATE:
                    if self.debug:
                        logging.info("ADS1220 %s: Reset successful", self.name)
                    return
                
                # If not successful, wait a bit longer and try again
                logging.warning("ADS1220 %s: Reset attempt %d failed: got %s, expected %s", 
                               self.name, retry + 1, hexify(val), hexify(RESET_STATE))
                time.sleep(0.010 * (retry + 1))
            except Exception as e:
                logging.error("ADS1220 %s: Exception during reset attempt %d: %s", 
                              self.name, retry + 1, str(e))
                time.sleep(0.010 * (retry + 1))
        
        # If we get here, all retries failed
        logging.error("ADS1220 %s: All reset attempts failed. Last state: %s", 
                      self.name, hexify(val))
        raise self.printer.command_error(
            "Invalid ads1220 reset state (got %s vs %s).\n"
            "This is generally indicative of connection problems\n"
            "(e.g. faulty wiring) or a faulty ADS1220 chip."
            % (hexify(val), hexify(RESET_STATE)))

    def setup_chip(self):
        continuous = 0x1  # enable continuous conversions
        mode = 0x2 if self.is_turbo else 0x0  # turbo mode
        sps_list = self.sps_turbo if self.is_turbo else self.sps_normal
        data_rate = list(sps_list.keys()).index(str(self.sps))
        reg_values = [(self.mux << 4) | (self.gain << 1) | int(self.pga_bypass),
                      (data_rate << 5) | (mode << 3) | (continuous << 2),
                      (self.vref << 6),
                      0x0]
        self.write_reg(0x0, reg_values)
        # start measurements immediately
        self.send_command(START_SYNC_CMD)

    def read_reg(self, reg, byte_count):
        # Add a small delay before SPI communication for stability
        time.sleep(0.001)
        read_command = [RREG_CMD | (reg << 2) | (byte_count - 1)]
        read_command += [NOOP_CMD] * byte_count
        
        if self.debug:
            logging.info("ADS1220 %s: Reading register 0x%x, bytes=%d, cmd=%s", 
                         self.name, reg, byte_count, hexify(read_command))
        
        try:
            params = self.spi.spi_transfer(read_command)
            response = bytearray(params['response'][1:])
            
            if self.debug:
                logging.info("ADS1220 %s: Read response: %s", self.name, hexify(response))
                # Check for all 0xFF which often indicates SPI communication failure
                if all(b == 0xff for b in response):
                    logging.warning("ADS1220 %s: All 0xFF response may indicate SPI failure", self.name)
                # Check for all 0x00 which could indicate incorrect CS handling
                elif all(b == 0x00 for b in response):
                    logging.warning("ADS1220 %s: All 0x00 response may indicate CS issue", self.name)
            
            return response
        except Exception as e:
            logging.error("ADS1220 %s: SPI read error: %s", self.name, str(e))
            # Return a safe fallback that will trigger a retry
            return bytearray([0xff] * byte_count)

    def send_command(self, cmd):
        # Add a small delay before SPI communication for stability
        time.sleep(0.001)
        
        if self.debug:
            cmd_name = "UNKNOWN"
            if cmd == RESET_CMD:
                cmd_name = "RESET"
            elif cmd == START_SYNC_CMD:
                cmd_name = "START_SYNC"
            elif cmd == NOOP_CMD:
                cmd_name = "NOOP"
            logging.info("ADS1220 %s: Sending command 0x%02x (%s)", self.name, cmd, cmd_name)
        
        try:
            self.spi.spi_send([cmd])
            if self.debug:
                logging.info("ADS1220 %s: Command sent successfully", self.name)
        except Exception as e:
            logging.error("ADS1220 %s: SPI command error: %s", self.name, str(e))
            # Re-raise to allow caller to handle the error
            raise

    def write_reg(self, reg, register_bytes):
        # Try write up to 3 times
        if self.debug:
            logging.info("ADS1220 %s: Writing to register 0x%x, value=%s", 
                         self.name, reg, hexify(register_bytes))
        
        for retry in range(3):
            if self.debug and retry > 0:
                logging.info("ADS1220 %s: Write attempt %d for register 0x%x", 
                             self.name, retry + 1, reg)
            try:
                write_command = [WREG_CMD | (reg << 2) | (len(register_bytes) - 1)]
                write_command.extend(register_bytes)
                
                if self.debug:
                    logging.info("ADS1220 %s: Write command: %s", self.name, hexify(write_command))
                
                self.spi.spi_send(write_command)
                
                # Add a small delay before verification
                time.sleep(0.002)
                
                if self.debug:
                    logging.info("ADS1220 %s: Verifying register 0x%x write", self.name, reg)
                
                stored_val = self.read_reg(reg, len(register_bytes))
                if bytearray(register_bytes) == stored_val:
                    if self.debug:
                        logging.info("ADS1220 %s: Register 0x%x write successful", self.name, reg)
                    return
                
                # If verification failed, log the attempt
                logging.warning("ADS1220 %s: Register 0x%x write verification failed (attempt %d): expected %s, got %s", 
                               self.name, reg, retry+1, hexify(register_bytes), hexify(stored_val))
                # Wait longer between retries
                time.sleep(0.010 * (retry + 1))
            except Exception as e:
                logging.error("ADS1220 %s: Register 0x%x write error on try %d: %s", 
                              self.name, reg, retry+1, str(e))
                time.sleep(0.010 * (retry + 1))
        
        # If we get here, all retries failed
        logging.error("ADS1220 %s: All write attempts failed for register 0x%x", self.name, reg)
        raise self.printer.command_error(
            "Failed to set ADS1220 register [0x%x] to %s: got %s. "
            "This may be a connection problem (e.g. faulty wiring)" % (
                reg, hexify(register_bytes), hexify(stored_val)))

    def cmd_ADS1220_TEST(self, gcmd):
        """G-code command to test ADS1220 SPI communication and hardware connections."""
        gcmd.respond_info("Starting ADS1220 hardware test on %s..." % self.name)
        
        # Test 1: Try to reset the chip
        gcmd.respond_info("Test 1: Sending RESET command...")
        try:
            self.send_command(RESET_CMD)
            gcmd.respond_info("Reset command sent successfully")
        except Exception as e:
            gcmd.respond_info("Error sending reset command: %s" % str(e))
        
        # Test 2: Try reading registers with different SPI modes
        gcmd.respond_info("Test 2: Reading registers with different SPI modes...")
        for mode in range(4):
            try:
                old_mode = self.spi.get_mode()
                gcmd.respond_info("Switching to SPI mode %d" % mode)
                self.spi.set_mode(mode)
                val = self.read_reg(0x0, 4)
                gcmd.respond_info("SPI mode %d result: %s" % (mode, hexify(val)))
                self.spi.set_mode(old_mode)
            except Exception as e:
                gcmd.respond_info("Error in SPI mode %d: %s" % (mode, str(e)))
                try:
                    self.spi.set_mode(old_mode)
                except:
                    pass
        
        # Test 3: Toggle CS pin manually
        gcmd.respond_info("Test 3: Testing CS pin control...")
        try:
            # Toggle CS pin high (inactive)
            gcmd.respond_info("Setting CS pin HIGH (inactive)")
            self.spi.get_command_send().send([self.spi.oid, self.spi.cmd_queue.CHIP_SELECT, 1])
            time.sleep(0.1)
            
            # Toggle CS pin low (active)
            gcmd.respond_info("Setting CS pin LOW (active)")
            self.spi.get_command_send().send([self.spi.oid, self.spi.cmd_queue.CHIP_SELECT, 0])
            time.sleep(0.1)
            
            # Return to inactive state
            gcmd.respond_info("Returning CS pin to HIGH (inactive)")
            self.spi.get_command_send().send([self.spi.oid, self.spi.cmd_queue.CHIP_SELECT, 1])
        except Exception as e:
            gcmd.respond_info("Error controlling CS pin: %s" % str(e))
        
        # Test 4: Try a simple SPI transfer
        gcmd.respond_info("Test 4: Performing simple SPI transfer...")
        try:
            # Send a NOOP command and read response
            self.spi.get_command_send().send([self.spi.oid, self.spi.cmd_queue.CHIP_SELECT, 0])
            resp = bytearray(4)
            self.spi.spi_transfer([NOOP_CMD], resp)
            gcmd.respond_info("SPI transfer response: %s" % hexify(resp))
            self.spi.get_command_send().send([self.spi.oid, self.spi.cmd_queue.CHIP_SELECT, 1])
        except Exception as e:
            gcmd.respond_info("Error in SPI transfer: %s" % str(e))
        
        gcmd.respond_info("ADS1220 hardware test complete")

    def register_test_command(self):
        """Register the ADS1220_TEST G-code command.
        This can be called directly or from a parent module like load_cell_probe."""
        gcode = self.printer.lookup_object('gcode')
        # Klipper G-code commands can only have one underscore, at the beginning
        # So we use ADS1220TEST as the base command name
        cmd_name = 'ADS1220TEST'
        if self.name != 'ads1220':
            # If this is not a direct ADS1220 instance, use a more specific name
            # Convert any underscores to avoid invalid command names
            safe_name = self.name.replace('_', '')
            cmd_name = 'ADS1220TEST' + safe_name.upper()
        gcode.register_command(cmd_name, self.cmd_ADS1220_TEST,
                              desc="Test ADS1220 SPI communication for " + self.name)
        logging.info("ADS1220 %s: Registered test command '%s'", 
                     self.name, cmd_name)

    def test_pins(self):
        """Test SPI pins and log the results"""
        logging.info("ADS1220 %s: Starting pin test...", self.name)
        
        # Log the pin configuration
        mcu_name = self.mcu.get_name()
        logging.info("ADS1220 %s: MCU: %s", self.name, mcu_name)
        
        # Get SPI pin information if available
        try:
            # Try to extract pin information from the SPI object
            spi_pins = self.spi.get_pin_info() if hasattr(self.spi, 'get_pin_info') else None
            if spi_pins:
                logging.info("ADS1220 %s: SPI pins - CS: %s, SCLK: %s, MOSI: %s, MISO: %s", 
                             self.name, spi_pins.get('cs', 'unknown'), 
                             spi_pins.get('sclk', 'unknown'),
                             spi_pins.get('mosi', 'unknown'),
                             spi_pins.get('miso', 'unknown'))
            else:
                # For software SPI, try to extract from config
                logging.info("ADS1220 %s: Using software SPI - pin info not directly available", self.name)
        except Exception as e:
            logging.info("ADS1220 %s: Error getting SPI pin info: %s", self.name, str(e))
        
        # Log data ready pin
        logging.info("ADS1220 %s: Data ready pin: %s", self.name, self.data_ready_pin)
        
        # Test CS pin toggling
        logging.info("ADS1220 %s: Testing CS pin control...", self.name)
        try:
            # Toggle CS pin high (inactive)
            self.spi.get_command_send().send([self.spi.oid, self.spi.cmd_queue.CHIP_SELECT, 1])
            time.sleep(0.1)
            
            # Toggle CS pin low (active)
            self.spi.get_command_send().send([self.spi.oid, self.spi.cmd_queue.CHIP_SELECT, 0])
            time.sleep(0.1)
            
            # Return to inactive state
            self.spi.get_command_send().send([self.spi.oid, self.spi.cmd_queue.CHIP_SELECT, 1])
            logging.info("ADS1220 %s: CS pin toggling successful", self.name)
        except Exception as e:
            logging.error("ADS1220 %s: Error controlling CS pin: %s", self.name, str(e))
        
        # Test basic SPI communication
        logging.info("ADS1220 %s: Testing basic SPI communication...", self.name)
        try:
            # Send a NOOP command and read response
            self.spi.get_command_send().send([self.spi.oid, self.spi.cmd_queue.CHIP_SELECT, 0])
            resp = bytearray(4)
            self.spi.spi_transfer([NOOP_CMD], resp)
            self.spi.get_command_send().send([self.spi.oid, self.spi.cmd_queue.CHIP_SELECT, 1])
            logging.info("ADS1220 %s: SPI transfer response: %s", self.name, hexify(resp))
            
            # Check for common error patterns
            if all(b == 0xff for b in resp):
                logging.warning("ADS1220 %s: All 0xFF response may indicate MISO pin disconnected or SPI failure", self.name)
            elif all(b == 0x00 for b in resp):
                logging.warning("ADS1220 %s: All 0x00 response may indicate MISO pin grounded or CS issue", self.name)
        except Exception as e:
            logging.error("ADS1220 %s: Error in SPI transfer: %s", self.name, str(e))
        
        logging.info("ADS1220 %s: Pin test complete", self.name)


ADS1220_SENSOR_TYPE = {"ads1220": ADS1220}
