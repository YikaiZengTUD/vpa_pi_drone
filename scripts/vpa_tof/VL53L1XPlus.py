# vl53l1x_plus.py
# Lightweight wrapper around your existing VL53L1X class.
# Adds: range status via raw I2C, spike rejection, EMA smoothing, easy config.

from smbus2 import i2c_msg
from VL53L1X import VL53L1X,VL53L1xDistanceMode,VL53L1xUserRoi,_TOF_LIBRARY
from collections import deque
from ctypes import c_uint32, c_uint8, pointer
# ST register: RESULT__RANGE_STATUS (lower 5 bits hold status code)
REG_RESULT_RANGE_STATUS = 0x0089

class VL53L1XPlus:
    def __init__(self, i2c_bus=1, i2c_address=0x29, tca9548a_num=255, tca9548a_addr=0):
        self.dev = VL53L1X(i2c_bus=i2c_bus, i2c_address=i2c_address,
                            tca9548a_num=tca9548a_num, tca9548a_addr=tca9548a_addr)
        self._i2c_bus = None
        self._z_est = None
        self.addr = i2c_address
        self._buf = deque(maxlen=5)
        self._roi = VL53L1xUserRoi(tlx=0, tly=15, brx=15, bry=0)

    def set_roi(self):
        self.dev.set_roi(self._roi)

    def get_timing(self):
        """
        Get the current timing budget and intermeasurement period.

        :return: A tuple (timing_budget, intermeasurement_period) where:
                 - timing_budget is in microseconds
                 - intermeasurement_period is in milliseconds
        """
        timing_budget = self.dev.get_timing_budget()
        # Get the intermeasurement period using the C library function
        intermeasurement_period = c_uint32(0)
        status = _TOF_LIBRARY.VL53L1_GetInterMeasurementPeriodMilliSeconds(
            self.dev._dev, 
            pointer(intermeasurement_period)
        )
        if status != 0:
            # If there's an error, return None for intermeasurement_period
            intermeasurement_period = None
        else:
            # Convert from ctypes to regular Python int
            intermeasurement_period = intermeasurement_period.value
        return timing_budget, intermeasurement_period
    def open(self,reset=False):
        self.dev.open(reset=reset)
        self._i2c = self.dev._i2c

    def close(self):
        self.dev.close()
        self._i2c = None

    def start(self, mode='short', timing_budget_ms=33, intermeasurement_ms=50):
        """Convenience: configure & start ranging."""
        mode_map = {'short': VL53L1xDistanceMode.SHORT,
                    'medium': VL53L1xDistanceMode.MEDIUM,
                    'long': VL53L1xDistanceMode.LONG}
        self.dev.set_distance_mode(mode_map[mode])
        self.dev.set_timing(timing_budget_ms * 1000, intermeasurement_ms)
        self.dev.start_ranging(mode_map[mode])

    def stop(self):
        """Convenience: stop ranging."""
        self.dev.stop_ranging()

    def _read_u8(self,reg16):
        msg_w = i2c_msg.write(self.addr, [(reg16 >> 8) & 0xFF, reg16 & 0xFF])
        msg_r = i2c_msg.read(self.addr, 1)
        self._i2c.i2c_rdwr(msg_w, msg_r)
        b = msg_r.buf[0][0] if hasattr(msg_r.buf[0], '__getitem__') else ord(msg_r.buf[0])
        return b
    
    def get_range_status(self):
        """Read the range status from the sensor."""
        status = self._read_u8(REG_RESULT_RANGE_STATUS)
        return status & 0x1F
    
    def get_distance_mm(self):
        """Raw distance from the vendor wrapper (mm). May be 0 when invalid."""
        return int(self.dev.get_distance())
    
    def get_user_roi(self):
        """
        Return current ROI as (top_left_x, top_left_y, bot_right_x, bot_right_y).
        """
        # Create an empty VL53L1xUserRoi instance
        roi = VL53L1xUserRoi()
        # Call the C function through the underlying lib
        _TOF_LIBRARY.getUserRoi(self.dev._dev, 
                                pointer(c_uint8(roi.top_left_x)),
                                pointer(c_uint8(roi.top_left_y)),
                                pointer(c_uint8(roi.bot_right_x)),
                                pointer(c_uint8(roi.bot_right_y)))
        return roi.top_left_x, roi.top_left_y, roi.bot_right_x, roi.bot_right_y

if __name__ == '__main__':
    import time
    import sys
    sensor = VL53L1XPlus()
    sensor.open()
    sensor.set_roi()
    sensor.start(mode='short', timing_budget_ms=33, intermeasurement_ms=50)
    print("VL53L1XPlus sensor initialized.")
    time.sleep(0.2)
    print("Reading settings...")
    roi_read = sensor.get_user_roi()
    print(f"Current ROI: {roi_read}")
    timing_budget, intermeasurement_period = sensor.get_timing()
    print(f"Timing budget: {timing_budget} µs, Intermeasurement period: {intermeasurement_period} ms")

    try:
        while True:
            distance = sensor.get_distance_mm()
            status = sensor.get_range_status()
            print(f"Distance: {distance} mm, Status: {status}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        sensor.stop()
        sensor.close()
        print("Sensor stopped and closed.")
        sys.exit(0)