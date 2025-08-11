# vl53l1x_plus.py
# Lightweight wrapper around your existing VL53L1X class.
# Adds: range status via raw I2C, spike rejection, EMA smoothing, easy config.

from smbus2 import i2c_msg
from VL53L1X import VL53L1X,VL53L1xDistanceMode,VL53L1xUserRoi,_TOF_LIBRARY
from collections import deque
from ctypes import c_uint32, c_uint8, pointer
import time
# ST register: RESULT__RANGE_STATUS (lower 5 bits hold status code)
REG_RESULT_RANGE_STATUS = 0x0089
REG_RESULT_INTERRUPT_STATUS = 0x0087
REG_SYSTEM_INTERRUPT_CLEAR  = 0x0086
REG_GPIO_TIO_HV_STATUS = 0x0031
REG_GPIO_HV_MUX_CTRL  = 0x0030

ALGO_CROSSTALK_PLANE = 0x0016
ALGO_CROSSTALK_XGRAD = 0x0018
ALGO_CROSSTALK_YGRAD = 0x001A

class VL53L1XPlus:
    def __init__(self, i2c_bus=15, i2c_address=0x29, tca9548a_num=255, tca9548a_addr=0):
        self.dev = VL53L1X(i2c_bus=i2c_bus, i2c_address=i2c_address,
                            tca9548a_num=tca9548a_num, tca9548a_addr=tca9548a_addr)
        self._i2c_bus   = None
        self._z_est     = None
        self.addr       = i2c_address
        self._buf       = deque(maxlen=5)
        # Define a larger region of interest (ROI)
        # VL53L1X has a 16x16 SPAD array, so setting to nearly full area
        # Format: top-left x,y to bottom-right x,y coordinates (0-15)
        self._roi = VL53L1xUserRoi(tlx=3, tly=3, brx=12, bry=12)

    def set_roi(self):
        self.dev.set_user_roi(self._roi)

    def get_timing(self):
        """
        Get the current timing budget and intermeasurement period.

        :return: A tuple (timing_budget, intermeasurement_period) where:
                 - timing_budget is in microseconds
                 - intermeasurement_period is in milliseconds
        """
        timing_budget = self.dev.get_timing()
        intermeasurement_period = None
        return timing_budget, intermeasurement_period
    
    def open(self,reset=False):
        self.dev.open(reset=reset)
        self._i2c = self.dev._i2c

    def close(self):
        self.dev.close()
        self._i2c = None

    def _get_int_active_level(self) -> int:
        # If bit4==0 → active-high (ready when bit0==1). If bit4==1 → active-low (ready when bit0==0).
        inv = (self._read_u8(REG_GPIO_HV_MUX_CTRL) >> 4) & 0x01
        return 0 if inv else 1  # value of bit0 that means "ready"
    
    def start(self, mode='short', timing_budget_ms=33, intermeasurement_ms=50):
        """Convenience: configure & start ranging."""
        mode_map = {
            'short':  VL53L1xDistanceMode.SHORT,
            'medium': VL53L1xDistanceMode.MEDIUM,
            'long':   VL53L1xDistanceMode.LONG
        }
        self.dev.stop_ranging()
        self.disable_xtalk()  # disable cross-talk compensation
        self.set_roi()  # set ROI before starting
        self.dev.set_distance_mode(mode_map[mode])
        self.dev.set_timing(int(timing_budget_ms*1000), int(intermeasurement_ms))  # µs, ms
        time.sleep(0.005)
        # IMPORTANT: start after all config
        self.dev.start_ranging(mode_map[mode]) 
    
    def disable_xtalk(self):
        self._write_u16(ALGO_CROSSTALK_PLANE, 0x0000)
        self._write_u16(ALGO_CROSSTALK_XGRAD, 0x0000)
        self._write_u16(ALGO_CROSSTALK_YGRAD, 0x0000)

    def stop(self):
        """Convenience: stop ranging."""
        self.dev.stop_ranging()

    def data_ready(self) -> bool:
        active = self._get_int_active_level()
        return (self._read_u8(REG_GPIO_TIO_HV_STATUS) & 0x01) == active

    def clear_interrupt(self):
        self._write_u8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)

    # i2c helpers, they must be redefined to use smbus2, since we do not want to touch the original VL53L1X class
    def _write_u16(self, reg, val):
        self._write_u8(reg,   (val >> 8) & 0xFF)
        self._write_u8(reg+1,  val       & 0xFF)

    def _write_u8(self, reg: int, val: int) -> None:
        msg = i2c_msg.write(self.addr, [(reg >> 8) & 0xFF, reg & 0xFF, val & 0xFF])
        self._i2c.i2c_rdwr(msg)

    def _read_u8(self, reg16):
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
    

if __name__ == '__main__':
    import sys, time

    sensor = VL53L1XPlus()
    sensor.open()

    # set ROI before start if needed
    # sensor.set_roi(...)

    sensor.start(mode='short', timing_budget_ms=33, intermeasurement_ms=50)
    time.sleep(0.2)
    print("VL53L1XPlus sensor initialized.")

    timing_budget, intermeasurement_period = sensor.get_timing()
    print(f"Timing budget: {timing_budget} µs, Intermeasurement period: {intermeasurement_period} ms")

    try:
        while True:
            print("Waiting for data...")
            time.sleep(0.1)  # wait for data to be ready
            # if sensor.data_ready():
            distance = sensor.get_distance_mm()
            status = sensor.get_range_status()
            print(f"Distance: {distance} mm, Status: {status}")
            # sensor.clear_interrupt()
            time.sleep(0.005)  # small poll delay
    except KeyboardInterrupt:
        print("Exiting for error")
        print(KeyboardInterrupt)
        pass
    finally:
        sensor.stop()
        sensor.close()
        print("Sensor stopped and closed.")
        sys.exit(0)
