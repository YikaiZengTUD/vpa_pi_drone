__author__ = "Aldo Vargas and Stefanie Tellex"
__copyright__ = "Copyright 2014 Altax.net, 2017"

__license__ = "GPL"
__version__ = "1.5"

# rewrite to py3

from email import header
import serial, time, struct
import numpy as np
from threading import Lock
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
    def __str__(self):
        return self.__repr__()
    def __repr__(self):
        return "PID(%f, %f, %f)" % (self.kp, self.ki, self.kd)
    
class MultiWii:
    REBOOT             = 68
    IDENT              = 100
    STATUS             = 101
    RAW_IMU            = 102
    POS_EST            = 123
    SERVO              = 103
    MOTOR              = 104
    RC                 = 105
    RAW_GPS            = 106
    COMP_GPS           = 107
    ATTITUDE           = 108
    ALTITUDE           = 109
    ANALOG             = 110
    RC_TUNING          = 111
    PID                = 112
    BOX                = 113
    MISC               = 114
    MOTOR_PINS         = 115
    BOXNAMES           = 116
    PIDNAMES           = 117
    WP                 = 118
    BOXIDS             = 119
    RC_RAW_IMU         = 121
    SET_RAW_RC         = 200
    SET_RAW_GPS        = 201
    SET_PID            = 202
    SET_BOX            = 203
    SET_RC_TUNING      = 204
    ACC_CALIBRATION    = 205
    MAG_CALIBRATION    = 206
    SET_MISC           = 207
    RESET_CONF         = 208
    SET_WP             = 209
    SWITCH_RC_SERIAL   = 210
    IS_SERIAL          = 211
    MSP_ARMING_DISABLE_FLAGS = 242
    DEBUG              = 254
    EEPROM_WRITE       = 250

    # Struct definitions
    RAW_IMU_STRUCT     = struct.Struct('<hhhhhhhhh')
    ATTITUDE_STRUCT    = struct.Struct('<hhh')
    PID_STRUCT         = struct.Struct('<BBBBBBBBBBBBBBBBBBBBBBBBBBBBBB')

    SEND_ZERO_STRUCT1  = struct.Struct('<2B%dh' % 0)
    SEND_EIGHT_STRUCT1 = struct.Struct('<2B%dh' % 8)
    # Structs for sending data
    codeS              = struct.Struct('<B')
    footerS            = struct.Struct('B')
    emptyString        = b""
    headerString       = b"$M<"


    def __init__(self,serPort):
        
        self.attitude   = {'angx':0,'angy':0,'heading':0,'elapsed':0,'timestamp':0}
        self.analog     = {'vbat':0,'intPowerMeterSum':0,'rssi':0,'amperage':0}
        self.status     = {'cycleTime':0,'i2c_errors_count':0,'sensor':0,'flag':0,'global_conf.currentSet':0,'timestamp':0}
        self.ident      = {'version':0,'multitype':0,'msp_version':0,'capability':0,'timestamp':0}
        self.rc         = {'cmd':0,'roll':0,'pitch':0,'yaw':0,'throttle':0,'aux1':0,'aux2':0,'aux3':0,'aux4':0,'timestamp':0}
        self.posest     = {'cmd':0,'x':0,'y':0,'z':0,'elapsed':0,'timestamp':0}
        self.motor      = {'cmd':0,'m1':0,'m2':0,'m3':0,'m4':0,'m5':0,'m6':0,'m7':0,'m8':0,'elapsed':0,'timestamp':0}
        self.pid        = {}
        self.rawIMU     = {}

        self.serPort = serPort


        self.ser = serial.Serial(serPort,
                            baudrate=115200,
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            timeout=1,
                            xonxoff=False,
                            rtscts=False,
                            dsrdtr=False,
                            writeTimeout=2,
                        )

        self.serial_port_write_lock  = Lock()
        self.serial_port_read_lock   = Lock()

    def reboot(self):
        self.send_raw_rc(0, MultiWii.REBOOT, [])
        print("Sent MSP_REBOOT command.")
        
    def create_raw_rc_packet(self,channels):

        """
        Create a MSP packet for RAW_RC command with given channel values.
        channels: List of 8 integers, each between 1000 and 2000 representing RC channel values.
        """

        header = b'$M<'  # MSP header
        code = MultiWii.SET_RAW_RC  # MSP code for RAW_RC
        data_length = 16  # Length of data for 8 channels, each 2 bytes

        data = struct.pack('<8H', *channels)

        # Calculate the checksum: XOR of size, code, and all bytes in data
        checksum = data_length ^ code
        for byte in data:
            checksum ^= byte

        checksum &= 0xFF

        packet = header + struct.pack('<BB', data_length, code) + data + struct.pack('<B', checksum)

        return packet

    def send_raw_rc(self, data_length, code, data):
        with self.serial_port_write_lock:
            if code == MultiWii.SET_RAW_RC:
                packet = self.create_raw_rc_packet(data)
            else:
                dl = len(data)
                if dl == 0:
                    s1 = MultiWii.SEND_ZERO_STRUCT1
                elif dl == 8:
                    s1 = MultiWii.SEND_EIGHT_STRUCT1
                else:
                    s1 = struct.Struct('<2B%dh' % len(data))

                dataString = s1.pack(data_length, code, *data)
                b = np.frombuffer(dataString, dtype=np.uint8)
                checksum = np.bitwise_xor.accumulate(b)[-1]
                footerString = MultiWii.footerS.pack(checksum)
                packet = MultiWii.emptyString.join([MultiWii.headerString, dataString, footerString])

            self.ser.write(packet)
            self.ser.flush() # I add this

    def receiveDataPacket(self, timeout=2.0):
        with self.serial_port_read_lock:
            start = time.time()
            # Resync to header: look for b'$M>'
            while True:
                if time.time() - start > timeout:
                    print("Timeout waiting for header")
                    return None
                first = self.ser.read(1)
                if first == b'':
                    print("Read Timeout but 0 data")
                    return None
                if first == b'$':
                    second = self.ser.read(1)
                    third = self.ser.read(1)
                    if second == b'M' and third == b'>':
                        break
            # Now read data length and code
            datalength  = self.ser.read(1)
            code        = self.ser.read(1)
            checksum    = self.ser.read(1)
            if len(datalength) == 0 or len(code) == 0:
                print("Timeout reading datalength or code")
                return None
            datalength = datalength[0]
            code = code[0]
            data = self.ser.read(datalength)
            if len(data) != datalength:
                print("Timeout reading data payload")
                return None
            readTime = time.time()
            elapsed = readTime - start
            # REDUCED SOME PROCESSING
            if code == MultiWii.ATTITUDE:
                attitude = MultiWii.ATTITUDE_STRUCT.unpack(data)
                self.attitude['cmd']        = code 
                self.attitude['angx']       = attitude[0]/10.0
                self.attitude['angy']       = attitude[1]/10.0
                self.attitude['heading']    = attitude[2]
                self.attitude['elapsed']    = elapsed
                self.attitude['timestamp']  = readTime
                return self.attitude
            elif code == MultiWii.ANALOG:
                analog = struct.unpack('<'+'B2HhH', data)
                self.analog['vbat']             = analog[0]
                self.analog['intPowerMeterSum'] = analog[1]
                self.analog['rssi']             = analog[2]
                self.analog['amperage']         = analog[3]
                self.analog['timestamp']        = readTime
                return self.analog
            elif code == MultiWii.STATUS:
                if datalength == 11:
                    # Old format
                    status = struct.unpack('<HHHIb', data)
                    self.status['cycleTime']                = status[0]
                    self.status['i2c_errors_count']         = status[1]
                    self.status['sensor']                   = status[2]
                    self.status['flag']                     = status[3]
                    self.status['global_conf.currentSet']   = status[4]
                    self.status['timestamp']                = readTime
                    return self.status
                elif datalength == 22:
                    # New format
                    status = struct.unpack('<HHIIBIIB', data)
                    self.status['cycleTime']                = status[0]
                    self.status['i2c_errors_count']         = status[1]
                    self.status['sensor']                   = status[2]
                    self.status['flag']                     = status[3]
                    self.status['global_conf.currentSet']   = status[4]
                    self.status['new_field_1']              = status[5]
                    self.status['new_field_2']              = status[6]
                    self.status['timestamp']                = readTime
                    return self.status
                else:
                    print("Unknown STATUS packet length:", datalength)
                    print("Raw data:", data.hex())
            elif code == MultiWii.ACC_CALIBRATION:
                print("ACC Calibration Data Received")
            elif code == MultiWii.IDENT:
                ident = struct.unpack('<'+'BBBI',data)
                self.ident['cmd']            = code
                self.ident['version']        = ident[0]
                self.ident['multitype']      = ident[1]
                self.ident['msp_version']    = ident[2]
                self.ident['capability']     = ident[3]
                self.ident['timestamp']      = readTime
                return self.ident
            elif code == MultiWii.RC:
                rc = struct.unpack('<'+'hhhhhhhhhhhh',data)
                self.rc['cmd']               = code
                self.rc['roll']              = rc[0]
                self.rc['pitch']             = rc[1]
                self.rc['yaw']               = rc[2]
                self.rc['throttle']          = rc[3]
                self.rc['aux1']              = rc[4]
                self.rc['aux2']              = rc[5]
                self.rc['aux3']              = rc[6]
                self.rc['aux4']              = rc[7]
                self.rc['timestamp']         = readTime
                return self.rc
            elif code == MultiWii.PID:
                pid = MultiWii.PID_STRUCT.unpack(data)
                self.pid['roll']    = PID(pid[0], pid[1], pid[2])
                self.pid['pitch']   = PID(pid[3], pid[4], pid[5])
                self.pid['yaw']     = PID(pid[6], pid[7], pid[8])
                self.pid['alt']     = PID(pid[9], pid[10], pid[11])
                self.pid['pos']     = PID(pid[12], pid[13], pid[14])
                self.pid['posr']    = PID(pid[15], pid[16], pid[17])
                self.pid['navr']    = PID(pid[18], pid[19], pid[20])
                self.pid['level']   = PID(pid[21], pid[22], pid[23])
                self.pid['mag']     = PID(pid[24], pid[25], pid[26])
                self.pid['vel']     = PID(pid[27], pid[28], pid[29])
                return self.pid
            elif code == MultiWii.RAW_IMU:
                imu = MultiWii.RAW_IMU_STRUCT.unpack(data)
                self.rawIMU['cmd']        = code
                self.rawIMU['ax']         = imu[0]
                self.rawIMU['ay']         = imu[1]
                self.rawIMU['az']         = imu[2]
                self.rawIMU['gx']         = imu[3]
                self.rawIMU['gy']         = imu[4]
                self.rawIMU['gz']         = imu[5]
                self.rawIMU['timestamp']  = readTime
                return self.rawIMU
            elif code == MultiWii.POS_EST:
                pos = struct.unpack('<'+'hhh',data)
                self.posest['cmd']        = code
                self.posest['x']          = pos[0]
                self.posest['y']          = pos[1]
                self.posest['z']          = pos[2]
                self.posest['elapsed']    = elapsed
                self.posest['timestamp']  = readTime
                return self.posest
            elif code == MultiWii.MOTOR:
                motor = struct.unpack('<'+'hhhhhhhh',data)
                self.motor['cmd']        = code
                self.motor['m1'] = motor[0]
                self.motor['m2'] = motor[1]
                self.motor['m3'] = motor[2]
                self.motor['m4'] = motor[3]
                self.motor['m5'] = motor[4]
                self.motor['m6'] = motor[5]
                self.motor['m7'] = motor[6]
                self.motor['m8'] = motor[7]
                self.motor['elapsed']    = elapsed
                self.motor['timestamp']  = readTime
                return self.motor
            elif code == MultiWii.SET_RAW_RC:
                return code
            elif code == MultiWii.MSP_ARMING_DISABLE_FLAGS:
                return code,data
            else:
                print("Unknown command received, code:", code)

        return None
    def getData(self,cmd):
        self.send_raw_rc(0, cmd, [])
        return self.receiveDataPacket()

    def calibrateACC(self,fname):
        self.send_raw_rc(0, MultiWii.ACC_CALIBRATION, [])
        self.receiveDataPacket()
        for i in range(200):
            # ignore 200 unsettle readings
            raw_imu = self.getData(MultiWii.RAW_IMU)
            time.sleep(0.01)
        raw_imu_totals = {}
        samples = 0.0
        for i in range(1000):
            raw_imu = self.getData(MultiWii.RAW_IMU)
            print(raw_imu)
            if raw_imu != None:
                for key, value in raw_imu.items():
                    raw_imu_totals.setdefault(key, 0.0)
                    raw_imu_totals[key] += value
                samples += 1
                time.sleep(0.01)

        for key, value in raw_imu_totals.items():
            raw_imu_totals[key] = raw_imu_totals[key] / samples

        import yaml
        with open(fname, "w") as f:
            yaml.dump(raw_imu_totals, f)

    def close(self):
        # print(f"self.ser: {self.ser}, is_open: {getattr(self.ser, 'is_open', None)}")
        if self.ser and self.ser.is_open:
            # ...existing code...
            try:
                print("Closing serial port...")
                self.ser.timeout = 0.1
                self.ser.writeTimeout = 0.1
                self.ser.close()
                print("Serial port closed.")
            except Exception as e:
                print(f"Exception during serial close: {e}")

    def get_arming_disable_flags(self):
        self.send_raw_rc(0, MultiWii.MSP_ARMING_DISABLE_FLAGS, [])
        while True:
            raw = self.receiveDataPacket()
            print("Received for arming disable flags:", raw)
            if raw is None:
                print("No response received for arming disable flags.")
                return None
            try:
                code = raw[0]
                if code == MultiWii.MSP_ARMING_DISABLE_FLAGS:
                    data = raw[1:]
                    return code, data
                else:
                    print(f"Received unexpected code: {code}, expected {MultiWii.MSP_ARMING_DISABLE_FLAGS}")
            except (IndexError, ValueError, TypeError):
                print("Error unpacking arming disable flags response, not arm disable packet")
            break
            
def decode_status_flag(flag):
    flags = []
    if flag & (1 << 0): flags.append("ARMED")
    if flag & (1 << 1): flags.append("ANGLE_MODE")
    if flag & (1 << 2): flags.append("HORIZON_MODE")
    # ... add more as needed ...
    if flag & (1 << 19): flags.append("GPS_FIX")
    if flag & (1 << 16): flags.append("FAILSAFE")
    return flags

def decode_arming_disable_flags(flags):
    reasons = []
    if flags & (1 << 0): reasons.append("NOGYRO")
    if flags & (1 << 1): reasons.append("FAILSAFE")
    if flags & (1 << 2): reasons.append("RXLOSS")
    if flags & (1 << 3): reasons.append("BADRX")
    if flags & (1 << 4): reasons.append("BOXFAILSAFE")
    if flags & (1 << 5): reasons.append("RUNAWAY_TAKEOFF")
    if flags & (1 << 6): reasons.append("THROTTLE")
    if flags & (1 << 7): reasons.append("ANGLE")
    if flags & (1 << 8): reasons.append("BOOTGRACE")
    if flags & (1 << 9): reasons.append("NOPREARM")
    if flags & (1 << 10): reasons.append("LOAD")
    if flags & (1 << 11): reasons.append("CALIB")
    if flags & (1 << 12): reasons.append("CLI")
    if flags & (1 << 13): reasons.append("CMSMENU")
    if flags & (1 << 14): reasons.append("OSD")
    if flags & (1 << 15): reasons.append("BLACKBOX")
    # if flags & (1 << 16): reasons.append("MSP")
    # if flags & (1 << 17): reasons.append("PARALYZE")
    # if flags & (1 << 18): reasons.append("GPS")
    # if flags & (1 << 19): reasons.append("RESCUE_SW")
    if flags & (1 << 20): reasons.append("RPMFILTER")
    if flags & (1 << 21): reasons.append("ACROTRAINER")
    if flags & (1 << 22): reasons.append("DYNAMIC_IDLE")
    if flags & (1 << 23): reasons.append("ARM_SWITCH")
    if flags & (1 << 24): reasons.append("PREARM_SWITCH")
    if flags & (1 << 25): reasons.append("USB")
    if flags & (1 << 26): reasons.append("SOFTSERIAL")
    if flags & (1 << 27): reasons.append("PIDLOOP")
    if flags & (1 << 28): reasons.append("AIRMODE")
    if flags & (1 << 29): reasons.append("SENSOR")
    if flags & (1 << 30): reasons.append("ANTI_GRAVITY")
    if flags & (1 << 31): reasons.append("DEBUG")
    return reasons

if __name__ == "__main__":
    # This will be a communication check example
    # Will attempt to ARM the drone

    from __init__ import arm_cmd,disarm_cmd
    from tqdm import tqdm
    
    poll_freq = 50

    Board = MultiWii('/dev/ttyACM0')
    try:
        # first stream disarm command
        count = 2 / (1 / poll_freq)
        for i in range(int(count)):
            Board.send_raw_rc(16, MultiWii.SET_RAW_RC, disarm_cmd)
            ack = Board.receiveDataPacket()

            time.sleep(1 / poll_freq)

        print("Disarm command sent")
        count = 20 / (1 / poll_freq)
        
        # Progress bar for arming sequence
        print("Attempting to arm...")
        for i in tqdm(range(int(count)), desc="Arming test", unit="cmd"):
            Board.send_raw_rc(16, MultiWii.SET_RAW_RC, arm_cmd)
            ack = Board.receiveDataPacket()
            
            time.sleep(1 / poll_freq)

            Board.getData(MultiWii.STATUS)
            reasons = decode_arming_disable_flags(Board.status['flag'])
            if len(reasons) > 0:
                print("Arming disable reasons:", reasons)

        count = 2 / (1 / poll_freq)
        for i in range(int(count)):
            Board.send_raw_rc(16, MultiWii.SET_RAW_RC, disarm_cmd)
            ack = Board.receiveDataPacket()
        time.sleep(1 / poll_freq)
    except KeyboardInterrupt:
        print("Keyboard Interrupt, closing connection")
    finally:
        Board.send_raw_rc(16, MultiWii.SET_RAW_RC, disarm_cmd)
        Board.close()
        print("Connection closed")
