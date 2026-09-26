import math
import struct
import smbus2
import time

# I2C Default Address
DEFAULT_ADDRESS = 0x68

# MPU-6500 Register Map
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_TEMP_OUT_H = 0x41
REG_GYRO_XOUT_H = 0x43
REG_PWR_MGMT_1 = 0x6B
REG_WHO_AM_I = 0x75
 
_VALID_WHO_AM_I = (0x70, 0x71, 0x73)

ACCEL_FS_SEL_2G = 0b00000000
ACCEL_FS_SEL_4G = 0b00001000
ACCEL_FS_SEL_8G = 0b00010000
ACCEL_FS_SEL_16G = 0b00011000

GYRO_FS_SEL_250DPS = 0b00000000
GYRO_FS_SEL_500DPS = 0b00001000
GYRO_FS_SEL_1000DPS = 0b00010000
GYRO_FS_SEL_2000DPS = 0b00011000

_ACCEL_SO = {
    ACCEL_FS_SEL_2G: 16384.0,
    ACCEL_FS_SEL_4G: 8192.0,
    ACCEL_FS_SEL_8G: 4096.0,
    ACCEL_FS_SEL_16G: 2048.0,
}
 
_GYRO_SO = {
    GYRO_FS_SEL_250DPS: 131.0,
    GYRO_FS_SEL_500DPS: 65.5,
    GYRO_FS_SEL_1000DPS: 32.8,
    GYRO_FS_SEL_2000DPS: 16.4,
}

_DPS_TO_RADIANS = math.pi / 180.0
_G_TO_MS2 = 9.80665

class MPU6500():
    def __init__(self,
                 i2c_bus=1,
                 address=DEFAULT_ADDRESS,
                 accel_fs=ACCEL_FS_SEL_2G,
                 gyro_fs=GYRO_FS_SEL_250DPS) -> None:
        self.address = address
        self.bus = smbus2.SMBus(i2c_bus)
        self.accel_fs = accel_fs
        self.gyro_fs = gyro_fs
        self.accel_so = _ACCEL_SO[accel_fs]
        self.gyro_so = _GYRO_SO[gyro_fs]
        
        self.initialize()
    
    def initialize(self) -> None:
        who_am_i = self.bus.read_byte_data(self.address, REG_WHO_AM_I)
        
        if who_am_i not in _VALID_WHO_AM_I:
            raise RuntimeError(
                f"Device unknown (0x{who_am_i:02X})"
            )
        
        self.bus.write_byte_data(self.address, REG_PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        
        self.bus.write_byte_data(self.address, REG_GYRO_CONFIG, self.gyro_fs)
        self.bus.write_byte_data(self.address, REG_ACCEL_CONFIG, self.accel_fs)
        time.sleep(0.1)
    
    def _read_words(self, register: int, count: int) -> tuple:
        data = self.bus.read_i2c_block_data(self.address, register, count * 2)
        return struct.unpack(">" + "h" * count, bytes(data))
    
    @property
    def acceleration(self) -> tuple:
        raw_x, raw_y, raw_z = self._read_words(REG_ACCEL_XOUT_H, 3)
        
        return (
            (raw_x / self.accel_so) * _G_TO_MS2,
            (raw_y / self.accel_so) * _G_TO_MS2,
            (raw_z / self.accel_so) * _G_TO_MS2,
        )
    
    @property
    def temperature(self) -> float:
        (raw_temp, ) = self._read_words(REG_TEMP_OUT_H, 1)

        return (raw_temp / 333.87) + 21.0
    
    @property
    def gyro(self) -> tuple:
        
        raw_x, raw_y, raw_z = self._read_words(REG_GYRO_XOUT_H, 3)
        
        return (
            (raw_x / self.gyro_so) * _DPS_TO_RADIANS,
            (raw_y / self.gyro_so) * _DPS_TO_RADIANS,
            (raw_z / self.gyro_so) * _DPS_TO_RADIANS,
        )

    def close(self) -> None:
        if hasattr(self, 'bus') and self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
    