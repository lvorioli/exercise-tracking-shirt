# Luke Orioli
from machine import Pin, SPI
import utime
import struct

class LSM6DSO:
    class REGISTER:
        FUNC_CFG_ACCESS = 0x01
        PIN_CTRL = 0x02
        FIFO_CTRL1 = 0x07
        FIFO_CTRL2 = 0x08
        FIFO_CTRL3 = 0x09
        FIFO_CTRL4 = 0x0A
        COUNTER_BDR_REG1 = 0x0B
        COUNTER_BDR_REG2 = 0x0C
        INT1_CTRL = 0x0D
        INT2_CTRL = 0x0E
        WHO_AM_I = 0x0F
        CTRL1_XL = 0x10
        CTRL2_G = 0x11
        CTRL3_C = 0x12
        CTRL4_C = 0x13
        CTRL5_C = 0x14
        CTRL6_C = 0x15
        CTRL7_G = 0x16
        CTRL8_XL = 0x17
        CTRL9_XL = 0x18
        CTRL10_C = 0x19
        ALL_INT_SRC = 0x1A
        WAKE_UP_SRC = 0x1B
        TAP_SRC = 0x1C
        D6D_SRC = 0x1D
        STATUS_REG = 0x1E
        STATUS_SPIAux = 0x1E
        OUT_TEMP_L = 0x20
        OUT_TEMP_H = 0x21
        OUTX_L_G = 0x22
        OUTX_H_G = 0x23
        OUTY_L_G = 0x24
        OUTY_H_G = 0x25
        OUTZ_L_G = 0x26
        OUTZ_H_G = 0x27
        OUTX_L_A = 0x28
        OUTX_H_A = 0x29
        OUTY_L_A = 0x2A
        OUTY_H_A = 0x2B
        OUTZ_L_A = 0x2C
        OUTZ_H_A = 0x2D
        EMB_FUNC_STATUS_MAINPAGE = 0x35
        FSM_STATUS_A_MAINPAGE = 0x36
        FSM_STATUS_B_MAINPAGE = 0x37
        STATUS_MASTER_MAINPAGE = 0x39
        FIFO_STATUS1 = 0x3A
        FIFO_STATUS2 = 0x3B
        TIMESTAMP0 = 0x40
        TIMESTAMP1 = 0x41
        TIMESTAMP2 = 0x42
        TIMESTAMP3 = 0x43
        TAP_CFG0 = 0x56
        TAP_CFG1 = 0x57
        TAP_CFG2 = 0x58
        TAP_THS_6D = 0x59
        INT_DUR2 = 0x5A
        WAKE_UP_THS = 0x5B
        WAKE_UP_DUR = 0x5C
        FREE_FALL = 0x5D
        MD1_CFG = 0x5E
        MD2_CFG = 0x5F
        I3C_BUS_ABV = 0x62
        INTERNAL_FREQ_FINE = 0x63
        INT_OIS = 0x6F
        CTRL1_OIS = 0x70
        CTRL2_OIS = 0x71
        CTRL3_OIS = 0x72
        X_OFS_USR = 0x73
        Y_OFS_USR = 0x74
        Z_OFS_USR = 0x75
        FIFO_DATA_OUT_TAG = 0x78
        FIFO_DATA_OUT_X_L = 0x79
        FIFO_DATA_OUT_X_H = 0x7A
        FIFO_DATA_OUT_Y_L = 0x7B
        FIFO_DATA_OUT_Y_H = 0x7C
        FIFO_DATA_OUT_Z_L = 0x7D
        FIFO_DATA_OUT_Z_H = 0x7E

    class REGISTER_IO_OPERATION:
        READ = 0x01
        WRITE = 0x00
    
    class AXIS:
        X = 0
        Y = 1
        Z = 2
    
    def __init__(self, spi_peripheral: SPI, cs: Pin):
        self.spi = spi_peripheral
        self.cs = cs
        self.acc_data_ready_flag = False
        self.gyro_data_ready_flag = False

    def configure(self):
        # Enable Block Data Update mode so that the data registers are not updated while reading them.
        # Also keep register auto-increment enabled.
        ctrl3_c_reg = 0x42
        assert(self.write(LSM6DSO.REGISTER.CTRL3_C, bytearray(ctrl3_c_reg.to_bytes(1, 'little'))))

        utime.sleep_ms(10)

        # Set the accelerometer's full scale selection to +-2g and the output data rate to 1667 Hz.
        ctrl1_xl_reg = 0x80
        assert(self.write(LSM6DSO.REGISTER.CTRL1_XL, bytearray(ctrl1_xl_reg.to_bytes(1, 'little'))))

        utime.sleep_ms(10)

        # Set the gyroscope's full scale selection to +- 250 and the output data rate to 1667 Hz.
        ctrl2_g_reg = 0x80
        assert(self.write(LSM6DSO.REGISTER.CTRL2_G, bytearray(ctrl2_g_reg.to_bytes(1, 'little'))))

        utime.sleep_ms(10)
    
    def read(self, reg: REGISTER, num_bytes: int = 1) -> bytearray:
        # Create the command frame that instructs the IMU to read from the specified register.
        command_frame = (LSM6DSO.REGISTER_IO_OPERATION.READ << 7 | reg).to_bytes(1, 'little')
        # Assert the IMU's chip select line.
        self.cs(0)
        # Instruct the IMU to read from the specified register.
        self.spi.write(command_frame)
        # Read the specified number of bytes starting at the specified register address.
        rx_bytes = self.spi.read(num_bytes)
        # Deassert the IMU's chip select line.
        self.cs(1)
        # Return the read bytes.
        return rx_bytes

    def write(self, reg: REGISTER, buffer: bytearray, verify: bool = True) -> bool:
        # Create the command frame that instructs the IMU to write to the specified register.
        command_frame = (LSM6DSO.REGISTER_IO_OPERATION.WRITE << 7 | reg).to_bytes(1, 'little')
        # Combine the command frame and buffer bytes.
        tx_bytes = bytes(bytearray(command_frame) + buffer)
        # Assert the IMU's chip select line.
        self.cs(0)
        # Instruct the IMU to write to the specified register and send the register contents.
        self.spi.write(tx_bytes)
        # Deassert the IMU's chip select line.
        self.cs(1)
        # Verify that the bytes were properly written to the IMU's register(s) if requested.
        if verify is True:
            utime.sleep_ms(10)
            # Read the IMU's register contents.
            contents = self.read(reg, len(buffer))
            # Indicate whether the contents match.
            return buffer == contents
        return True
    
    def read_acc_axis(self, axis: AXIS) -> float:
        reg = LSM6DSO.REGISTER.OUTX_L_A + 2 * axis
        acc_axis_bytes = self.read(reg, 2)
        if len(acc_axis_bytes) > 0:
            acc_axis_raw, = struct.unpack('<h', acc_axis_bytes)
            acc_axis_g = acc_axis_raw * 0.061 / 1000.0
            return acc_axis_g
        return None
    
    def read_gyro_axis(self, axis: AXIS) -> float:
        reg = LSM6DSO.REGISTER.OUTX_L_G + 2 * axis
        gyro_axis_bytes = self.read(reg, 2)
        if len(gyro_axis_bytes) > 0:
            gyro_axis_raw, = struct.unpack('<h', gyro_axis_bytes)
            gyro_axis_dps = gyro_axis_raw * 8.75 / 1000.0
            return gyro_axis_dps
        return None
    
    def read_acc(self) -> tuple[float, float, float]:
        self.acc = (self.read_acc_axis(LSM6DSO.AXIS.X), self.read_acc_axis(LSM6DSO.AXIS.Y), self.read_acc_axis(LSM6DSO.AXIS.Z))
        return self.acc

    def read_gyro(self) -> tuple[float, float, float]:
        self.gyro = (self.read_gyro_axis(LSM6DSO.AXIS.X), self.read_gyro_axis(LSM6DSO.AXIS.Y), self.read_gyro_axis(LSM6DSO.AXIS.Z))
        return self.gyro
    
    def get_acc(self) -> tuple[float, float, float]:
        return self.acc
    
    def get_gyro(self) -> tuple[float, float, float]:
        return self.gyro
    
    def read_status(self) -> None:
        status_reg = self.read(LSM6DSO.REGISTER.STATUS_REG)
        if len(status_reg) == 1:
            status = int.from_bytes(status_reg, 'little')
            if status & (1 << 0) > 0:
                self.acc_data_ready_flag = True
            if status & (1 << 1) > 0:
                self.gyro_data_ready_flag = True
    
    def acc_data_ready(self) -> bool:
        ret = self.acc_data_ready_flag
        self.acc_data_ready_flag = False
        return ret

    def gyro_data_ready(self) -> bool:
        ret = self.gyro_data_ready_flag
        self.gyro_data_ready_flag = False
        return ret
