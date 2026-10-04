import ustruct
import time


class PCA9685:
    """
    Driver for the PCA9685 16-channel PWM/servo controller (I2C).
    """

    def __init__(self, i2c, address=0x40):
        """
        Args:
            i2c: an initialised machine.I2C instance
                 e.g. i2c = I2C(0, sda=Pin(8), scl=Pin(9))
            address: I2C address of the board (default 0x40)
        """
        self.i2c = i2c
        self.address = address
        self.reset()

    # ---------- low level ----------

    def _write(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytearray([value]))

    def _read(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    @staticmethod
    def _check_index(index):
        if not 0 <= index <= 15:
            raise ValueError("channel index must be 0-15, got {}".format(index))

    # ---------- setup ----------

    def reset(self):
        self._write(0x00, 0x00)  # MODE1 = 0 -> normal mode, no sleep

    def freq(self, freq=None):
        """Get or set the PWM frequency (Hz). Typical servo freq = 50."""
        if freq is None:
            # NOTE: this only reads back the prescaler, it is an approximation
            return int(25000000.0 / 4096 / (self._read(0xFE) - 0.5))
        prescale = int(25000000.0 / 4096.0 / freq + 0.5)
        old_mode = self._read(0x00)  # MODE1
        self._write(0x00, (old_mode & 0x7F) | 0x10)  # go to sleep to change prescale
        self._write(0xFE, prescale)
        self._write(0x00, old_mode)
        time.sleep_us(5)
        self._write(0x00, old_mode | 0xA1)  # restart + auto-increment on

    # ---------- per-channel PWM ----------

    def pwm(self, index, on=None, off=None):
        self._check_index(index)
        if on is None or off is None:
            data = self.i2c.readfrom_mem(self.address, 0x06 + 4 * index, 4)
            return ustruct.unpack('<HH', data)
        data = ustruct.pack('<HH', on, off)
        self.i2c.writeto_mem(self.address, 0x06 + 4 * index, data)

    def duty(self, index, value=None, invert=False):
        """Get or set a 0-4095 duty value for a channel."""
        if value is None:
            pwm = self.pwm(index)
            if pwm == (0, 4096):
                value = 0
            elif pwm == (4096, 0):
                value = 4095
            else:
                # BUGFIX: this used to run unconditionally and overwrite the
                # 0 / 4095 special cases above. Now it only runs when neither
                # special case applies.
                value = pwm[1]
            if invert:
                value = 4095 - value
            return value

        if not 0 <= value <= 4095:
            raise ValueError("duty out of range 0-4095: {}".format(value))
        if invert:
            value = 4095 - value
        if value == 0:
            self.pwm(index, 0, 4096)
        elif value == 4095:
            self.pwm(index, 4096, 0)
        else:
            self.pwm(index, 0, value)

    def release_all(self):
        """Turn off every channel (servos go limp) - useful before power-down."""
        for i in range(16):
            self.pwm(i, 0, 4096)