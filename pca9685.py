# pca9685.py
import ustruct
import time

class PCA9685:
    """
    PCA9685 16-channel PWM driver (e.g., for servos).
    Use with MicroPython on Raspberry Pi Pico.
    """

    _MODE1   = 0x00
    _PRESCALE= 0xFE
    _LED0_ON_L = 0x06

    def __init__(self, i2c, address=0x40):
        """
        Args:
            i2c: machine.I2C instance, e.g. I2C(0, sda=Pin(0), scl=Pin(1))
            address: I2C addr (default 0x40)
        """
        self.i2c = i2c
        self.address = address
        self.reset()

    def _write(self, reg, val):
        self.i2c.writeto_mem(self.address, reg, bytearray([val]))

    def _read(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def reset(self):
        # MODE1 reset to default
        self._write(self._MODE1, 0x00)

    def freq(self, freq=None):
        """
        Get or set the PWM frequency (Hz). For servos use 50 Hz.
        """
        if freq is None:
            # Return approximate current frequency from PRESCALE
            prescale = self._read(self._PRESCALE)
            return int(25000000.0 / 4096.0 / (prescale + 1))
        # Datasheet formula: prescale = round(25e6/(4096*freq)) - 1
        prescale = int(round(25000000.0 / (4096.0 * freq)) - 1)
        if prescale < 3:   # chip min guard
            prescale = 3
        old_mode = self._read(self._MODE1)
        # sleep
        self._write(self._MODE1, (old_mode & 0x7F) | 0x10)
        # set prescale
        self._write(self._PRESCALE, prescale)
        # wake & restart + auto-increment
        self._write(self._MODE1, old_mode)
        time.sleep_us(5000)
        self._write(self._MODE1, old_mode | 0xA1)  # RESTART + AI

    def pwm(self, index, on=None, off=None):
        """
        Get/set raw on/off counts for channel index (0..15).
        """
        base = self._LED0_ON_L + 4 * index
        if on is None or off is None:
            data = self.i2c.readfrom_mem(self.address, base, 4)
            return ustruct.unpack('<HH', data)
        data = ustruct.pack('<HH', on, off)
        self.i2c.writeto_mem(self.address, base, data)

    def duty(self, index, value=None, invert=False):
        """
        Get/set duty in 0..4095 (12-bit). If invert=True, logical invert.
        """
        if value is None:
            on, off = self.pwm(index)
            # Handle full-off / full-on shortcuts
            if on == 0 and off == 4096:
                raw = 0
            elif on == 4096 and off == 0:
                raw = 4095
            else:
                raw = off  # typical use is ON at 0, OFF at value
            return 4095 - raw if invert else raw

        if not 0 <= value <= 4095:
            raise ValueError("duty out of range 0..4095")
        if invert:
            value = 4095 - value
        if value == 0:
            # full off
            self.pwm(index, 0, 4096)
        elif value == 4095:
            # full on
            self.pwm(index, 4096, 0)
        else:
            self.pwm(index, 0, value)
