import time

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


class I2CLCD:
    LCD_CHR = 1
    LCD_CMD = 0

    LCD_LINE_1 = 0x80
    LCD_LINE_2 = 0xC0
    LCD_LINE_3 = 0x94
    LCD_LINE_4 = 0xD4

    LCD_BACKLIGHT = 0x08
    ENABLE = 0b00000100

    def __init__(self, address=0x27, bus_num=1, cols=16, rows=2):
        self.address = address
        self.bus = SMBus(bus_num)
        self.cols = cols
        self.rows = rows
        self.backlight = self.LCD_BACKLIGHT

        self._init_lcd()

    def _write_byte(self, bits):
        self.bus.write_byte(self.address, bits | self.backlight)
        time.sleep(0.0005)

    def _toggle_enable(self, bits):
        self._write_byte(bits | self.ENABLE)
        time.sleep(0.0005)
        self._write_byte(bits & ~self.ENABLE)
        time.sleep(0.0005)

    def _send(self, value, mode):
        high = mode | (value & 0xF0)
        low = mode | ((value << 4) & 0xF0)

        self._write_byte(high)
        self._toggle_enable(high)

        self._write_byte(low)
        self._toggle_enable(low)

    def command(self, cmd):
        self._send(cmd, self.LCD_CMD)

    def write_char(self, char):
        self._send(ord(char), self.LCD_CHR)

    def _init_lcd(self):
        time.sleep(0.05)

        self._write_byte(0x30)
        self._toggle_enable(0x30)
        self._write_byte(0x30)
        self._toggle_enable(0x30)
        self._write_byte(0x30)
        self._toggle_enable(0x30)

        self._write_byte(0x20)
        self._toggle_enable(0x20)

        if self.rows > 1:
            self.command(0x28)
        else:
            self.command(0x20)

        self.command(0x0C)
        self.command(0x01)
        self.command(0x06)
        time.sleep(0.01)

    def clear(self):
        self.command(0x01)
        time.sleep(0.01)

    def home(self):
        self.command(0x02)
        time.sleep(0.01)

    def backlight_on(self):
        self.backlight = self.LCD_BACKLIGHT
        self._write_byte(0)

    def backlight_off(self):
        self.backlight = 0x00
        self._write_byte(0)

    def set_cursor(self, col, row):
        row_addresses = [
            self.LCD_LINE_1,
            self.LCD_LINE_2,
            self.LCD_LINE_3,
            self.LCD_LINE_4
        ]

        if row < 0 or row >= len(row_addresses):
            return

        self.command(row_addresses[row] + col)

    def write(self, text, row=0, col=0):
        self.set_cursor(col, row)
        text = str(text)

        for ch in text[: self.cols - col]:
            self.write_char(ch)

    def write_lines(self, lines):
        self.clear()
        for row, text in enumerate(lines[:self.rows]):
            self.write(str(text), row=row, col=0)
    def write_byte_value(self, value):
        self._send(value, self.LCD_CHR)

    def create_char(self, location, pattern):
        location &= 0x7
        self.command(0x40 | (location << 3))
        for row in pattern:
            self.write_byte_value(row)
    def close(self):
        self.clear()
        self.backlight_off()
        self.bus.close()