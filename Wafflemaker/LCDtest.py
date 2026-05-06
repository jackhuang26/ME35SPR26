import time
from lcd_i2c import I2CLCD

lcd = I2CLCD(address=0x27, bus_num=1, cols=20, rows=4)

duck = [
    0b00000,
    0b00110,
    0b01111,
    0b01110,
    0b00111,
    0b01110,
    0b00000,
    0b00000
]

try:
    lcd.create_char(0, duck)
    row = 1
    lcd.clear()

    while True:
        for col in range(20):
            lcd.clear()
            lcd.write("duck run", row=0, col=0)
            lcd.set_cursor(col, row)
            lcd.write_byte_value(0)
            time.sleep(0.15)

except KeyboardInterrupt:
    lcd.clear()