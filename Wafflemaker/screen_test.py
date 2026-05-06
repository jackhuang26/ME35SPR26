from RPLCD.i2c import CharLCD
from time import sleep

# Try address 0x27 first (most common), then 0x3F if needed
lcd = CharLCD(
    i2c_expander='PCF8574',
    address=0x27,
    port=1,
    cols=20,
    rows=4,
    charmap='A00'
)

lcd.clear()

lcd.write_string("Hello Tyler!")
sleep(2)

lcd.cursor_pos = (1, 0)
lcd.write_string("Raspberry Pi")

lcd.cursor_pos = (2, 0)
lcd.write_string("20x4 Display")

lcd.cursor_pos = (3, 0)
lcd.write_string("Working??")