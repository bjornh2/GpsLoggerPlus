from machine import Pin, UART, SPI, I2C
import os
import sdcard
import time
import struct

from micropyGPS import MicropyGPS

# Constants
ADXL345_ADDRESS = 0x53 # address for accelerometer 
ADXL345_POWER_CTL = 0x2D # address for power control
ADXL345_DATA_FORMAT = 0x31 # configure data format
ADXL345_DATAX0 = 0x32 # where the x-axis data starts
ADXL345_X_CAL = 13 # constand for normalize RAW value to 0-1
ADXL345_Y_CAL = 13 # constand for normalize RAW value to 0-1
ADXL345_Z_CAL = 13 # constand for normalize RAW value to 0-1
ADXL345_XYZ_TRANS = 0.0039 # constand for normalize RAW value to 0-0.999

# Initialize ADXL345
def init_adxl345():
    i2c.writeto_mem(ADXL345_ADDRESS, ADXL345_POWER_CTL, bytearray([0x08]))  # Set bit 3 to 1 to enable measurement mode
    i2c.writeto_mem(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, bytearray([0x0B]))  # Set data format to full resolution, +/- 16g

# Read acceleration data
def read_accel_data():
    data = i2c.readfrom_mem(ADXL345_ADDRESS, ADXL345_DATAX0, 6)
    x, y, z = struct.unpack('<3h', data)
    return x, y, z

# Initialize GNSS/GPS
my_gps = MicropyGPS()

# Assign SPI0 chip select (CS) pin (and start it high)
cs = machine.Pin(5, machine.Pin.OUT)

#spi = SPI(1, 10_000_000, sck=Pin(14), mosi=Pin(15), miso=Pin(12))
spi = SPI(0, 10_000_000, sck=Pin(2), mosi=Pin(3), miso=Pin(4))

# Initialize I2C
i2c = I2C(1, sda=Pin(6), scl=Pin(7), freq=400000)

# Setup pins for RGB-LED
green_lu = Pin(10, Pin.OUT, value=0)
yellow_lu = Pin(11, Pin.OUT, value=0)
red_lu = Pin(12, Pin.OUT, value=0)
green_ld = Pin(13, Pin.OUT, value=0)
yellow_ld = Pin(14, Pin.OUT, value=0)
red_ld = Pin(15, Pin.OUT, value=0)
green_ru = Pin(16, Pin.OUT, value=0)
yellow_ru = Pin(17, Pin.OUT, value=0)
red_ru = Pin(18, Pin.OUT, value=0)
green_rd = Pin(19, Pin.OUT, value=0)
yellow_rd = Pin(20, Pin.OUT, value=0)
red_rd = Pin(21, Pin.OUT, value=0)

'''
Reserved C++ UART0 TX 0, RX 1
SPI0 SCK 2, TX 3, RX 4, CS 5
I2C1 SDA 6, SCL 7
UART1 TX 8, RX 9
'''

'''
b'$GNGGA,111719.000,5738.34790,N,01819.11322,E,1,08,1.7,-14.2,M,24.9,M,,*55\r\n'
b'$GNGLL,5738.34790,N,01819.11322,E,111719.000,A,A*4B\r\n'
b'$GNGSA,A,3,19,28,32,,,,,,,,,,2.8,1.7,2.2,1*3E\r\n'
b'$GNGSA,A,3,13,22,28,36,43,,,,,,,,2.8,1.7,2.2,4*32\r\n'
b'$GPGSV,3,1,10,02,65,168,,03,58,255,,04,15,199,,17,39,294,,0*66\r\n'
b'$GPGSV,3,2,10,19,23,319,26,21,49,152,,22,07,284,,28,26,089,15,0*69\r\n'
b'$GPGSV,3,3,10,31,13,122,,32,27,051,15,0*61\r\n'
b'$BDGSV,2,1,05,13,06,089,18,22,25,062,28,28,08,321,27,36,11,055,25,0*72\r\n'
b'$BDGSV,2,2,05,43,54,273,21,0*42\r\n'
b'$GNRMC,111719.000,A,5738.34790,N,01819.11322,E,0.00,13.08,280724,,,A,V*37\r\n'
b'$GNVTG,13.08,T,,M,0.00,N,0.00,K,A*19\r\n'
b'$GNZDA,111719.000,28,07,2024,00,00*4F\r\n'
b'$GPTXT,01,01,01,ANTENNA OK*35\r\n'
'''
uart= UART(1, baudrate=9600, bits=8, parity=None, stop=1, tx=Pin(8), rx=Pin(9), timeout=300)

# Initialize SD card
#sd = sdcard.SDCard(spi, cs)

# Mount filesystem
# vfs = os.VfsFat(sd)
# os.mount(vfs, "/sd")

# Create a file and write something to it
# with open("/sd/test02.txt", "w") as file:
#     file.write("Hello, SD World!\r\n")
#     file.write("This is a test\r\n")
# 
# # Open the file we just created and read from it
# with open("/sd/test02.txt", "r") as file:
#     data = file.read()
#     print(data)
    
# Create a log file and write nmea data to it
# with open("/sd/nmea.txt", "a") as file:
#     
#     print ('Reading GPS data...')
#     while True:
#         if uart.any():
#             nmea_msg = uart.readline()
#             #print (nmea_msg)print (nmea_msg)
#             my_gps.update_nmea_sentence(nmea_msg)
#             
#             if my_gps.last_nmea_type == 'GNRMC':
#                 print (nmea_msg)
#                 file.write(nmea_msg)
#                 print('nmea_type          ', my_gps.last_nmea_type)
#                 print('latitude           ', my_gps.latitude)
#                 print('longitude          ', my_gps.longitude)
#                 print('geoid_height       ', my_gps.geoid_height)
#                 print('altitude           ', my_gps.altitude)
#                 print('speed              ', my_gps.speed)
#                 print('course             ', my_gps.course)
#                 print('satellites_in_view ', my_gps.satellites_in_view)
#                 print('satellites_in_use  ', my_gps.satellites_in_use)
#                 print('fix_type           ', my_gps.fix_type)
#                 print('timestamp          ', my_gps.timestamp)
#                 print('date               ', my_gps.date)
#                 print('local_offset       ', my_gps.local_offset)
#                 
#                 yellow_lu.value(not yellow_lu.value())

#     print('one more time')
#     time.sleep(1)

# Main loop
init_adxl345()
while True:
    x, y, z = read_accel_data()
    print('--------------------')
    print(x, y, z) # raw values from sensor
    x_cal = (x + ADXL345_X_CAL) * ADXL345_XYZ_TRANS
    y_cal = (y + ADXL345_Y_CAL) * ADXL345_XYZ_TRANS
    z_cal = (z + ADXL345_Z_CAL) * ADXL345_XYZ_TRANS
    print("X: {}, Y: {}, Z: {}".format(x*ADXL345_XYZ_TRANS, y*ADXL345_XYZ_TRANS, z*ADXL345_XYZ_TRANS))
    print("X: {}, Y: {}, Z: {}".format(x + ADXL345_X_CAL, y + ADXL345_Y_CAL, z + ADXL345_Z_CAL))
    print("X: {}, Y: {}, Z: {}".format(x_cal, y_cal, z_cal))
    time.sleep(0.5)
    
print('ready')
