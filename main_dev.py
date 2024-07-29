from machine import Pin, UART, SPI, I2C
#import os
import vfs
import sdcard
import time
import struct

from micropyGPS import MicropyGPS

# Constants
ADXL345_ADDRESS = 0x53 # address for accelerometer 
ADXL345_DEVICE_ID_REG = 0x00 # address for accelerometer 
ADXL345_DEVICE_ID = 0xE5 # Device ID of ADXL345
#ADXL345_RESET = 0x?? # address for power control

ADXL345_POWER_CTL = 0x2D # address for power control
ADXL345_DATA_FORMAT = 0x31 # configure data format
ADXL345_DATAX0 = 0x32 # where the x-axis data starts
# SENSITIVITY has not any inpact on tilt angle
ADXL345_SENSITIVITY_2G = 0x08 # full resolution, +/- 2g
ADXL345_SENSITIVITY_4G = 0x09 # full resolution, +/- 4g
ADXL345_SENSITIVITY_8G = 0x0A # full resolution, +/- 8g
ADXL345_SENSITIVITY_16G = 0x0B # full resolution, +/- 16g
ADXL345_X_CAL = -1 # constand for calibration of RWA value
ADXL345_Y_CAL = -3 # constand for calibration of RWA value
ADXL345_Z_CAL = -1 # constand for calibration of RWA value
ADXL345_TILT_RES = 4 # constand for resulution of tilt angle, RV only intrested of +-10 deg
# max out 255/90 deg -> 2.8/1 deg -> 10 deg = 28
# 3 LED = 0-7, 4/LED-step -> gitter resistant on +-2
ADXL345_XYZ_TRANS = 0.0039 # constand for normalize RAW value to 0-0.999

# Initialize ADXL345
def init_adxl345():
    # TODO: call reset! - ADXL345_RESET
    data = i2c.readfrom_mem(ADXL345_ADDRESS, ADXL345_DEVICE_ID_REG, 1) # get device ID
    print('ADXL345 Divece ID', data)
    if (data != bytearray((ADXL345_DEVICE_ID,))):
        print("ERROR: Could not communicate with ADXL345")
    i2c.writeto_mem(ADXL345_ADDRESS, ADXL345_POWER_CTL, bytearray([0x08]))  # Set bit 3 to 1 to enable measurement mode
    i2c.writeto_mem(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, bytearray([ADXL345_SENSITIVITY_16G]))  # Set data format resolution

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
i2c = I2C(1, sda=Pin(6), scl=Pin(7), freq=100000)
#i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)
print('')
print(i2c.scan())

# Setup pins for RGB-LED
green_x_plus = Pin(10, Pin.OUT, value=0)
yellow_x_plus = Pin(11, Pin.OUT, value=0)
red_x_plus = Pin(12, Pin.OUT, value=0)
green_x_minus = Pin(13, Pin.OUT, value=0)
yellow_x_minus = Pin(14, Pin.OUT, value=0)
red_x_minus = Pin(15, Pin.OUT, value=0)
green_y_plus = Pin(16, Pin.OUT, value=0)
yellow_y_plus = Pin(17, Pin.OUT, value=0)
red_y_plus = Pin(18, Pin.OUT, value=0)
green_y_minus = Pin(19, Pin.OUT, value=0)
yellow_y_minus = Pin(20, Pin.OUT, value=0)
red_y_minus = Pin(21, Pin.OUT, value=0)

# Function for turning off LED:s
def turn_off_leds(led1, led2, led3):
    led1.off()
    led2.off()
    led3.off()

# Function for translate tilt angle and activate corsponding LED:s
def control_led_values(led1, led2, led4, value):
    value_int = round(value)
    # bit 1
    if value_int in [1, 3, 5, 7] or value_int > 7:
        led1.on()
    else:
        led1.off()
    # bit 2
    if value_int in [2, 3, 6, 7] or value_int > 7:
        led2.on()
    else:
        led2.off()    
    # bit 3
    if value_int >= 4:
        led4.on()
    else:
        led4.off()    


# Function for calculate tilt and activate corsponding LED:s
def activate_tilt_leds(x, y, z):
    # check tilt on X axel
    tilt_x = abs(x / ADXL345_TILT_RES)
    if x > 0:
        control_led_values(green_x_plus, yellow_x_plus, red_x_plus, tilt_x)
        turn_off_leds(green_x_minus, yellow_x_minus, red_x_minus)
    else:
        control_led_values(green_x_minus, yellow_x_minus, red_x_minus, tilt_x)
        turn_off_leds(green_x_plus, yellow_x_plus, red_x_plus)
        
    # check tilt on Y axel
    tilt_y = abs(y / ADXL345_TILT_RES)
    if y > 0:
        control_led_values(green_y_plus, yellow_y_plus, red_y_plus, tilt_y)
        turn_off_leds(green_y_minus, yellow_y_minus, red_y_minus)
    else:
        control_led_values(green_y_minus, yellow_y_minus, red_y_minus, tilt_y)
        turn_off_leds(green_y_plus, yellow_y_plus, red_y_plus)

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

# Inti tilt sensor ADXL345
init_adxl345()

# Initialize SD card
sd = sdcard.SDCard(spi, cs)

# Mount sdcard filesystem
# vfs = os.VfsFat(sd)
# os.mount(vfs, "/sd")
vfs_sd = vfs.VfsFat(sd)
vfs.mount(vfs_sd, "/sd")

# Create a file and write something to it
# with open("/sd/test02.txt", "w") as file:
#     file.write("Hello, SD World!\r\n")
#     file.write("This is a test\r\n")
# 
# # Open the file we just created and read from it
# with open("/sd/test02.txt", "r") as file:
#     data = file.read()
#     print(data)
    
    
print ('Reading GPS data...')
while True:
    if uart.any():
        nmea_msg = uart.readline()
        #print(nmea_msg)
        my_gps.update_nmea_sentence(nmea_msg)
        
        if my_gps.last_nmea_type == 'GNRMC':
            print (nmea_msg)
            # Append nmea data to log file
            file_name = '/sd/nmea_20' + str(my_gps.date[2]) + '_' + str(my_gps.date[1]) + '_' + str(my_gps.date[0]) + '.txt'  
            print(file_name)
            #with open("/sd/nmea.txt", "a") as file:
            with open(file_name, "a") as file:
                file.write(nmea_msg)
            # print out the result of my_gps
            print('nmea_type          ', my_gps.last_nmea_type)
            print('latitude           ', my_gps.latitude)
            print('longitude          ', my_gps.longitude)
            print('geoid_height       ', my_gps.geoid_height)
            print('altitude           ', my_gps.altitude)
            print('speed              ', my_gps.speed)
            print('course             ', my_gps.course)
            print('satellites_in_view ', my_gps.satellites_in_view)
            print('satellites_in_use  ', my_gps.satellites_in_use)
            print('fix_type           ', my_gps.fix_type)
            print('timestamp          ', my_gps.timestamp)
            print('date               ', my_gps.date)
            print('local_offset       ', my_gps.local_offset)
            time.sleep(0.2)

    else:
        time.sleep(0.5)
        
    # read tilt angle
    x, y, z = read_accel_data()
    print('--------------------')
    print(x, y, z) # raw values from sensor
    x_cal = (x + ADXL345_X_CAL) * ADXL345_XYZ_TRANS
    y_cal = (y + ADXL345_Y_CAL) * ADXL345_XYZ_TRANS
    z_cal = (z + ADXL345_Z_CAL) * ADXL345_XYZ_TRANS
    print("X: {}, Y: {}, Z: {}".format(x*ADXL345_XYZ_TRANS, y*ADXL345_XYZ_TRANS, z*ADXL345_XYZ_TRANS))
    print("X: {}, Y: {}, Z: {}".format(x + ADXL345_X_CAL, y + ADXL345_Y_CAL, z + ADXL345_Z_CAL))
    print("X: {}, Y: {}, Z: {}".format(x_cal, y_cal, z_cal))
    activate_tilt_leds(x + ADXL345_X_CAL, y + ADXL345_Y_CAL, z + ADXL345_Z_CAL)
    
print('ready')
