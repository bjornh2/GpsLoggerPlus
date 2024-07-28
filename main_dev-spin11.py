from machine import Pin, UART, SPI, I2C
import os
import sdcard
import time

from micropyGPS import MicropyGPS
my_gps = MicropyGPS()

# Assign SPI0 chip select (CS) pin (and start it high)
cs = machine.Pin(5, machine.Pin.OUT)

#spi = SPI(1, 10_000_000, sck=Pin(14), mosi=Pin(15), miso=Pin(12))
spi = SPI(0, 10_000_000, sck=Pin(2), mosi=Pin(3), miso=Pin(4))

# Setup pins for LED
green_lu = Pin(10, Pin.OUT, 0)
yellow_lu = Pin(11, Pin.OUT, Pin.high())
red_lu = Pin(12, Pin.OUT, Pin.low())

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
sd = sdcard.SDCard(spi, cs)

# Mount filesystem
vfs = os.VfsFat(sd)
os.mount(vfs, "/sd")

# Create a file and write something to it
with open("/sd/test02.txt", "w") as file:
    file.write("Hello, SD World!\r\n")
    file.write("This is a test\r\n")

# Open the file we just created and read from it
with open("/sd/test02.txt", "r") as file:
    data = file.read()
    print(data)
    
# Create a log file and write nmea data to it
with open("/sd/nmea.txt", "w") as file:
    
    print ('Reading GPS data...')
    while True:
        if uart.any():
            nmea_msg = uart.readline()
            #print (nmea_msg)print (nmea_msg)
            my_gps.update_nmea_sentence(nmea_msg)
            
            if my_gps.last_nmea_type == 'GNRMC':
                print (nmea_msg)
                file.write(nmea_msg)
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

#     print('one more time')
#     time.sleep(1)
