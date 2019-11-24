import time
import board
import digitalio
import busio
import audiocore
import adafruit_lis3dh
import adafruit_thermistor
from adafruit_ble.uart_server import UARTServer
#from adafruit_ble.uart_client import UARTClient
from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket
from adafruit_bluefruit_connect.accelerometer_packet import AccelerometerPacket
from adafruit_bluefruit_connect.location_packet import LocationPacket
import neopixel

## Hardware Configuration
# Hardware Global Variables
brightnessLevel = 0.25
noPixel = 10

# Initiate BLE Server
uart_server = UARTServer()
#uart_client = UARTClient()

if hasattr(board, 'ACCELEROMETER_SCL'):
    i2c = busio.I2C(board.ACCELEROMETER_SCL, board.ACCELEROMETER_SDA)
    int1 = digitalio.DigitalInOut(board.ACCELEROMETER_INTERRUPT)
    lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c, address=0x19, int1=int1)
else:
    i2c = busio.I2C(board.SCL, board.SDA)
    int1 = digitalio.DigitalInOut(board.D6)  # Set to correct pin for interrupt!
    lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c, int1=int1)
    
# Set range of accelerometer (can be RANGE_2_G, RANGE_4_G, RANGE_8_G or RANGE_16_G).
lis3dh.range = adafruit_lis3dh.RANGE_2_G
pixels = neopixel.NeoPixel(board.NEOPIXEL, noPixel, brightness = brightnessLevel, auto_write = False)


## Software Configuration
# Global Variables
threshold_low = -0.3
threshold_high = 0.3
getLocation = 1
flatGreen = (0, 255, 0)
edgeGreen = (10, 100, 10)
uphillRed = (255, 0, 0)
edgeRed = (100, 10, 10)
downhillBlue = (0, 0 ,255)
edgeBlue = (10, 10, 100)
white = (255, 255, 255)
p1 = (255, 0, 0)
p2 = (0, 255, 0)
p3 = (0, 0, 255)
p4 = (255, 255, 0)
p5 = (0, 255, 255)
p6 = (255, 255, 255)
p7 = (255, 100, 200)
p8 = (100, 255, 200)
p9 = (100, 200, 255)
p10 = (50, 100, 200)
blue = (0, 0, 255)
black = (0, 0, 0)


# Function Definitions
# Calculate Threshold - Up/Down/Flat Threshold Value, 2 = downhill; 1 = flat; 0 = uphill
def threshold(input, init):
    if input < threshold_low + init:
        threshold_value = 2
    elif input > threshold_high + init:
        threshold_value = 0
    else:
        threshold_value = 1
    return threshold_value
    
def initAcc(x,y,z):
    xInit = x
    yInit = y
    zInit = z
    return xInit, yInit, zInit

def pixelsClear():
    pixels.fill(black)
    pixels.show()

    
def tiltIndicator(threshold_value):
    if threshold_value == 1:
        # Middle LEDS - 2, 1, 3 & 7, 8, 6
        pixelsClear()
        pixels[2] = flatGreen
        pixels[7] = flatGreen
        pixels[1] = edgeGreen
        pixels[3] = edgeGreen
        pixels[8] = edgeGreen
        pixels[6] = edgeGreen
        pixels.show()
        print("Middle LEDs")
    elif threshold_value == 0:
        # Back LEDs 4, 3 & 5, 6
        pixelsClear()
        pixels[4] = uphillRed
        pixels[5] = uphillRed
        pixels[3] = edgeRed
        pixels[6] = edgeRed
        pixels.show()
        print("Back LEDs")
    elif threshold_value == 2:
        # Front LEDs 0,1 & 9,8
        pixelsClear()
        pixels[0] = downhillBlue
        pixels[9] = downhillBlue
        pixels[1] = edgeBlue
        pixels[8] = edgeBlue
        pixels.show()
        print("Front LEDs")


# Main Loop
while True:
    # Read accelerometer values (in m / s ^ 2).  Returns a 3-tuple of x, y,
    # z axis values.  Divide them by 9.806 to convert to Gs.
    x, y, z = [value / adafruit_lis3dh.STANDARD_GRAVITY for value in lis3dh.acceleration]
    print("x = %0.3f G, y = %0.3f G, z = %0.3f G" % (x, y, z))
    accValues = (x, y, z)
    print("accValues = ", accValues)
    xInit, yInit, zInit = initAcc(x, y, z)
    threshold_value = threshold(y, yInit)
    print("Threshold = ", threshold_value)
    print("Acc Inits: x = %0.3f, y = %0.3f, z = %0.3f" % (xInit, yInit, zInit))
#    accPacket = AccelerometerPacket(accValues)
    # Small delay to keep things responsive but give time for interrupt processing.
    time.sleep(0.3)
    print("pixel")
    pixels[1]=p1
    pixels[2]=p2
    pixels[3]=p3
    pixels[4]=p4
    pixels[5]=p5
    pixels[6]=p6
    pixels[7]=p7
    pixels[8]=p8
    pixels[9]=p9
    pixels[0]=p10
    pixels.show()
    
    uart_server.start_advertising()
#    uart_addresses = []
#    while not uart_addresses:
#        uart_addresses = uart_client.scan(scanner)
    while not uart_server.connected:
        pass
    while uart_server.connected:
#        one_byte = uart_server.read(threshold_value)
#        uart_server.write(one_byte)
        x, y, z = [value / adafruit_lis3dh.STANDARD_GRAVITY for value in lis3dh.acceleration]
        print("x = %0.3f G, y = %0.3f G, z = %0.3f G" % (x, y, z))
        x_str = str(x)
        y_str = str(y)
#        uart_server.write(x_str + ',' + y_str + '\n')
        time.sleep(0.1)
        getLocation = threshold(y, yInit)
        print("getLocation = ", getLocation)
        tiltIndicator(getLocation)
        packet = Packet.from_stream(uart_server)
#        if isinstance(packet, AccelerometerPacket):
#            print(packet.x, packet.y, packet.z)
#            time.sleep(0.1)
        if (isinstance(packet, LocationPacket) & getLocation == 1):
            print(packet.latitude, packet.longitude)
            time.sleep(2)
            lat=str(packet.latitude)
            longt=str(packet.longitude)
            alti=str(packet.altitude)
#            uart_server.write(lat + ',' + longt + alti + '\n')
            getLocation = 0
            
