import time
import board
import digitalio
import busio
import adafruit_lis3dh
import adafruit_thermistor
from adafruit_ble.uart_server import UARTServer
from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket
from adafruit_bluefruit_connect.accelerometer_packet import AccelerometerPacket
from adafruit_bluefruit_connect.location_packet import LocationPacket


## Hardware Configuration
# Initiate BLE Server
uart_server = UARTServer()

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


## Software Configuration
# Global Variables



# Function Definitions


# Main Loop
while True:
    # Read accelerometer values (in m / s ^ 2).  Returns a 3-tuple of x, y,
    # z axis values.  Divide them by 9.806 to convert to Gs.
    x, y, z = [value / adafruit_lis3dh.STANDARD_GRAVITY for value in lis3dh.acceleration]
    print("x = %0.3f G, y = %0.3f G, z = %0.3f G" % (x, y, z))
    # Small delay to keep things responsive but give time for interrupt processing.
    time.sleep(0.1)
    
    uart_server.start_advertising()
    while not uart_server.connected:
        pass
    while uart_server.connected:
        time.sleep(10)
        packet = Packet.from_stream(uart_server)
        if isinstance(packet, AccelerometerPacket):
            print(packet.x, packet.y, packet.z)
            time.sleep(0.1)
        if isinstance(packet, LocationPacket):
            print(packet.latitude, packet.longitude)
            time.sleep(0.1)
