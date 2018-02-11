from smbus2 import SMBusWrapper

with SMBusWrapper(1) as bus:
    b=bus.read_byte_data(0x40, 000000)
    print (b)
