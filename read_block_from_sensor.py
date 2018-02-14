from smbus2 import SMBusWrapper

with SMBusWrapper(1) as bus:

    block= bus.read_i2c_block_data(0x40, 00000, 16)
    print(block)
