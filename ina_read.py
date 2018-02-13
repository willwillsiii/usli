from ina219 import INA219
from ina219 import DeviceRangeError
import time

shunt_ohm=0.1

def read():

    ina=INA219(shunt_ohm)
    ina.configureina.RANGE_16V)

    try:
        while True:
            print("Bus Voltage: ",ina.voltage()," V ")
            print("Bus Current: ",ina.current(), " mA ")
            print("Power: ",ina.power()," mW ")
            print("Shunt Voltage: ",ina.shunt_Voltage()," mV \n")
            time.sleep(0.1)
    except DeviceRangeError as e:
        print ("\n,"e)
        
if __name__ == "__main__":
    read()
