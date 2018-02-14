from ina219 import INA219
from ina219 import DeviceRangeError

shunt_ohm=0.1

def read_once():

    ina=INA219(shunt_ohm)
    ina.configure(ina.RANGE_16V)

    try:
        print("Bus Voltage: ",ina.voltage()," V ")
        print("Bus Current: ",ina.current(), " mA ")
        print("Power: ",ina.power()," mW ")
        print("Shunt Voltage: ",ina.shunt_voltage()," mV ")
        print("Supply Voltage: ", ina.supply_voltage(), "V")
    except DeviceRangeError as e:
        print(e)
        
if __name__ == "__main__":
    read_once()
