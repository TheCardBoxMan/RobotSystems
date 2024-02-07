'''
    Line Following program for Picar-X:

    Pay attention to modify the reference value of the grayscale module 
    according to the practical usage scenarios.
    Auto calibrate grayscale values:
        Please run ./calibration/grayscale_calibration.py
    Manual modification:
        Use the following: 
            px.set_line_reference([1400, 1400, 1400])
        The reference value be close to the middle of the line gray value
        and the background gray value.

'''
import picarx_improved as pc
from time import sleep
px = pc.Picarx()

class Sensor:
    def __init__(self):
        self.adc_1 = ADC(channel=1)
        self.adc_2 = ADC(channel=2)
        self.adc_3 = ADC(channel=3)

    def read_sensor(self):
        # Poll the three ADC structures and put their outputs into a list
        sensor_readings = px.get_grayscale_data(self)
        return sensor_readings
    
if __name__=='__main__':
    print("Line Following Start")
    my_sensor = Sensor()
    readings = my_sensor.read_sensor()
    print("Sensor readings:", readings)