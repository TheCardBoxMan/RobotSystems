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
        self.adc_1 = ADC('A0') #Right
        self.adc_2 = ADC('A1') #Middle
        self.adc_3 = ADC('A2') #Left
        self.adc = Grayscale_Module(self.adc_1,self.adc_1,self.adc_3,reference=None)

    def read_sensor(self):
        print("read sensor")
        return (self.adc.read())
    
if __name__=='__main__':
    print("Line Following Start")
