import time
import os
import math
try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic
    from robot_hat.utils import reset_mcu, run_command
except ImportError:
    print("This is not a PiCar-X System, Shadowing Hardware")
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic
    from sim_robot_hat import reset_mcu, run_command

import atexit
from readerwriterlock import rwlock
import concurrent.futures
import rossros as rr

reset_mcu()
time.sleep(0.2)


import logging #Sets Up Logging with: logging.debug("message")
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO,
datefmt="%H:%M:%S")


def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))

class Picarx(object):
    CONFIG = '/opt/picar-x/picar-x.conf'

    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    DIR_MIN = -30
    DIR_MAX = 30
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -35
    CAM_TILT_MAX = 65

    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    # servo_pins: camera_pan_servo, camera_tilt_servo, direction_servo
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # ultrasonic_pins: tring, echo2
    # config: path of config file
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P12', 'P13'],
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3'],
                config:str=CONFIG,
                ):
        atexit.register(self.stop)

        # reset robot_hat
        reset_mcu()
        time.sleep(0.2)

        # --------- config_flie ---------
        self.config_flie = fileDB(config, 777, os.getlogin())

        # --------- servos init ---------
        self.cam_pan = Servo(servo_pins[0])
        self.cam_tilt = Servo(servo_pins[1])   
        self.dir_servo_pin = Servo(servo_pins[2])
        # get calibration values
        self.dir_cali_val = float(self.config_flie.get("picarx_dir_servo", default_value=0))
        self.cam_pan_cali_val = float(self.config_flie.get("picarx_cam_pan_servo", default_value=0))
        self.cam_tilt_cali_val = float(self.config_flie.get("picarx_cam_tilt_servo", default_value=0))
        # set servos to init angle
        self.dir_servo_pin.angle(self.dir_cali_val)
        self.cam_pan.angle(self.cam_pan_cali_val)
        self.cam_tilt.angle(self.cam_tilt_cali_val)

        # --------- motors init ---------
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        # get calibration values
        self.cali_dir_value = self.config_flie.get("picarx_dir_motor", default_value="[1, 1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip().strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        # init pwm
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.line_reference = self.config_flie.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        self.cliff_reference = self.config_flie.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]
        # transfer reference
       # self.grayscale.reference(self.line_reference)

        # --------- ultrasonic init ---------
        tring, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(tring), Pin(echo))

        # --------- stop when canceled ---------
        atexit.register(self.stop)
    
    #Set Distance
    try:
        objectdisthesh = int(input("Enter Object Distance Detection: "))
    except:
        objectdisthesh = 5


    #Ultrasonic Sensor
    def get_distance(self):
        return self.ultrasonic.read()

    def obstacle_avoidance(self):
    
        distance = self.get_distance()
        print("Sensor Distance: " ,distance)
        if distance < self.objectdisthesh:
            Obstacle = True
        else:
            Obstacle = False
        return Obstacle


    def set_motor_speed(self, motor, speed):
        ''' set motor speed
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param speed: speed
        type speed: int      
        '''
        speed = constrain(speed, -100, 100)
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        # if speed != 0:
        #     speed = int(speed /2 ) + 50
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibrate(self, motor, value):
        ''' set motor direction calibration value
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param value: speed
        type value: int
        '''      
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)

    def dir_servo_calibrate(self, value):
        self.dir_cali_val = value
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    def set_dir_servo_angle(self, value):
        self.dir_current_angle = constrain(value, self.DIR_MIN, self.DIR_MAX)
        angle_value  = self.dir_current_angle + self.dir_cali_val
        self.dir_servo_pin.angle(angle_value)

    def cam_pan_servo_calibrate(self, value):
        self.cam_pan_cali_val = value
        self.config_flie.set("picarx_cam_pan_servo", "%s"%value)
        self.cam_pan.angle(value)

    def cam_tilt_servo_calibrate(self, value):
        self.cam_tilt_cali_val = value
        self.config_flie.set("picarx_cam_tilt_servo", "%s"%value)
        self.cam_tilt.angle(value)

    def set_cam_pan_angle(self, value):
        value = constrain(value, self.CAM_PAN_MIN, self.CAM_PAN_MAX)
        self.cam_pan.angle(-1*(value + -1*self.cam_pan_cali_val))

    def set_cam_tilt_angle(self,value):
        value = constrain(value, self.CAM_TILT_MIN, self.CAM_TILT_MAX)
        self.cam_tilt.angle(-1*(value + -1*self.cam_tilt_cali_val))

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def backward(self, speed):
        #logging.debug("Mater Style Activated")
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            power_scale = math.cos(math.radians(abs_current_angle)) #ackerman steering approximations
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, -1*speed)
                self.set_motor_speed(2, speed * power_scale)
            else:
                self.set_motor_speed(1, -1*speed * power_scale)
                self.set_motor_speed(2, speed )
        else:
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)  

    def forward(self, speed):
        #logging.debug("Lightning Style Activated")
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            power_scale = math.cos(math.radians(abs_current_angle)) #ackerman steering approximations
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, 1*speed * power_scale)
                self.set_motor_speed(2, -speed) 
            else:
                self.set_motor_speed(1, speed)
                self.set_motor_speed(2, -1*speed * power_scale)
        else:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)                  

    def stop(self):
        '''
        Execute twice to make sure it stops
        '''
        for _ in range(2):
            self.motor_speed_pins[0].pulse_width_percent(0)
            self.motor_speed_pins[1].pulse_width_percent(0)
            time.sleep(0.002)

    def get_distance(self):
        return self.ultrasonic.read()

    def set_grayscale_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.line_reference = value
            self.grayscale.reference(self.line_reference)
            self.config_flie.set("line_reference", self.line_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())

    def get_line_status(self,gm_val_list):
        return self.grayscale.read_status(gm_val_list)

    def set_line_reference(self, value):
        self.set_grayscale_reference(value)

    def get_cliff_status(self,gm_val_list):
        for i in range(0,3):
            if gm_val_list[i]<=self.cliff_reference[i]:
                return True
        return False

    def set_cliff_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = value
            self.config_flie.set("cliff_reference", self.cliff_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")





class Sensor: #Set up sensors and read the vaule
    def __init__(self):
        self.adc_1 = ADC('A0') #Right
        self.adc_2 = ADC('A1') #Middle
        self.adc_3 = ADC('A2') #Left
        self.adc = Grayscale_Module(self.adc_1,self.adc_1,self.adc_3,reference=None) #Currently Doesn't do anything
        self.Calibrator = self.Intial_calibrate()
    def read_sensor(self):
        self.adc = px.get_grayscale_data() #Already gives it in a list... ex: [600,600,1000]
        #Testing
        #self.adc= [600,600,1000]

        self.calibrated_adc = sensor.calibrate(self.adc,self.Calibrator)
        #print("Raw Sensor Data: ",self.calibrated_adc)
        return self.calibrated_adc
    
    def producer(self,bus,delay):

        while True:
            #Constant Updated Sensor Vaules
            sensor_data = self.read_sensor() 
            print("Raw Sensor Data: ",sensor_data)
            bus.write(sensor_data)
            time.sleep(delay)



    def Intial_calibrate(self):
        Intial_Vaules = px.get_grayscale_data()
        #Testing
        #Intial_Vaules = [600,600,1000]

        Avg_Intial_Vaule = sum(Intial_Vaules)/len(Intial_Vaules)
        Calibrator = [None]*3
        for i in range(3):
            Calibrator[i] = Intial_Vaules[i]-Avg_Intial_Vaule
        return Calibrator
    
    def calibrate(self,base_sensor_vaules,Calibrator):
        Calibaratred_Vaules = [x - y for x, y in zip(base_sensor_vaules , Calibrator)]
        return Calibaratred_Vaules


class Interpreter():
    def __init__(self, sensitivity_input, polarity_input):
        try:
            self.sensitivity = float(input("Input given Sensitivitiy: "))
        except:
            self.sensitivity= sensitivity_input

        self.polarity= polarity_input
    def proccessing(self,sensor_vaules):

        normlized_list = interpret.normilize(sensor_vaules)
        significant_list = interpret.significance(normlized_list)
        print("Proccessed Data: ", significant_list)

        return significant_list

    def normilize(self,sensor_vaules):

        self.avg = sum(sensor_vaules)/len(sensor_vaules)
        #print(self.avg)

        self.norm = [0] * 3
        try:

            for i in range(3):
                self.norm[i] = (sensor_vaules[i]-self.avg)/self.avg
            #print("Normilized Vaules: {self.norm}")
            return self.norm
            
        except:
            self.norm [0,0,0]
            print("Failed Norm")
        return self.norm


    def significance(self,norm_list):
        self.significant = [0] * 3
        #print(self.sensitivity)
        for i in range(3):
            Vaule = self.polarity * norm_list[i]
            if Vaule > self.sensitivity:
                self.significant[i] = 1
        
        return self.significant
    
    def consumer_producer(self,input_bus,output_bus, delay):
        while True:
                data = input_bus.read()
                if data != None:
                    proccessed_data = self.proccessing(data)
                    print("Proccessed Data: ", proccessed_data)
                    output_bus.write(proccessed_data)
                time.sleep(delay)


class Controller():
    def __init__(self):
        self.px = Picarx()
        self.Steer_angle = 0

        try:
            self.steering_factor = float(input("Input given Stearing Factor: "))

        except:
            self.steering_factor = 1.5
        try:
            self.power = float(input("Input given Power: "))
        except:
            self.power = 40

    def Control(self,Line_Direction,obstacle_avoidance):

        if obstacle_avoidance == False:

            if Line_Direction == [0,1,0]:
                print("Forward")
                self.Steer_angle = self.steering_factor * 0
            elif Line_Direction == [1,0,0]:
                print("Turn Left")
                Last_Direction = "Left"
                self.Steer_angle = self.steering_factor * -20

            elif Line_Direction == [1,1,0]:
                print("Turn Less Left")
                Last_Direction = "Left"
                self.Steer_angle = self.steering_factor * -10
            elif Line_Direction == [0,0,1]:
                print("Turn Right")
                Last_Direction = "Right"
                self.Steer_angle = self.steering_factor * 20
            elif Line_Direction == [0,1,1]:
                print("Turn Less Right")
                Last_Direction = "Right"
                self.Steer_angle = self.steering_factor * 10

            else:
                print("Lost")
            self.px.set_dir_servo_angle(self.Steer_angle)
            print("Steer Angle: ", self.Steer_angle)
            self.px.forward(controller.power)

        elif obstacle_avoidance == True:    
            px.stop()
            print("Object Found")
        else:
            print("No Object Data")


    def consumer(self,input_bus,delay):
        while True:
            data = input_bus.read()
            if data != None:
                self.Control(data,False)
            time.sleep(delay)




def LineFollowing(Sensor_Cycles):
    Calibrator = sensor.Intial_calibrate()
    #print(Calibrator)
    print("Line Following Start")
    while Sensor_Cycles != 0:
        Sensor_Cycles -=1
        Sensor_List = sensor.read_sensor()
        print(Sensor_List)

        Line_Direction = interpret.proccessing(Sensor_List)
        controller.Control(Line_Direction)
        #time.sleep(1) #Delay for testing
        
def User_Input():
    action=input("Start Line Following?")
    if action == "":
        action = -1 #Default Action makes infinate cycles
        return action
    elif not action.isalpha():
        try:
            User_Num = int(action)
            
            if User_Num > 0:
                print("Chosen Number of Cycles")
                return User_Num
        except ValueError:
            print("Not a Valid Input")
    else:
        print("Not a Valid Input")

#Docking Setup
class Broadcast:
    def __init__(self):
        self.message = None  # Initialize the message attribute to None
        self.lock = rwlock.RWLockWrite() # Initialize the read-write lock with writer priority

    def write(self,message):
        with self.lock.gen_wlock():
            self.message = message
    def read(self):
        with self.lock.gen_rlock():
            message = self.message
            return message

def Run_Bus():
    sensor_values_bus = Broadcast()
    interpreter_bus = Broadcast()
    #Input Delays
    sensor_delay = .01
    interpreter_delay = .02
    controller_delay = .03

    print("Start Bus")
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        eSensor = executor.submit(sensor.producer, sensor_values_bus,sensor_delay)
        eInterpreter = executor.submit(interpret.consumer_producer,sensor_values_bus, interpreter_bus,interpreter_delay)
        eController = executor.submit(controller.consumer, interpreter_bus, controller_delay)
    try:
        eSensor.result()
        eInterpreter.result()
        eController.result()
    except Exception as e:
        print("Bus Error: {e}")

if __name__ == "__main__":
    px = Picarx()
    sensor = Sensor()
    interpret = Interpreter(0.1,1) #Default vaules of 0.25 & 1
    controller = Controller()

    # Initiate data and termination busses
    try:
        RunTime = int(input("Enter Run Time: "))
    except:
        RunTime = 10

    BusSensor = rr.Bus(sensor.read_sensor(),"Grey Scale Bus") 
    BusObstacle = rr.Bus(px.obstacle_avoidance(),"Obsticle Bus")
    BusInterpret = rr.Bus(interpret.proccessing(sensor.read_sensor()),"Interpret Bus") 
    BusControl = rr.Bus(controller.Control(interpret.proccessing(sensor.read_sensor()),px.obstacle_avoidance()),"Control Bus")
    #Replace False with px.obstacle_avoidance()
    bTerminate = rr.Bus(0, "Termination Bus")

    # Read Sensor Data
    readSensor = rr.Producer(
        sensor.read_sensor,#Function for generate
        BusSensor,#Output Bus
        0.05,#Delay of Bus
        bTerminate,
        "Read Sensor Data"
        )

    readAvoidance = rr.Producer(
    px.obstacle_avoidance,  # function that will generate data
    BusObstacle,  # output data bus
    0.2,  # delay between data generation cycles
    bTerminate,  # bus to watch for termination signal
    "Read Avoidance signal")

    interpretData = rr.ConsumerProducer(
            interpret.proccessing,  # function that will process data
            BusSensor,  # input data buses
            BusInterpret,  # output data bus
            0.3,  # delay between data control cycles
            bTerminate,  # bus to watch for termination signal
            "Interpret Grey Scale Data")

    controlPiCar = rr.Consumer(
            controller.Control,  # function that will process data
            (BusInterpret, BusObstacle),  # input data buses
            0.3,  # delay between data control cycles
            bTerminate,  # bus to watch for termination signal
            "Control PiCar")

    printBuses = rr.Printer(
    (BusSensor, BusObstacle, BusInterpret, BusControl, bTerminate),  # Currently turned off
    0.25,  # delay between printing cycles
    bTerminate,  # bus to watch for termination signal
    "Print raw and derived data",  # Name of printer
    "Data bus readings are: ")  # Prefix for output

    terminationTimer = rr.Timer(
            bTerminate,  # Output data bus
            RunTime,  # Duration
            0.1,  # Delay between checking for termination time
            bTerminate,  # Bus to check for termination signal
            "Termination timer")  # Name of this timer
    

    # Create a list of producer-consumers to execute concurrently
    producer_consumer_list = [readSensor,
                            readAvoidance,
                          interpretData,
                          controlPiCar,                      
                          terminationTimer]
    

    rr.runConcurrently(producer_consumer_list)

    

    
    



    #User_Cycles = User_Input()
    #print(User_Cycles)
    #LineFollowing(User_Cycles)
    #Testing


    #print("Yaya")
    #px.forward(50)
    #time.sleep(1)
    #px.stop()