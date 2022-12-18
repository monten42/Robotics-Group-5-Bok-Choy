#Crane stretch part
#Kiefer, Lillian, Allen
from spike import PrimeHub, Motor, MotorPair, ColorSensor, DistanceSensor, LightMatrix
from spike.control import Timer, wait_for_seconds
from spike.operator import greater_than, less_than
import random

hub = PrimeHub()
#set Crane motors   
crane_motor = Motor('E')
# set Drive motors    
motor_pair = MotorPair('C','D')
l_motor = Motor('C')
r_motor = Motor('D')
#color Sensors
dropoff_sensor = ColorSensor('F')
back_sensor = ColorSensor('A')
# Initialize the Distance Sensor
object_detector = DistanceSensor('B')

mid_light = 50 #default mid light value

last_state = 'calibrating'
state = 'calibrating'
next_state = None

timer = None

wander_begin_time = None
wander_duration = None

search_begin_time = None
search_duration = None

correct_bounds_begin_time = None

has_object = False
backing_up = False

def getRandomSteering(min=-100, max=100):
    return random.randint(min, max)

def getRandomDuration():
    return random.randint(2, 6)

def getRandomDegrees():
    return random.randint(10, 350)

def craneDown():
    crane_motor.run_to_degrees_counted(-100, 10)
    
def craneUp():
    crane_motor.run_to_degrees_counted(100, 10)

#Calibrate lift to the right height to pickup/dropoff blocks
def liftCalibration():
    hub.status_light.on('violet')
    hub.speaker.beep()
    while((hub.right_button.is_pressed() and hub.left_button.is_pressed()) == False):
        if (hub.left_button.is_pressed()):
            crane_motor.start(-10)
        elif (hub.right_button.is_pressed()):
            crane_motor.start(10)
        else:
            crane_motor.stop()         
    crane_motor.stop()
    hub.status_light.on('white')
    hub.speaker.beep()
    crane_motor.set_degrees_counted(0)


#Calibrate dropoff sensor to detect black and white accurately
def calibrateDropoffSensor():
    global mid_light
    hub.status_light.on('pink')
    hub.speaker.beep()
    hub.left_button.wait_until_pressed()
    wait_for_seconds(0.5)
    left = dropoff_sensor.get_reflected_light() #Take first reading
    hub.speaker.beep()
    hub.right_button.wait_until_pressed()
    wait_for_seconds(0.5)
    right = dropoff_sensor.get_reflected_light() #Take second reading
    hub.speaker.beep()
    #Set mid_light to the average of the two readings
    mid_light = int((left + right) / 2)
    hub.status_light.on('white')

def setupWander():
    global wander_duration, wander_begin_time
    hub.status_light.on('azure')
    hub.speaker.beep()
    motor_pair.start(steering=getRandomSteering(), speed=-30)
    wander_duration = getRandomDuration()
    wander_begin_time = timer.now()

def wander():
    global next_state
    if timer.now() - wander_begin_time >= wander_duration:
        setupWander()

    if hitBounds():
        motor_pair.stop()
        next_state = 'correcting_bounds'
        return
    
    #Note: This part throws errors when an object gets in grabbing range and overshoots.
    #      Unsupported types 'Nonetype' and 'int'.
    #Item detected within grabbing range
    distance = object_detector.get_distance_cm()
    if (distance != None and less_than(2, distance) and less_than(distance, 10)):
        motor_pair.stop()
        next_state = 'picking_up'
        hub.status_light.on('green')
    #Item detected within range
    elif (distance != None and less_than(distance, 30)):
        hub.status_light.on('pink') 
        motor_pair.start(steering=0, speed=-10)
    else:
        hub.status_light.on('azure')

def setupSearch():
    global search_duration, search_begin_time
    hub.status_light.on('orange')
    hub.speaker.beep()
    motor_pair.start(steering=getRandomSteering(), speed=30)
    search_duration = getRandomDuration()
    search_begin_time = timer.now()

def search():
    global next_state
    if timer.now() - search_begin_time >= search_duration:
        setupSearch()

    if hitBounds():
        motor_pair.stop()
        next_state = 'correcting_bounds'
        return 
    
    color = dropoff_sensor.get_color()
    
    #If a dropoff point is located, begin dropoff process
    if color == 'red' or color == 'blue' or color == 'yellow':
        motor_pair.stop()
        next_state = 'dropping_off'
        hub.status_light.on(color)

def hitBounds():
    sensor = dropoff_sensor
    if backing_up:
        sensor = back_sensor
    light = sensor.get_reflected_light()
    on_black = light < mid_light

    if on_black:
        return True
    return False 


def setupCorrectBounds():
    global correct_bounds_begin_time, backing_up
    hub.status_light.on('red')
    correct_bounds_begin_time = timer.now()
    if backing_up:
        motor_pair.start(steering=getRandomSteering(min=-20, max=20), speed=30)
    else:
        motor_pair.start(steering=getRandomSteering(min=-20, max=20), speed=-30)
    backing_up = not backing_up

def correctBounds():
    global next_state, backing_up

    sensor = dropoff_sensor
    if backing_up:
        sensor = back_sensor
    
    light = sensor.get_reflected_light()
    on_black = light < mid_light
    if on_black:
        motor_pair.stop()
        setupCorrectBounds()

    if timer.now() - correct_bounds_begin_time >= 1.4:
        motor_pair.stop()
        backing_up = not backing_up
        if has_object:
            next_state = 'searching'
        else:
            next_state = 'wandering'

def dropOff():
    global next_state

    craneUp()
    wait_for_seconds(0.5)
    craneDown()
    wait_for_seconds(0.5)
    craneUp()
    wait_for_seconds(0.5)
    craneDown()

    motor_pair.move(3, 'in', steering=20, speed=-30)

    #next_state = "wandering"
    next_state = "done"
    hub.status_light.on('white')

def pickUp():
    global next_state
    
    print("In pick-up")
    hub.status_light.on('green')
    motor_pair.move(360, unit='degrees', steering=100, speed=15)
    motor_pair.move(5, 'in', steering=0, speed=15)
    craneDown()
    next_state = 'searching'
    has_object = True
    hub.status_light.on('white')
        

def mainLoop(duration):
    global timer, last_state, state, next_state
    timer = Timer()

    while(timer.now() < duration):
        if state == 'wandering':
            if last_state != 'wandering':
                setupWander()
            wander()
        elif state == 'searching':
            if last_state != 'searching':
                setupSearch()
            search()
        elif state == 'dropping_off':
            dropOff()
        elif state == 'picking_up':
            pickUp()
        elif state == 'correcting_bounds':
            if last_state != 'correcting_bounds':
                setupCorrectBounds()
            correctBounds()

        last_state = state
        state = next_state

        print(last_state, ", ", state, ", ", next_state)

        #State can be wandering(has no block), searching(for dropoff), dropping off, picking up, bounds correction

calibrateDropoffSensor()
liftCalibration()
if has_object:
    next_state = 'searching'
    craneDown()
else:
    next_state = 'wandering'
    craneUp()

mainLoop(200)