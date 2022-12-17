#Crane stretch part
#Kiefer, Lillian, Allen
'''Notes:
   - Item detection throws unsupported types 'Nonetype' and 'int' errors at line 108
   - The ground reflective sensor does not like moving backwards. It will frequently
     overshoot the boundary line and get stuck outside of the boundary.'''
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
item_sensor = ColorSensor('A')
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

def getRandomSteering():
    return random.randint(-100, 100)

def getRandomDuration():
    return random.randint(2, 6)

def craneDown():
    crane_motor.run_to_degrees_counted(-50, 10)
    
def craneUp():
    crane_motor.run_to_degrees_counted(50, 10)

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

    light = dropoff_sensor.get_reflected_light()
    on_black = light < mid_light

    if on_black:
        motor_pair.stop()
        next_state = 'correcting_bounds'
    
    #Note: This part throws errors when an object gets in grabbing range and overshoots.
    #      Unsupported types 'Nonetype' and 'int'.
    #Item detected within range
    if (object_detector.get_distance_cm() != None and less_than(object_detector.get_distance_cm(), 50)): 
        motor_pair.start(steering=0, speed=-15)
    #Item detected within grabbing range
    elif (object_detector.get_distance_cm() != None and less_than(2, object_detector.get_distance_cm()) and less_than(object_detector.get_distance_cm(), 10)):
        motor_pair.stop()
        next_state = 'picking_up'
        hub.status_light.on('green')
    elif(object_detector.get_distance_cm() == None):
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

    light = dropoff_sensor.get_reflected_light()
    on_black = light < mid_light

    if on_black:
        motor_pair.stop()
        next_state = 'correcting_bounds'
        return 
    
    color = dropoff_sensor.get_color()
    
    #If a dropoff point is located, begin dropoff process
    if color == 'red' or color == 'blue' or color == 'yellow':
        motor_pair.stop()
        next_state = 'dropping_off'
        hub.status_light.on(color)


def setupCorrectBounds():
    global correct_bounds_begin_time
    hub.status_light.on('red')
    correct_bounds_begin_time = timer.now()
    motor_pair.start(steering=100, speed=30)

def correctBounds():
    global next_state

    if timer.now() - correct_bounds_begin_time >= 1.4:
        motor_pair.stop()
        if has_object:
            next_state = 'searching'
        else:
            next_state = 'wandering'

def dropOff():
    global next_state

    craneDown()
    wait_for_seconds(0.5)
    craneUp()
    wait_for_seconds(0.5)
    craneDown()
    wait_for_seconds(0.5)
    craneUp()

    motor_pair.move(3, 'in', steering=20, speed=-30)

    #next_state = "wandering"
    next_state = "done"
    hub.status_light.on('white')

def pickUp():
    hub.status_light.on('green')
    motor_pair.move(180, unit='degrees', steering=100, speed=15)
    motor_pair.move(2, 'in', steering=0, speed=15)
    craneDown()
    next_state = "searching"
    has_object = True
    hub.status_light.on('white')
        

def mainLoop(duration):
    global timer, last_state, state, next_state
    timer = Timer()
    if has_object:
        next_state = 'searching'
        craneUp()
    else:
        next_state = 'wandering'

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

        #State can be wandering(has no block), searching(for dropoff), dropping off, picking up, bounds correction

calibrateDropoffSensor()
liftCalibration()
mainLoop(200)