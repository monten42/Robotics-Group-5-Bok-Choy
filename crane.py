#Crane stretch part
#Kiefer, Lillian, Allen
from spike import PrimeHub, Motor, MotorPair, ColorSensor, DistanceSensor, LightMatrix
from spike.control import Timer, wait_for_seconds
from spike.operator import greater_than, less_than
import random

#Get hub
hub = PrimeHub()

#set Crane motors   
crane_motor = Motor('E')

#set Drive motors    
motor_pair = MotorPair('C','D')
l_motor = Motor('C')
r_motor = Motor('D')

#Dropoff (front) sensor
dropoff_sensor = ColorSensor('F')
#Back color sensor
back_sensor = ColorSensor('A')

#Initialize the Distance Sensor
object_detector = DistanceSensor('B')

#default mid light value
mid_light = 50 

#Setup state variables.  Initial state is 'calibrating'.
last_state = 'calibrating'
state = 'calibrating'
next_state = None

#Timer used for various tasks
timer = None

#Global variables to be used by wander functions
wander_begin_time = None
wander_duration = None

#Global variables to be used by search functions
search_begin_time = None
search_duration = None

#Global variable to be used by correct bounds functions
correct_bounds_begin_time = None

#Does the robot start with an object? 
#If so the program will skip right to the search state, where it tries to find a dropoff point.
#If not, it will go into the wander state, and look for a block to pickup.
has_object = False

#Is the robot currently backing up? This is useful for bounds checking and for determining 
#which light sensor will be at the 'front' of the robot at any given time.
backing_up = True

#Returns a random value that steering can be set to
def getRandomSteering(min=-100, max=100):
    return random.randint(min, max)

#Returns a random duration between 2 and 6
def getRandomDuration():
    return random.randint(2, 6)

#Returns a random amount of degrees to turn
def getRandomDegrees():
    return random.randint(10, 700)

#Move crane to down (floor) position
def craneDown():
    crane_motor.run_to_degrees_counted(0, 10)

#Move crane to up position  
def craneUp():
    crane_motor.run_to_degrees_counted(80, 10)

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

#Run once each time wander state is entered. Sets up some variables.  
def setupWander():
    global wander_duration, wander_begin_time
    hub.status_light.on('azure')
    hub.speaker.beep()
    motor_pair.start(steering=getRandomSteering(), speed=-30)
    wander_duration = getRandomDuration()
    wander_begin_time = timer.now()

#Wander state. Runs every loop as long as state == 'wandering'
def wander():
    global next_state, backing_up
    if timer.now() - wander_begin_time >= wander_duration:
        setupWander()

    #Note: This part throws errors when an object gets in grabbing range and overshoots.
    #      Unsupported types 'Nonetype' and 'int'.
    #Item detected within grabbing range
    distance = object_detector.get_distance_cm()
    if (distance != None and less_than(2, distance) and less_than(distance, 10)):
        motor_pair.stop()
        next_state = 'picking_up'
        hub.status_light.on('green')
        backing_up = False
    #Item detected within range
    elif (distance != None and less_than(distance, 30)):
        hub.status_light.on('pink') 
        motor_pair.start(steering=0, speed=-10)
    else:
        hub.status_light.on('azure')
    
    if hitBounds():
        motor_pair.stop()
        next_state = 'correcting_bounds'
        return

#Run once each time search state is entered. Sets up some variables.  
def setupSearch():
    global search_duration, search_begin_time
    hub.status_light.on('orange')
    hub.speaker.beep()
    motor_pair.start(steering=getRandomSteering(), speed=30)
    search_duration = getRandomDuration()
    search_begin_time = timer.now()

#Search state. Runs every loop as long as state == 'searching'
def search():
    global next_state
    if timer.now() - search_begin_time >= search_duration:
        setupSearch()
    
    color = dropoff_sensor.get_color()
    
    #If a dropoff point is located, begin dropoff process
    if color == 'red' or color == 'blue' or color == 'yellow':
        motor_pair.stop()
        next_state = 'dropping_off'
        hub.status_light.on(color)
    
    if hitBounds():
        motor_pair.stop()
        next_state = 'correcting_bounds'
        return 

#Returns True if robot's current forward-facing sensor sees black
def hitBounds():
    sensor = dropoff_sensor
    if backing_up:
        sensor = back_sensor

    color = sensor.get_color()
    if color == 'red' or color == 'blue' or color == 'yellow':  #If it's just a dropoff point that the sensor sees, ignore it
        return False
        
    light = sensor.get_reflected_light()
    on_black = light < mid_light

    if on_black:
        return True
    return False 

#Run once each time correcting_bounds state is entered. Sets up some variables.  
def setupCorrectBounds():
    global correct_bounds_begin_time, backing_up
    hub.status_light.on('red')
    correct_bounds_begin_time = timer.now()
    if backing_up:
        motor_pair.start(steering=getRandomSteering(min=-20, max=20), speed=30)
    else:
        motor_pair.start(steering=getRandomSteering(min=-20, max=20), speed=-30)
    backing_up = not backing_up

#Correct bounds state. Runs every loop as long as state == 'correcting_bounds'
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

        sign = random.randint(0, 1)
        if sign == 0:
            motor_pair.move(getRandomDegrees(), unit='degrees', steering=-100, speed=15)
        else:
            motor_pair.move(getRandomDegrees(), unit='degrees', steering=100, speed=15)

#Dropoff state. Runs every loop as long as state == 'dropping_off'
def dropOff():
    global next_state

    craneUp()
    #wait_for_seconds(0.5)
    #craneDown()
    #wait_for_seconds(0.5)
    #craneUp()
    #wait_for_seconds(0.5)
    #craneDown()

    motor_pair.move(3, 'in', steering=20, speed=-30)

    #next_state = "wandering"
    next_state = "done"
    hub.status_light.on('white')

#Pickup state. Runs every loop as long as state == 'picking_up'
def pickUp():
    global next_state, backing_up, has_object
    
    print("In pick-up")
    hub.status_light.on('green')
    motor_pair.move(360, unit='degrees', steering=100, speed=15)
    motor_pair.move(5, 'in', steering=0, speed=15)
    craneDown()
    next_state = 'searching'
    has_object = True
    hub.status_light.on('white')
    backing_up = False
        
#Main loop of program.  Decides what functions to run based on state and last_state.
#Runs until duration has passed or robot completes tasks
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


#Calibrate
calibrateDropoffSensor()
liftCalibration()

#Decide initial state based on if robot has item or not
if has_object:
    next_state = 'searching'
    craneDown()
else:
    next_state = 'wandering'
    craneUp()

#Begin main loop
mainLoop(200)