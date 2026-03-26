# WEB SERVER IMPORTS
from flask import Flask, render_template
from flask_socketio import SocketIO

# MAIN LIBRARY IMPORTS
import time
from bmp180 import BMP180
from mpu6050 import mpu6050
from RK4Tracker import Combined_RK4
from picamera2 import Picamera2
import io
import base64
import RPi.GPIO as GPIO

# LIBRARY OBJECT DEFINITIONS
bmp = BMP180()
mpu = mpu6050(0x68) # 0x68 is the address for the MPU6050 on the I2C bus.
calibrationSamples = 500
calibrationTime = 0.005
global accelBias
accelBias = [0.0, 0.0, 0.0]
tracker = Combined_RK4(initial_angles=[0, 0, 0], alpha=0.98, damping=1.0)
cam = Picamera2()
camera_config = cam.create_preview_configuration(main={"size": (640, 480)})
cam.configure(camera_config)
cam.start()
camera_fps = 2
camera_interval = 1.0 / camera_fps
last_camera_time = time.monotonic()

# MOTOR GPIO SETUP
IN1 = 12
IN2 = 13
ENA = 14
motor_speed = 25
motor_position_s = 0.0
motor_max_s = 10.0  # Max allowed extension time from top to bottom.
motor_direction = 0  # 1=down, -1=up, 0=stopped
motor_until = None # define the variable but leave it empty for now
motor_last_update = time.monotonic()
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
pwm = GPIO.PWM(ENA, 100)
pwm.start(0)

# SENSOR VALUE VARIABLES (we define them at the top so that they are global)
global pressure, altitude, temperature
global position, velocity, orientation
global lastAccelList, lastGyroList
lastAccelList = []
lastGyroList = []
x, y, z = 0, 1, 2 # indexes for easier reading of accel, vel, pos, gyro, and orientation lists (now its accel[x] instead of accel[0])

# UPDATE TIME VARIABLE
last_html_update_time = time.monotonic() # time.monotonic() always counts up in seconds. We can use this to keep track of when we last sent sensor data to the HTML page.
html_update_interval = 1.0 # This is the interval (in seconds) at which we will send data to the HTML page.
last_data_update_time = time.monotonic()

# FLASK APP SETUP
app = Flask(__name__) # Here, we create the neccesary base app. You don't need to worry about this.
socketio = SocketIO(app)
background_loop_started = False # prevents starting two loops at once
@app.route('/') # When someone requests the root page from our web server, we return 'index.html'.
def index():
    return render_template('index.html')

# This function runs in the background to transmit data to connected clients.
def update_html():
    orientationList = orientation.tolist()
    for i in range (0,3):
        orientationList[i] = round(orientationList[i], 3)
        
    velocityList = velocity.tolist()
    for i in range (0,3):
        velocityList[i] = round(velocityList[i], 3)
        
    positionList = position.tolist()
    for i in range (0,3):
        positionList[i] = round(positionList[i], 3)
        
    socketio.emit(  # Then, we emit an event called "update_data" - but this can actually be whatever we want - with the data being a dictionary
        'update_data',
        {
            'barometricPressure': pressure,
            'temperature': temperature,
            'altitude': altitude,
            'rawAcceleration': lastAccelList,
            'linearAcceleration': [0, 0, 0],
            'velocity': velocityList,
            'position': positionList,
            'rawGyroscope': lastGyroList,
            'orientation': orientationList,
            'bias': accelBias

        }
    )
    
def update_sensor_data():
    global pressure, altitude, temperature
    global position, velocity, orientation
    global last_data_update_time
    global lastAccelList, lastGyroList

    pressure = round(bmp.get_pressure(), 2)
    altitude = round(44330*(1-(pressure/101325)**0.1903), 2)
    temperature = round(bmp.get_temperature(), 2)


    dt = time.monotonic() - last_data_update_time    
    lastAccelList = correctedAccel()
    lastGyroList = correctedGyro()

    accelState, orientation = tracker.update(lastAccelList, lastGyroList, dt)
    
    position = accelState[0]
    velocity = accelState[1]
    last_data_update_time = time.monotonic()




def loop(): # MAIN LOOP FUNCTION
    # calibrateAccel() TEMPORARILY DISABLED FOR TESTING
    global last_html_update_time, last_camera_time, last_data_update_time
    while True:
        currentTime = time.monotonic()
        update_motor_state(currentTime)

        currentTime = time.monotonic()
        if (currentTime - last_data_update_time) >= 0.001: # update the sensor data if the minimum time has passed between readings
            update_sensor_data()
    
        currentTime = time.monotonic()
        if currentTime - last_html_update_time >= html_update_interval: # update HTML if its been long enough
            last_html_update_time = currentTime
            update_html()
        
        currentTime = time.monotonic()
        if currentTime - last_camera_time >= camera_interval:
            last_camera_time = currentTime
            send_image()


def calibrateAccel():
    global accelBias
    
    sum_accel = [0.0, 0.0, 0.0]
    
    for i in range(calibrationSamples):
        a = mpu.get_accel_data()
        sum_accel[x] += a['x']
        sum_accel[y] += a['y']
        sum_accel[z] += a['z']
        time.sleep(calibrationTime)

    avg_accel = [s / calibrationSamples for s in sum_accel]

    accelBias[x] = avg_accel[x] - 0.0
    accelBias[y] = avg_accel[y] - 0.0
    accelBias[z] = avg_accel[z] - 9.81

    print("Accel bias:", accelBias)

def correctedAccel():
    raw = mpu.get_accel_data()
    return [
        round(raw['x'] - accelBias[x], 3),
        round(raw['y'] - accelBias[y], 3),
        round(raw['z'] - accelBias[z], 3)
    ]

def correctedGyro():
    raw = mpu.get_gyro_data()
    return [
        round(raw['x'], 3),
        round(raw['y'], 3),
        round(raw['z'], 3)
    ]

# This function runs when someone connects to the server - and all we do is start the background thread to update the data.
@socketio.on('connect')
def handle_connect():
    global background_loop_started
    print('Client connected')
    if not background_loop_started:
        background_loop_started = True
        socketio.start_background_task(target=loop)
    emit_motor_status()

@socketio.on('clip_pressure')
def send_pressure():
    global pressure
    print("Clipping pressure...")
    socketio.emit('update_pressure', {'pressure': pressure})

# @socketio.on('request_image')
def send_image():
    stream = io.BytesIO()
    cam.capture_file(stream, format='jpeg')
    stream.seek(0)
    b64_image = base64.b64encode(stream.read()).decode('utf-8')
    socketio.emit('new_image', {'image_data': b64_image})

def motorDown():
    global motor_speed
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(motor_speed)
def motorUp():
    global motor_speed
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm.ChangeDutyCycle(motor_speed)
def motorStop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(0)

def emit_motor_status(): # Update the html page with the current motor status
    state = "stopped"
    if motor_direction == 1:
        state = "down"
    elif motor_direction == -1:
        state = "up"

    socketio.emit(
        "motor_status",
        {
            "position_s": round(motor_position_s, 3), # current time postition of the motor
            "max_s": motor_max_s, # max time possition
            "state": state, # current motor direction/state
        },
    )

def stop_motor_command():
    global motor_direction, motor_until
    motor_direction = 0
    motor_until = None
    motorStop()
    emit_motor_status()

def start_motor_command(direction, duration_s=None):
    global motor_direction, motor_until

    if direction not in ("up", "down"): # dont continue if the motor is supposed to be stopped
        return False

    if direction == "up": # calculate how long the motor can go up or down without passing the limits
        remaining = motor_position_s
    else:
        remaining = motor_max_s - motor_position_s
    
    if remaining <= 0: # stop if its at or over the limit.
        stop_motor_command()
        return False

    if duration_s is not None: 
        duration_s = max(0.0, min(float(duration_s), remaining)) # Sets how long the motor will spin for. Also has logic to ensrure the motor doesnt excede its limits or go negative.
        if duration_s <= 0: # stop the motor if its reached its destination
            stop_motor_command()
            return False
        motor_until = time.monotonic() + duration_s # caluclate the system time at which the motor should stop
    else:
        motor_until = None # sets the motor run duration to None if there is no duration set by the function call.

    if direction == "up":
        motorUp()
        motor_direction = -1
    else:
        motorDown()
        motor_direction = 1

    emit_motor_status()
    return True

def update_motor_state(now): # keeps track of the motors position in time
    global motor_position_s, motor_last_update
    dt = now - motor_last_update
    if dt < 0:
        dt = 0

    if motor_direction == 1: # increment the time position based on its direction.
        motor_position_s += dt
    elif motor_direction == -1:
        motor_position_s -= dt

    hit_limit = False
    if motor_position_s <= 0: # checks to see if the motor has passed its limits.
        motor_position_s = 0.0
        hit_limit = (motor_direction == -1)
    elif motor_position_s >= motor_max_s:
        motor_position_s = motor_max_s
        hit_limit = (motor_direction == 1)

    timed_out = False # checks to see if the motor has reached its run time
    if motor_until is not None:
        if now >= motor_until:
            timed_out = True
    motor_last_update = now

    if hit_limit or timed_out: # if the motor has reached its limits or run time the motor is stopped.
        stop_motor_command()

@socketio.on("motor_step")
def handle_motor_step(msg): # Handles the html page's call to move the motor in a direction for a specified amount of time
    direction = (msg or {}).get("dir")
    duration_ms = (msg or {}).get("duration_ms", 1000)
    duration_s = max(0.0, float(duration_ms) / 1000.0)
    start_motor_command(direction, duration_s=duration_s)

@socketio.on("motor_hold_start")
def handle_motor_hold_start(msg): # runs the motor in a direction until the "motor_hold_stop" command is sent from the html page
    direction = (msg or {}).get("dir")
    start_motor_command(direction, duration_s=None)

@socketio.on("motor_hold_stop")
def handle_motor_hold_stop(): # stops the motor from running. Used to stop after the "motor_hold_start" command is sent from the html page
    stop_motor_command()

@socketio.on("motor_stop")
def handle_motor_stop(): # stops the motor immediately.
    stop_motor_command()

@socketio.on("motor_go_top")
def handle_motor_go_top(): # moves the motor to the top position
    start_motor_command("up", duration_s=motor_position_s)

@socketio.on("motor_go_bottom")
def handle_motor_go_bottom(): # move the motor to the bottom position
    start_motor_command("down", duration_s=(motor_max_s - motor_position_s))

def main():
    # These specific arguments are required to make sure the webserver is hosted in a consistent spot, so don't change them unless you know what you're doing.
    socketio.run(app, host='0.0.0.0', port=80, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    main()
