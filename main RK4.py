# WEB SERVER IMPORTS
from flask import Flask, render_template
from flask_socketio import SocketIO

# MAIN LIBRARY IMPORTS
import time
from bmp180 import BMP180
from mpu6050 import mpu6050
from RK4Tracker import Combined_RK4


# LIBRARY OBJECT DEFINITIONS
bmp = BMP180()
mpu = mpu6050(0x68) # 0x68 is the address for the MPU6050 on the I2C bus.
calibrationSamples = 500
calibrationTime = 0.005
global accelBias
accelBias = [0.0, 0.0, 0.0]
tracker = Combined_RK4(initial_angles=[0, 0, 0], alpha=0.98, damping=1.0)


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
            'orientation': orientationList
        }
    )
    
def update_sensor_data():
    global pressure, altitude, temperature
    global position, velocity, orientation
    global last_data_update_time
    global lastAccelList, lastGyroList

    pressure = bmp.get_pressure() 
    altitude = bmp.get_altitude() 
    temperature = bmp.get_temperature() 

    dt = time.monotonic() - last_data_update_time    
    lastAccelList = correctedAccel()
    lastGyroList = correctedGyro

    accelState, orientation = tracker.update(lastAccelList, lastGyroList, dt)
    
    position = accelState[0]
    velocity = accelState[1]
    last_data_update_time = time.monotonic()




def loop(): # MAIN LOOP FUNCTION
    calibrateAccel()
    global last_html_update_time
    while True:
        currentTime = time.monotonic()
        if (currentTime - last_data_update_time) >= 0.001: # update the sensor data if the minimum time has passed between readings
            update_sensor_data()
    
        currentTime = time.monotonic()
        if currentTime - last_html_update_time >= html_update_interval: # update HTML if its been long enough
            last_html_update_time = currentTime
            update_html()


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
    print('Client connected')
    socketio.start_background_task(target=loop)

# This function is called
def main():
    # These specific arguments are required to make sure the webserver is hosted in a consistent spot, so don't change them unless you know what you're doing.
    socketio.run(app, host='0.0.0.0', port=80, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    main()
