# WEB SERVER IMPORTS
from flask import Flask, render_template
from flask_socketio import SocketIO

# MAIN LIBRARY IMPORTS
import time
import math
import numpy as np
from bmp180 import BMP180
from mpu6050 import mpu6050
from RK4Tracker import Combined_RK4


# LIBRARY OBJECT DEFINITIONS
bmp = BMP180()
mpu = mpu6050(0x68) # 0x68 is the address for the MPU6050 on the I2C bus.
tracker = Combined_RK4(initial_angles=[0, 0, 0], alpha=0.98, damping=1.0)


# SENSOR VALUE VARIABLES (we define them at the top so that they are global)
global pressure, altitude, temperature
global position, velocity, orientation
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
    socketio.emit(  # Then, we emit an event called "update_data" - but this can actually be whatever we want - with the data being a dictionary
        'update_data',
        {
            'barometricPressure': pressure,
            'temperature': temperature,
            'altitude': altitude,
            'rawAcceleration': mpu.get_accel_data(),
            'linearAcceleration': [0, 0, 0],
            'velocity': velocity,
            'position': position,
            'rawGyroscope': mpu.get_gyro_data(),
            'orientation': orientation
        }
    )

def update_sensor_data():
    global pressure, altitude, temperature
    global position, velocity, orientation
    global last_data_update_time
    # Read barometer
    pressure = bmp.get_pressure() 
    altitude = bmp.get_altitude() 
    temperature = bmp.get_temperature() 

    dt = time.monotonic() - last_data_update_time
    accelRaw = mpu.get_accel_data()
    accelList = [accelRaw['x'], accelRaw['y'], accelRaw['z']]
    gyroRaw = mpu.get_gyro_data()
    gyroList = [gyroRaw['x'], gyroRaw['y'], gyroRaw['z']]
    accelState, orientation = tracker.update(np.array(accelList), np.array(gyroList), dt)
    position = accelState[0]
    velocity = accelState[1]
    last_data_update_time = time.monotonic()




def loop(): # MAIN LOOP FUNCTION
    global last_html_update_time
    while True:
        currentTime = time.monotonic()
        if (currentTime - last_data_update_time) >= 0.001: # update the sensor data if the minimum time has passed between readings
            update_sensor_data()
    
        currentTime = time.monotonic()
        if currentTime - last_html_update_time >= html_update_interval: # update HTML if its been long enough
            last_html_update_time = currentTime
            update_html()


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

