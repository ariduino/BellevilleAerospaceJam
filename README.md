# Carson's-Killer-Drone-From-Belleville
Belleville HS Drone Team Codebase
https://docs.aerospacejam.org/

Basic sensor troubleshooting:
  - Make sure the piSugar is not set to on while plugged into power via the micro USB cable
  - If the the sensors are not working and are wired correclty i2c might need to be enabled
        run: "sudo raspi-config nonint get_i2c" to test if its enabled (0 is enabled, 1 disabled)
        if its disabled run "sudo raspi-config nonint do_i2c 0" to enable it

Pressure Sensor:
  - 2 points: display current ambient pressure
  - 4 points: measure the ambient pressure at a specific location
        A button to clip the current pressure and display it in a specfic box would work well for this (https://docs.aerospacejam.org/getting-started/first-interaction/)
  - 6 points: display height above sea level
        The library calculates this for us so it should be quite simple
  - 8 points: log changes in the height over time
        could be done with a graph seeing as we will need one for the MPU data

Accelerometer/Gyro Sensor:
  - 2 points: display current accel + gyro readings
  - 4 points: log and display multiple accel + gyro readings over time
        a table of the last 10 or so readings constantly updatiang should work
  - 8 points: live graph of the accel + gyro data
        The hard part will be the webserver side having to display a graph
  - 12 points: Integrate the MPU readings over time to keep track of the drones position over time (dead reckoning)
        Managing noise will be a huge challenge
        This value needs to be constantly updated (more than the once per second rate of the data sending) for accuracy
        Gravity needs to be accounted for

Graphing/logging data:
  - probably easiest to store our data in lists/arrays to send over in one piece
  - the same array of data can be used for both graphing and logging the data
  - there might be an easy graphing library for HTML out there already