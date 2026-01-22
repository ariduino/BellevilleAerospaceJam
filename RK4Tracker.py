import math
import numpy as np

class RK4_Accel:
    '''Uses RK4 to integrate acceleration into velocity and position'''
    def __init__(self, initial_a=0, initial_v=0, initial_s=0):
        self.state = np.array([initial_s, initial_v]) # State vector: [position, velocity]
        self.accel_prev = initial_a

    def f(self, state, accel):
        """State derivative function: dy/dt = f[v, a]"""
        velocity = state[1]
        return np.array([velocity, accel])

    def update(self, accel_now, dt):
        '''Takes an accelearation reading and time step then returns updated position and velocity'''
        accel_mid = (self.accel_prev + accel_now) / 2.0 # midpoint of previous and current acceleration
        
        # RK4 Steps
        # k values are vectors: [position_slope, velocity_slope]
        k1 = self.f(self.state, self.accel_prev)
        k2 = self.f(self.state + (dt / 2.0) * k1, accel_mid)
        k3 = self.f(self.state + (dt / 2.0) * k2, accel_mid)
        k4 = self.f(self.state + dt * k3, accel_now)
        
        # Apply weighted average update to state vector
        self.state += (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        
        # Store for next iteration
        self.accel_prev = accel_now
        
        return self.state

class RK4_Gyro:
    '''Uses RK4 to integrate angular velocity into angles'''
    def __init__(self, initial_angles=None, alpha=0.98):
        # State vector: [angle_x, angle_y, angle_z]
        if initial_angles is None:
            self.state = np.array([0.0, 0.0, 0.0])
        else:
            self.state = np.array(initial_angles)
        # Previous angular velocity (gyro readings)
        self.gyro_prev = np.array([0.0, 0.0, 0.0])
        self.alpha = alpha

    def update(self, accel_now, gyro_now, dt):
        '''Takes a gyro reading, accelerometer reading, and time step then returns updated angles'''
        # 1. RK4 Integration (all in degrees)
        # gyro_now is in deg/s, dt is in seconds
        gyro_now = np.array(gyro_now)
        gyro_mid = (self.gyro_prev + gyro_now) / 2.0
        
        # RK4 weighted change in degrees
        delta_angle = (dt / 6.0) * (self.gyro_prev + 4*gyro_mid + gyro_now)
        gyro_predicted_angle = self.state + delta_angle

        # 2. Accelerometer Angle (Convert Radians -> Degrees)
        accel_roll_deg = math.atan2(accel_now[1], accel_now[2]) * 180 / math.pi
        accel_pitch_deg = math.atan2(-accel_now[0], math.sqrt(accel_now[1]**2 + accel_now[2]**2)) * 180 / math.pi
        accel_angles = np.array([accel_roll_deg, accel_pitch_deg, 0.0]) # No yaw from accel

        # 3. Fuse in Degrees
        # Alpha (e.g., 0.98) trusts the gyro for short-term changes
        self.state[0:2] = self.alpha * gyro_predicted_angle[0:2] + (1 - self.alpha) * accel_angles[0:2]
        
        # Yaw (Z) cannot be corrected by accelerometer; use pure gyro integration
        self.state[2] = gyro_predicted_angle[2] 

        self.gyro_prev = gyro_now
        return self.state

import math
import numpy as np

class Combined_RK4:
    '''Uses RK4 to integrate acceleration into velocity/position and angular velocity into angles'''
    def __init__(self, initial_angles=None, alpha=0.98, damping=1):
        """
        Initializes a 3D RK4 Tracker.
        - AccelState: 2x3 matrix [[pos_x, pos_y, pos_z], 
                                  [vel_x, vel_y, vel_z]]
        - GyroState:  1x3 vector [roll, pitch, yaw]
        - damping: Velocity decay factor to mitigate integration drift
        """ 
        self.AccelState = np.zeros((2, 3), dtype=float)
        self.accel_prev = np.array([0.0, 0.0, 0.0])

        if initial_angles is None:
            self.GyroState = np.array([0.0, 0.0, 0.0], dtype=float)
        else:
            self.GyroState = np.array(initial_angles, dtype=float)
            
        self.gyro_prev = np.array([0.0, 0.0, 0.0])
        self.alpha = alpha
        self.damping = damping # Fix for long-term drift

    def f(self, state, accel):
        """
        State derivative function: dy/dt = [velocity, acceleration]
        - state[0]: Position vector
        - state[1]: Velocity vector
        """
        # The rate of change of Position is Velocity
        vel_slope = state[1] 
        # The rate of change of Velocity is Acceleration
        accel_slope = accel 
        
        return np.vstack((vel_slope, accel_slope))
    
    def update(self, gyro_now, accel_now, dt):
        gyro_now = np.array(gyro_now)
        accel_now = np.array(accel_now)
        
        # 1. Update Orientation (Gyro + Complementary Filter)
        gyro_mid = (self.gyro_prev + gyro_now) / 2.0
        delta_angle = (dt / 6.0) * (self.gyro_prev + 4*gyro_mid + gyro_now)
        gyro_predicted_angle = self.GyroState + delta_angle

        # Accelerometer absolute reference (Low-Pass)
        accel_roll_deg = math.atan2(accel_now[1], accel_now[2]) * 180 / math.pi
        accel_pitch_deg = math.atan2(-accel_now[0], math.sqrt(accel_now[1]**2 + accel_now[2]**2)) * 180 / math.pi
        
        # Fuse orientation
        self.GyroState[0:2] = self.alpha * gyro_predicted_angle[0:2] + (1 - self.alpha) * np.array([accel_roll_deg, accel_pitch_deg])
        self.GyroState[2] = gyro_predicted_angle[2] 

        # 2. Gravity Compensation
        # Convert fused angles to radians for trig
        roll_rad  = math.radians(self.GyroState[0])
        pitch_rad = math.radians(self.GyroState[1])
        
        gravity_body = np.array([
            -9.81 * math.sin(pitch_rad),
            9.81 * math.sin(roll_rad) * math.cos(pitch_rad),
            9.81 * math.cos(roll_rad) * math.cos(pitch_rad)
        ])

        # Isolate Linear Acceleration
        lin_accel_now = accel_now - gravity_body 

        # 3. Position/Velocity Integration (RK4)
        accel_mid = (self.accel_prev + lin_accel_now) / 2.0
        
        k1 = self.f(self.AccelState, self.accel_prev)
        k2 = self.f(self.AccelState + (dt / 2.0) * k1, accel_mid)
        k3 = self.f(self.AccelState + (dt / 2.0) * k2, accel_mid)
        k4 = self.f(self.AccelState + dt * k3, lin_accel_now)
        
        self.AccelState += (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        
        # 4. Apply Velocity Damping (Fix for "Runaway" Amplitude)
        # We dampen the velocity (row 1) but leave position (row 0) alone
        self.AccelState[1] *= self.damping
        
        # Cleanup
        self.accel_prev = lin_accel_now
        self.gyro_prev = gyro_now

        return self.AccelState, self.GyroState

