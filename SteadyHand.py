# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# Parkinsons_Spoon_With_Native_PWM
# Copyright (c) 2025 Neel Vasa. All rights reserved.
# Parkinsons_Spoon_With_Native_PWM™ is a trademark of Neel Vasa.
# -----------------------------------------------------------------------------
# This file contains functions related to the gyroscopic stabilization of a utensil used by Parkinson's patients experiencing tremors.

import machine
import time
import ustruct 
import math
from machine import Pin, I2C, PWM

# 1. CONFIGURATION

# MPU6050 I2C Setup 
I2C_BUS = 1 
I2C_SDA_PIN = 6 
I2C_SCL_PIN = 7
I2C_FREQUENCY = 100000

# Servo Pin Assignment
SERVO_PIN_1 = 4 # Vertical (Roll) 
SERVO_PIN_2 = 5 # Horizontal (Pitch) 
SERVO_FREQ = 50 

# Servo Angle Constants (Pulse width in NANOSECONDS for 0, 90, 180 degrees)
PULSE_MIN_NS = 500000   # 0 degrees
PULSE_MID_NS = 1500000  # 90 degrees
PULSE_MAX_NS = 2500000  # 180 degrees

# Stabilization Parameters (PI Controller Gains)
KP = 0.945    # Proportional Gain (Response speed)
KI = 0.05   # Integral Gain (Eliminates steady-state error)
DEADBAND_DEGREES = 0.5 
NON_LINEAR_GAIN = 0.00105 
SMOOTHING_FACTOR = 0.98 

# Safety Limits: CRITICAL for preventing motor spin/stalling
MIN_SERVO_ANGLE = 5.0  
MAX_SERVO_ANGLE = 175.0 

# --- CALIBRATION OFFSETS (ADJUST THESE!) ---
# 1. Hold the handle perfectly still in its desired resting position (e.g., vertical).
# 2. Look at the console output: "CALIBRATION: Raw Roll: X.XX | Raw Pitch: Y.YY"
# 3. Adjust the offsets below to be the NEGATIVE of those values
ROLL_OFFSET_DEG = 07.0
PITCH_OFFSET_DEG = 7.0

# Integral Limit (Anti-Windup Safety)
MAX_INTEGRAL_ERROR = 1.0 

# 2. MPU6050 DRIVER CLASS

class MPU6050:
    MPU_ADDR = 0x68
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    
    def __init__(self, i2c):
        self.i2c = i2c
        self.roll = 0.0 
        self.pitch = 0.0 
        
        self.roll_integral = 0.0
        self.pitch_integral = 0.0
        
        # Reset MPU6050
        self.i2c.writeto(self.MPU_ADDR, bytes([self.PWR_MGMT_1, 0]))
        time.sleep_ms(5) 

    def read_raw_data(self):
        for i in range(10): 
            try:
                # Read 14 bytes (Accels, Temp, Gyros)
                data = self.i2c.readfrom_mem(self.MPU_ADDR, self.ACCEL_XOUT_H, 14)
                ax, ay, az, temp, gx, gy, gz = ustruct.unpack('>hhhhhhh', data)
                return (ax, ay, az, gx, gy, gz)
            except OSError as e:
                # Retry only on I2C timeouts (Errno 110) or I/O errors (Errno 5)
                if e.args[0] == 110 or e.args[0] == 5:
                    time.sleep_ms(2 + i * 2) # Exponential backoff delay on retry
                    continue
                else:
                    raise e
        # If all retries fail, raise a specific error
        raise OSError("[Errno 110] MPU6050 I2C TIMEOUT/EIO. **POWER ISOLATION RECOMMENDED.**")

    def calculate_angles(self, ax, ay, az):
        roll_accel = math.atan2(ay, az) * 180 / math.pi
        pitch_accel = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 180 / math.pi
        return roll_accel, pitch_accel

    def read_stabilized_angles(self, dt):
        ax, ay, az, gx, gy, gz = self.read_raw_data()
        
        # Calculate angle from accelerometer data (16384.0 is the sensitivity factor)
        roll_accel, pitch_accel = self.calculate_angles(ax / 16384.0, ay / 16384.0, az / 16384.0) 
        
        # Calculate angle rate from gyroscope data (131.0 is the sensitivity factor)
        roll_rate = gx / 131.0
        pitch_rate = gy / 131.0
        
        # Complementary Filter: Fuses gyro rate (fast) and accelerometer angle (stable)
        self.roll = (self.roll + roll_rate * dt) * SMOOTHING_FACTOR + roll_accel * (1.0 - SMOOTHING_FACTOR)
        self.pitch = (self.pitch + pitch_rate * dt) * SMOOTHING_FACTOR + pitch_accel * (1.0 - SMOOTHING_FACTOR)
        
        return self.roll, self.pitch
        
# 3. HELPER FUNCTIONS

def set_servo_angle(servo_pwm, angle_degrees):
    # Clamps angle between 0 and 180 degrees (pulse width limits)
    if angle_degrees < 0: angle_degrees = 0
    if angle_degrees > 180: angle_degrees = 180
    
    pulse_range_ns = PULSE_MAX_NS - PULSE_MIN_NS
    
    # Calculate pulse width in nanoseconds
    pulse_ns = PULSE_MIN_NS + (angle_degrees / 180.0) * pulse_range_ns
    
    # Set the duty cycle
    servo_pwm.duty_ns(int(pulse_ns))

# 4. MAIN CONTROL LOOP

def stabilize_loop():
    
    # Hardware Initialization
    
    try:
        # Initialize I2C Bus for MPU6050
        i2c = I2C(I2C_BUS, sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN), freq=I2C_FREQUENCY)
        
        # Check for MPU6050
        devices = i2c.scan()
        if MPU6050.MPU_ADDR not in devices:
            print("ERROR: MPU6050 not found. Check GP6/GP7 wiring.")
            return

        mpu = MPU6050(i2c)
        
        # Initialize Native PWM for Servos
        servo1_pwm = PWM(Pin(SERVO_PIN_1))
        servo2_pwm = PWM(Pin(SERVO_PIN_2))
        
        servo1_pwm.freq(SERVO_FREQ)
        servo2_pwm.freq(SERVO_FREQ)
        
        servos = {
            'roll': servo1_pwm, 
            'pitch': servo2_pwm
        }
        
    except Exception as e:
        print(f"Hardware Initialization Error: {e}")
        return 

    # Set initial servo position to center (90 for roll, 90 for pitch)
    set_servo_angle(servos['roll'], 90)
    set_servo_angle(servos['pitch'], 90)
    time.sleep(0.5) 
    
    # Stabilization Loop
    
    last_time = time.ticks_ms()
    debug_time = time.ticks_ms()

    while True:
        try:
            current_time = time.ticks_ms()
            dt = (current_time - last_time) / 1000.0
            last_time = current_time
            
            # 1. Read Stabilized Angles
            roll_angle_raw, pitch_angle_raw = mpu.read_stabilized_angles(dt)

            # Apply Electronic Offset Correction
            # This is the angle error that should be corrected when the handle is vertical.
            roll_angle = roll_angle_raw + ROLL_OFFSET_DEG
            pitch_angle = pitch_angle_raw + PITCH_OFFSET_DEG

            # DEBUGGING: Print offset-corrected angles every second for calibration
            if time.ticks_diff(current_time, debug_time) > 1000:
                # The output below shows the angle the stabilizer is *trying* to correct to zero.
                # If the platform is level, these values should be close to zero.
                print(f"CALIBRATION: Corrected Roll: {roll_angle:.2f} | Corrected Pitch: {pitch_angle:.2f}")
                debug_time = current_time
            
            # 2. Apply Deadband and Calculate Correction Angle using PI Control
            
            # Roll Axis (Vertical Stabilization)
            if abs(roll_angle) < DEADBAND_DEGREES:
                roll_angle = 0.0
            
            mpu.roll_integral += roll_angle * dt
            # Integral anti-windup clamping
            mpu.roll_integral = max(min(mpu.roll_integral, MAX_INTEGRAL_ERROR), -MAX_INTEGRAL_ERROR)
            
            P_scale_factor_1 = 1.0 + NON_LINEAR_GAIN * abs(roll_angle)
            
            correction_1 = -1 * ((roll_angle * KP * P_scale_factor_1) + (mpu.roll_integral * KI)) 

            # Pitch Axis (Horizontal Stabilization)
            if abs(pitch_angle) < DEADBAND_DEGREES:
                pitch_angle = 0.0
                
            mpu.pitch_integral += pitch_angle * dt
            # Integral anti-windup clamping
            mpu.pitch_integral = max(min(mpu.pitch_integral, MAX_INTEGRAL_ERROR), -MAX_INTEGRAL_ERROR)
            
            P_scale_factor_2 = 1.0 + NON_LINEAR_GAIN * abs(pitch_angle)
            
            # Pitch Correction
            correction_2 = (pitch_angle * KP * P_scale_factor_2) + (mpu.pitch_integral * KI)
            
            # Limit instantaneous correction to +/- 90 degrees
            MAX_CORRECTION = 90.0
            correction_1 = max(min(correction_1, MAX_CORRECTION), -MAX_CORRECTION)
            correction_2 = max(min(correction_2, MAX_CORRECTION), -MAX_CORRECTION)
            
            # 3. Determine New Servo Position 
            new_angle_1 = 90 + correction_1
            new_angle_2 = 90 + correction_2

            # Safety Clamping (CRITICAL for stability)
            # Ensures the motors do not command outside of MIN/MAX_SERVO_ANGLE limits
            new_angle_1 = max(min(new_angle_1, MAX_SERVO_ANGLE), MIN_SERVO_ANGLE) 
            new_angle_2 = max(min(new_angle_2, MAX_SERVO_ANGLE), MIN_SERVO_ANGLE) 
            
            # 4. Command Servos via Native PWM
            set_servo_angle(servos['roll'], new_angle_1)
            set_servo_angle(servos['pitch'], new_angle_2)
            
        except OSError as e:
            # Handles I2C communication errors (likely related to power/motor noise)
            if e.args[0] == 5:
                # Pass silently on intermittent I/O error
                pass 
            else:
                # Use the MPU6050's built-in error message for critical failure
                if e.args[0] == 110:
                    raise e
                else:
                    # Re-raise any other unknown OSError
                    raise e

        # Short delay to manage loop speed.
        time.sleep(0.01) 

if __name__ == '__main__':
    try:
        stabilize_loop()
    except KeyboardInterrupt:
        print("\nStabilizer stopped by user.")

