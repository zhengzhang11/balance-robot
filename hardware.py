import smbus
import time
import math
import matplotlib.pyplot as plt
import numpy as np
from threading import Thread, Event

# MPU6050 Register Addresses
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

class MPU6050:
    def __init__(self, bus=1, address=0x68):
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.gyro_scale = 131.0  # Corresponds to ±250°/s range
        self.accel_scale = 16384.0  # Corresponds to ±2g range
        self.dt = 0.01  # Sampling interval (seconds)
        self.gyro_offset_x = 0
        self.gyro_offset_y = 0
        self.gyro_offset_z = 0
        self.accel_offset_x = 0
        self.accel_offset_y = 0
        self.accel_offset_z = 0
        
        # Initialize MPU6050
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0)  # Wake up MPU6050
        self.bus.write_byte_data(self.address, SMPLRT_DIV, 7)  # Sample rate divisor
        self.bus.write_byte_data(self.address, CONFIG, 0)  # Configure low-pass filter
        self.bus.write_byte_data(self.address, GYRO_CONFIG, 0)  # Gyroscope range ±250°/s
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, 0)  # Accelerometer range ±2g
        self.bus.write_byte_data(self.address, INT_ENABLE, 1)  # Enable interrupts
        
        # Calibrate gyroscope
        self.calibrate_gyro()
        # Calibrate accelerometer
        self.calibrate_accel()
        
        # Complementary filter parameters
        self.alpha = 0.98  # Complementary filter coefficient
        self.angle_x = 0  # Complementary filter angle X
        self.angle_y = 0  # Complementary filter angle Y
        
        # Data storage
        self.timestamps = []
        self.accel_angles_x = []
        self.gyro_angles_x = []
        self.complimentary_angles_x = []
        
        # Thread control
        self.stop_event = Event()
        self.collection_thread = None
    
    def read_word_2c(self, register):
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            return -((65535 - value) + 1)
        else:
            return value
    
    def calibrate_gyro(self, samples=100):
        print("Calibrating gyroscope, please keep the sensor stationary...")
        sum_x = 0
        sum_y = 0
        sum_z = 0
        
        for _ in range(samples):
            gyro_x = self.read_word_2c(GYRO_XOUT_H)
            gyro_y = self.read_word_2c(GYRO_XOUT_H + 2)
            gyro_z = self.read_word_2c(GYRO_XOUT_H + 4)
            
            sum_x += gyro_x
            sum_y += gyro_y
            sum_z += gyro_z
            
            time.sleep(0.01)
        
        self.gyro_offset_x = sum_x / samples
        self.gyro_offset_y = sum_y / samples
        self.gyro_offset_z = sum_z / samples
        
        print(f"Gyroscope calibration complete: X={self.gyro_offset_x}, Y={self.gyro_offset_y}, Z={self.gyro_offset_z}")
    
    def calibrate_accel(self, samples=100):
        print("Calibrating accelerometer, please keep the sensor stationary...")
        sum_x = 0
        sum_y = 0
        sum_z = 0
        
        for _ in range(samples):
            accel_x = self.read_word_2c(ACCEL_XOUT_H)
            accel_y = self.read_word_2c(ACCEL_XOUT_H + 2)
            accel_z = self.read_word_2c(ACCEL_XOUT_H + 4)
            
            sum_x += accel_x
            sum_y += accel_y
            sum_z += accel_z
            
            time.sleep(0.01)
        
        # The Z-axis of the accelerometer should be approximately 1g when stationary
        self.accel_offset_x = sum_x / samples
        self.accel_offset_y = sum_y / samples
        self.accel_offset_z = (sum_z / samples) - self.accel_scale
        
        print(f"Accelerometer calibration complete: X={self.accel_offset_x}, Y={self.accel_offset_y}, Z={self.accel_offset_z}")
    
    def get_raw_data(self):
        # Read accelerometer data
        accel_x = self.read_word_2c(ACCEL_XOUT_H) - self.accel_offset_x
        accel_y = self.read_word_2c(ACCEL_XOUT_H + 2) - self.accel_offset_y
        accel_z = self.read_word_2c(ACCEL_XOUT_H + 4) - self.accel_offset_z
        
        # Read gyroscope data
        gyro_x = self.read_word_2c(GYRO_XOUT_H) - self.gyro_offset_x
        gyro_y = self.read_word_2c(GYRO_XOUT_H + 2) - self.gyro_offset_y
        gyro_z = self.read_word_2c(GYRO_XOUT_H + 4) - self.gyro_offset_z
        
        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    
    def get_angles(self):
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.get_raw_data()
        
        # Convert to actual units
        accel_x = accel_x / self.accel_scale
        accel_y = accel_y / self.accel_scale
        accel_z = accel_z / self.accel_scale
        
        gyro_x = gyro_x / self.gyro_scale
        gyro_y = gyro_y / self.gyro_scale
        gyro_z = gyro_z / self.gyro_scale
        
        # 1. Calculate angle using only accelerometer
        accel_angle_x = math.atan2(accel_y, accel_z) * 180 / math.pi
        accel_angle_y = math.atan2(-accel_x, math.sqrt(accel_y * accel_y + accel_z * accel_z)) * 180 / math.pi
        
        # 2. Calculate angle using only gyroscope integration
        self.gyro_angle_x += gyro_x * self.dt
        self.gyro_angle_y += gyro_y * self.dt
        
        # 3. Complementary filter fusion of both sensors
        self.angle_x = self.alpha * (self.angle_x + gyro_x * self.dt) + (1 - self.alpha) * accel_angle_x
        self.angle_y = self.alpha * (self.angle_y + gyro_y * self.dt) + (1 - self.alpha) * accel_angle_y
        
        return accel_angle_x, accel_angle_y, self.gyro_angle_x, self.gyro_angle_y, self.angle_x, self.angle_y
    
    def collect_data(self, duration=10):
        """Collect data for the specified duration"""
        print(f"Starting data collection, duration: {duration} seconds")
        self.gyro_angle_x = 0
        self.gyro_angle_y = 0
        self.angle_x = 0
        self.angle_y = 0
        
        start_time = time.time()
        last_time = start_time
        
        while time.time() - start_time < duration and not self.stop_event.is_set():
            current_time = time.time()
            self.dt = current_time - last_time
            last_time = current_time
            
            # Get angle data
            accel_angle_x, _, gyro_angle_x, _, compl_angle_x, _ = self.get_angles()
            
            # Store data
            self.timestamps.append(current_time - start_time)
            self.accel_angles_x.append(accel_angle_x)
            self.gyro_angles_x.append(gyro_angle_x)
            self.complimentary_angles_x.append(compl_angle_x)
            
            # Control sampling frequency
            elapsed = time.time() - current_time
            if elapsed < self.dt:
                time.sleep(self.dt - elapsed)
        
        print("Data collection complete")
    
    def start_collection_thread(self, duration=10):
        """Start data collection in a separate thread"""
        self.collection_thread = Thread(target=self.collect_data, args=(duration,))
        self.collection_thread.daemon = True
        self.collection_thread.start()
    
    def stop_collection(self):
        """Stop data collection"""
        self.stop_event.set()
        if self.collection_thread:
            self.collection_thread.join(timeout=1.0)
    
    def plot_results(self):
        """Plot the angle comparison graph of the three methods"""
        if not self.timestamps:
            print("No data available for plotting")
            return
        
        plt.figure(figsize=(12, 8))
        
        # Plot the angle curves from the three methods
        plt.plot(self.timestamps, self.accel_angles_x, 'r-', label='Accelerometer Only')
        plt.plot(self.timestamps, self.gyro_angles_x, 'g-', label='Gyroscope Only (Integration)')
        plt.plot(self.timestamps, self.complimentary_angles_x, 'b-', label='Complementary Filter')
        
        plt.xlabel('Time (seconds)')
        plt.ylabel('Angle (degrees)')
        plt.title('Comparison of Angle Calculation Methods')
        plt.legend()
        plt.grid(True)
        
        # Display the graph
        plt.show()

if __name__ == "__main__":
    try:
        mpu = MPU6050()
        
        # Start data collection thread
        mpu.start_collection_thread(duration=10)
        
        # Wait for collection to complete
        while mpu.collection_thread.is_alive():
            time.sleep(0.1)
        
        # Plot results
        mpu.plot_results()
        
    except KeyboardInterrupt:
        print("Program interrupted by user")
        mpu.stop_collection()
    except Exception as e:
        print(f"An error occurred: {e}")
        mpu.stop_collection()    