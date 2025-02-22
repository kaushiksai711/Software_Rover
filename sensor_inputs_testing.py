import time
import random
from typing import Tuple, List, Dict
from rplidar import RPLidar  # For LIDAR integration
import RPi.GPIO as GPIO

from mpu6050 import mpu6050 # For IMU
import busio
import board
from adafruit_ina219 import INA219  # For current sensing
from adafruit_motor import servo
from adafruit_motorkit import MotorKit  # For motor control

class System:
    def __init__(self):
        self.sensors = {
            "lidar": False,
            "camera": False,
            "motors": False,
            "imu": False,
            "current_sensor": False,
            "encoders": False,
            "rtc": False
        }
        self.mode = "IDLE"
        self.initialize_hardware()
        
    def initialize_hardware(self):
        """Initialize all hardware components"""
        try:
            # Initialize LIDAR
            self.lidar = RPLidar('/dev/ttyUSB0')
            
            # Initialize IMU
            self.imu = mpu6050(0x68)
            
            # Initialize motor controllers
            self.motor_kit = MotorKit()
            
            # Initialize current sensor
            i2c = busio.I2C(board.SCL, board.SDA)
            self.current_sensor = INA219(i2c)
            
            # Initialize servo motors
            self.servo_mg995 = servo.Servo(board.D5)
            self.servo_sg90 = servo.Servo(board.D6)
            
            # Initialize encoder pins
            GPIO.setmode(GPIO.BCM)
            self.encoder_pins = {'left': 17, 'right': 18}
            for pin in self.encoder_pins.values():
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                
        except Exception as e:
            print(f"Hardware initialization error: {e}")
            return False
            
        return True

    def start_system(self):
        """Initialize and check all sensors with actual hardware checks"""
        print("Starting and checking sensors...")
        
        # Check LIDAR
        try:
            self.lidar.connect()
            self.sensors["lidar"] = True
        except Exception as e:
            print(f"LIDAR initialization failed: {e}")
            
        # Check IMU
        try:
            self.imu.get_accel_data()
            self.sensors["imu"] = True
        except Exception as e:
            print(f"IMU initialization failed: {e}")
            
        # Check motors
        try:
            # Test motor movement
            self.motor_kit.motor1.throttle = 0.1
            time.sleep(0.1)
            self.motor_kit.motor1.throttle = 0
            self.sensors["motors"] = True
        except Exception as e:
            print(f"Motor initialization failed: {e}")
            
        self.mode = "READY" if all(self.sensors.values()) else "ERROR"
        print(f"System is {self.mode}")

    def read_lidar(self):
        """Read data from LIDAR"""
        try:
            scan_data = []
            for scan in self.lidar.iter_scans():
                scan_data.extend(scan)
                if len(scan_data) > 360:  # One full rotation
                    break
            return scan_data
        except Exception as e:
            print(f"LIDAR read error: {e}")
            return None

    def read_imu(self):
        """Read data from IMU"""
        try:
            accel_data = self.imu.get_accel_data()
            gyro_data = self.imu.get_gyro_data()
            return {"accel": accel_data, "gyro": gyro_data}
        except Exception as e:
            print(f"IMU read error: {e}")
            return None

    def read_current_sensor(self):
        """Read data from current sensor"""
        try:
            current = self.current_sensor.current
            voltage = self.current_sensor.voltage
            power = self.current_sensor.power
            return {"current": current, "voltage": voltage, "power": power}
        except Exception as e:
            print(f"Current sensor read error: {e}")
            return None

# Example usage
if __name__ == "__main__":
    system = System()
    system.start_system()
    
    if system.sensors["lidar"]:
        lidar_data = system.read_lidar()
        print(f"LIDAR data: {lidar_data}")
    
    if system.sensors["imu"]:
        imu_data = system.read_imu()
        print(f"IMU data: {imu_data}")
    
    if system.sensors["current_sensor"]:
        current_data = system.read_current_sensor()
        print(f"Current sensor data: {current_data}")