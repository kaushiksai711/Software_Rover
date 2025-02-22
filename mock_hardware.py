import random
import math
from typing import Dict, List, Tuple

class MockLidar:
    def __init__(self):
        self.connected = False
        
    def connect(self):
        self.connected = True
        
    def disconnect(self):
        self.connected = False
        
    def stop(self):
        pass
        
    def iter_scans(self):
        # Generate mock scan data for a simple room with some obstacles
        mock_environment = [
            # Each entry: (quality, angle, distance in mm)
            # Simulating walls and obstacles
            *[(50, angle, 2000 + random.randint(-100, 100)) for angle in range(0, 360, 2)],  # Walls
            *[(50, angle, 500) for angle in range(45, 90, 2)],  # Obstacle 1
            *[(50, angle, 800) for angle in range(180, 200, 2)],  # Obstacle 2
        ]
        yield mock_environment

class MockIMU:
    def get_accel_data(self):
        return {
            'x': random.uniform(-1, 1),
            'y': random.uniform(-1, 1),
            'z': random.uniform(9.5, 10.5)
        }

class MockMotor:
    def __init__(self):
        self._throttle = 0
        
    @property
    def throttle(self):
        return self._throttle
        
    @throttle.setter
    def throttle(self, value):
        self._throttle = max(min(value, 1), -1)

class MockMotorKit:
    def __init__(self):
        self.motor1 = MockMotor()
        self.motor2 = MockMotor()

class MockCurrentSensor:
    def get_current(self):
        return random.uniform(0.1, 2.0)

class MockGPIO:
    BCM = "BCM"
    PUD_UP = "PUD_UP"
    IN = "IN"
    
    @staticmethod
    def setmode(mode):
        pass
        
    @staticmethod
    def setup(pin, mode, pull_up_down=None):
        pass
        
    @staticmethod
    def cleanup():
        pass

# Replace actual hardware imports with mocks
RPLidar = MockLidar
mpu6050 = MockIMU
MotorKit = MockMotorKit
INA219 = MockCurrentSensor
GPIO = MockGPIO
