import time
import random
from typing import Tuple, List, Dict
#install
## Note: D* Lite implementation would need to be added
from rplidar import RPLidar  # For LIDAR integration
try:
    import RPi.GPIO as GPIO
except ModuleNotFoundError:
    import RPi.GPIO_mock as GPIO

# Now you can use GPIO like normal for testing
    # For Raspberry Pi GPIO
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

class TerrainMapping:
    def __init__(self, lidar):
        self.lidar = lidar
        self.map_data = {
            "traversable_points": [],
            "non_traversable_points": [],
            "arena_boundary": []
        }
        self.arena_size = (1000, 1000)  # 10m x 10m in cm
        
    def scan_arena(self) -> Dict:
        """
        Actual LIDAR scanning implementation
        """
        print("Scanning terrain...")
        try:
            scan_data = []
            for scan in self.lidar.iter_scans():
                scan_data.extend(scan)
                if len(scan_data) > 360:  # One full rotation
                    break
                    
            # Process LIDAR data to identify obstacles and boundaries
            for quality, angle, distance in scan_data:
                x = distance * math.cos(math.radians(angle))
                y = distance * math.sin(math.radians(angle))
                
                # Check if point is within arena bounds
                if 0 <= x <= self.arena_size[0] and 0 <= y <= self.arena_size[1]:
                    if distance < 30:  # Points closer than 30cm are obstacles
                        self.map_data["non_traversable_points"].append((x, y))
                    else:
                        self.map_data["traversable_points"].append((x, y))
                        
        except Exception as e:
            print(f"LIDAR scanning error: {e}")
            
        return self.map_data

class Movement:
    def __init__(self, motor_kit, encoders):
        self.motor_kit = motor_kit
        self.encoders = encoders
        self.max_speed = 0.8  # 80% of max speed
        
    def set_motor_speeds(self, left_speed: float, right_speed: float):
        """Set motor speeds with safety limits"""
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
        
        self.motor_kit.motor1.throttle = left_speed
        self.motor_kit.motor2.throttle = right_speed
        
    def move_to(self, target_position: Tuple[float, float], current_position: Tuple[float, float]):
        """Move to a specific position with encoder feedback"""
        # Calculate required rotation and distance
        dx = target_position[0] - current_position[0]
        dy = target_position[1] - current_position[1]
        distance = math.sqrt(dx*dx + dy*dy)
        angle = math.atan2(dy, dx)
        
        # First rotate to face target
        self.rotate_to_angle(angle)
        
        # Then move forward
        self.move_distance(distance)

class Rover:
    def __init__(self):
        self.system = System()
        self.terrain_mapping = TerrainMapping(self.system.lidar)
        self.movement = Movement(self.system.motor_kit, self.system.encoder_pins)
        self.current_position = (0, 0)
        self.destination_found = False
        
    def detect_destination(self, image):
        """
        Implement computer vision to detect red bucket
        This would need to be implemented with OpenCV
        """
        pass
        
    def operate(self):
        try:
            # 1. Initial scan of arena
            self.map_data = self.terrain_mapping.scan_arena()
            
            # 2. Continuous operation loop
            while not self.destination_found:
                # Update map with new LIDAR data
                new_scan = self.terrain_mapping.scan_arena()
                self.map_data = self.update_map(self.map_data, new_scan)
                
                # Plan path to unexplored areas
                next_position = self.plan_exploration_path()
                
                # Move to next position
                self.movement.move_to(next_position, self.current_position)
                self.current_position = next_position
                
                # Check for destination
                self.destination_found = self.detect_destination()
                
                # Emergency checks
                if not self.system.emergency_check():
                    print("Emergency stop triggered")
                    break
                    
        except Exception as e:
            print(f"Operation error: {e}")
            self.system.mode = "ERROR"
            
        finally:
            # Cleanup
            self.system.lidar.stop()
            self.system.lidar.disconnect()
            GPIO.cleanup()
# export default Rover
# import time
# import random
# from d_star_lite import DStarLite  # Import DStarLite for path planning
# from typing import Tuple

# class System:
#     def __init__(self):
#         self.sensors = {"lidar": False, "camera": False, "motor": False, "communication": False}
#         self.mode = "IDLE"

#     def start_system(self):
#         """
#         Initialize and check all sensors.
#         """
#         print("Starting and checking sensors...")
#         for sensor in self.sensors:
#             self.sensors[sensor] = True  # Simulate sensor initialization
#             print(f"{sensor} initialized.")
#         self.mode = "READY"
#         print("System is ready!")

#     def emergency_check(self):
#         """
#         Recheck system functionality in case of an emergency.
#         """
#         print("Performing emergency checks...")
#         for sensor, status in self.sensors.items():
#             if not status:
#                 print(f"{sensor} not functioning properly.")
#                 return False
#         print("All systems operational.")
#         return True


# class Telemetry:
#     def __init__(self):
#         self.data = {"mode": None, "sensor_data": {}, "map": None}

#     def transmit_data(self):
#         """
#         Simulate transmitting data to the base station.
#         """
#         print(f"Transmitting telemetry data: {self.data}")

#     def update_telemetry(self, mode, sensor_data=None, map_data=None):
#         self.data["mode"] = mode
#         if sensor_data:
#             self.data["sensor_data"].update(sensor_data)
#         if map_data:
#             self.data["map"] = map_data


# class Movement:
#     def move_forward(self):
#         print("Moving forward.")

#     def move_backward(self):
#         print("Moving backward.")

#     def turn_left(self):
#         print("Turning left.")

#     def turn_right(self):
#         print("Turning right.")

#     def move_to(self, position: Tuple[int, int]):
#         """
#         Move to a specific position.
#         """
#         print(f"Moving to position: {position}")


# class TerrainMapping:
#     def __init__(self):
#         self.map_data = {}

#     def scan_arena(self):
#         """
#         Simulate scanning the arena with LIDAR and creating a 3D map.
#         """
#         print("Scanning terrain...")
#         self.map_data = {
#             "traversable_points": [(x, y) for x in range(10) for y in range(10) if (x + y) % 2 == 0],  # Example criteria
#             "non_traversable_points": [(random.randint(0, 10), random.randint(0, 10)) for _ in range(5)],
#         }
#         print("Arena scanned.")
#         return self.map_data


# class ObjectDetection:
#     def detect_obstacles(self, map_data):
#         """
#         Simulate detecting pot holes or obstacles and updating the map.
#         """
#         print("Detecting obstacles...")
#         map_data["non_traversable_points"].extend([(random.randint(0, 10), random.randint(0, 10)) for _ in range(3)])
#         print("Obstacles detected.")
#         return map_data

#     def detect_source_point(self):
#         """
#         Detect the source point (e.g., a can).
#         """
#         print("Detecting source point...")
#         source_point = (random.randint(0, 10), random.randint(0, 10))
#         print(f"Source point detected at {source_point}.")
#         return source_point

#     def detect_destination_point(self):
#         """
#         Detect the destination point (e.g., a red bucket).
#         """
#         print("Detecting destination point...")
#         destination_point = (random.randint(0, 10), random.randint(0, 10))
#         print(f"Destination point detected at {destination_point}.")
#         return destination_point


# class PathPlanning:
#     def find_path(self, start, end, map_data):
#         """
#         Simulate finding a path using A* algorithm.
#         """
#         print(f"Finding path from {start} to {end}...")
#         path = [start]
#         current = start
#         while current != end:
#             # Dummy logic to simulate path planning
#             current = (current[0] + 1, current[1] + 1)
#             if current in map_data["non_traversable_points"]:
#                 print(f"Encountered obstacle at {current}.")
#                 break
#             path.append(current)
#         print(f"Path found: {path}")
#         return path


# class Rover:
#     def __init__(self):
#         self.system = System()
#         self.telemetry = Telemetry()
#         self.movement = Movement()
#         self.terrain_mapping = TerrainMapping()
#         self.object_detection = ObjectDetection()
#         self.path_planning = PathPlanning()
        
#         # Initialize DStarLite variables
#         self.dstar = None
#         self.start_position = (0, 0)  # Starting position of the rover
#         self.goal_position = (5, 5)    # Example goal position
#         self.map_data = None            # To hold the map data for D* Lite

#     def start(self):
#         self.system.start_system()

#     def operate(self):
#         # Example operation flow
#         self.map_data = self.terrain_mapping.scan_arena()
#         self.map_data = self.object_detection.detect_obstacles(self.map_data)
        
#         # Initialize DStarLite with the occupancy grid map
#         self.dstar = DStarLite(self.map_data, self.start_position, self.goal_position)

#         # Find path to the goal
#         path = self.dstar.move_and_replan(self.start_position)[0]  # Get the smoothed path
        
#         # Move along the path
#         for position in path:
#             self.movement.move_to(position)  # Move to each position in the path

#         self.telemetry.update_telemetry("ACTIVE", map_data=self.map_data)
#         self.telemetry.transmit_data()

#         print("Operation complete.")

# # Main logic
# if __name__ == "__main__":
#     rover = Rover()
#     rover.start()
#     rover.operate()