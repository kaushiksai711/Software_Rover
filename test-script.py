import time
from mock_hardware import *
from Software_rover import System, TerrainMapping, Movement, Rover

def test_system_initialization():
    print("\nTesting system initialization...")
    system = System()
    system.start_system()
    assert system.mode == "READY", f"System mode should be READY, got {system.mode}"
    print("System initialization test passed!")

def test_terrain_mapping():
    print("\nTesting terrain mapping...")
    lidar = MockLidar()
    terrain_mapper = TerrainMapping(lidar)
    map_data = terrain_mapper.scan_arena()
    
    assert "traversable_points" in map_data, "Map should contain traversable points"
    assert "non_traversable_points" in map_data, "Map should contain non-traversable points"
    print("Terrain mapping test passed!")
    print(f"Found {len(map_data['traversable_points'])} traversable points")
    print(f"Found {len(map_data['non_traversable_points'])} non-traversable points")

def test_movement():
    print("\nTesting movement system...")
    motor_kit = MockMotorKit()
    encoders = {'left': 17, 'right': 18}
    movement = Movement(motor_kit, encoders)
    
    # Test different motor speeds
    print("Testing motor speeds...")
    test_speeds = [
        (0.5, 0.5),   # Forward
        (-0.5, -0.5), # Backward
        (0.5, -0.5),  # Spin right
        (-0.5, 0.5)   # Spin left
    ]
    
    for left, right in test_speeds:
        movement.set_motor_speeds(left, right)
        actual_left = motor_kit.motor1.throttle
        actual_right = motor_kit.motor2.throttle
        assert abs(actual_left - left) < 0.01, f"Left motor speed should be {left}, got {actual_left}"
        assert abs(actual_right - right) < 0.01, f"Right motor speed should be {right}, got {actual_right}"
    print("Movement test passed!")

def test_full_rover_operation():
    print("\nTesting full rover operation...")
    rover = Rover()
    
    # Test initialization
    assert rover.system.mode == "READY", "Rover should be in READY state"
    
    # Test basic operation cycle
    print("Running rover operation cycle...")
    try:
        # Run for a few seconds
        start_time = time.time()
        while time.time() - start_time < 5:  # Run for 5 seconds
            map_data = rover.terrain_mapping.scan_arena()
            print(f"Current position: {rover.current_position}")
            time.sleep(1)
        print("Rover operation test passed!")
    except Exception as e:
        print(f"Rover operation test failed: {e}")
        raise

def main():
    print("Starting rover system tests...")
    try:
        test_system_initialization()
        test_terrain_mapping()
        test_movement()
        test_full_rover_operation()
        print("\nAll tests completed successfully!")
    except AssertionError as e:
        print(f"\nTest failed: {e}")
    except Exception as e:
        print(f"\nUnexpected error: {e}")

if __name__ == "__main__":
    main()
