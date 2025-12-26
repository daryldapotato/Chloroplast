import socket
import time
import threading
from enum import Enum

class DroneType(Enum):
    STANDARD = 1
    ADVANCED = 10

class DroneTester:
    def __init__(self, drone_ip="192.168.1.1", command_port=7099):
        self.drone_ip = drone_ip
        self.command_port = command_port
        self.socket = None
        self.running = False
        self.heartbeat_thread = None
        
        # Flight control parameters (from FlyController)
        self.control_accelerator = 128  # Default center value
        self.control_byte1 = 128
        self.control_byte2 = 128  
        self.control_turn = 128
        
        # Mode flags - using FastFly for takeoff and FastDrop for landing
        self.is_fast_fly = False      # Used for takeoff
        self.is_fast_drop = False     # Used for landing  
        self.is_emergency_stop = False
        self.is_gyro_correction = False
        self.is_no_head_mode = False
        self.is_fast_return = False
        self.is_unlock = False
        self.is_circle_turn_end = False
        
    def connect(self):
        """Initialize UDP socket connection"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(2.0)
            print(f"Connected to drone at {self.drone_ip}:{self.command_port}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def send_command(self, data: bytes):
        """Send raw command to drone"""
        if self.socket:
            try:
                self.socket.sendto(data, (self.drone_ip, self.command_port))
                print(f"Sent command: {data.hex()}")
            except Exception as e:
                print(f"Send error: {e}")
    
    def start_heartbeat(self):
        """Start periodic heartbeat (required for drone communication)"""
        self.running = True
        def heartbeat_loop():
            while self.running:
                self.send_heartbeat()
                time.sleep(1)  # Send heartbeat every second
        self.heartbeat_thread = threading.Thread(target=heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()
        print("Heartbeat started")
    
    def stop_heartbeat(self):
        """Stop heartbeat thread"""
        self.running = False
        if self.heartbeat_thread:
            self.heartbeat_thread.join(timeout=2.0)
        print("Heartbeat stopped")
    
    def send_heartbeat(self):
        """Send heartbeat command"""
        heartbeat_cmd = bytes([1, 1])
        self.send_command(heartbeat_cmd)
    
    def build_flight_command(self, device_type=DroneType.ADVANCED.value):
        """Build flight control command based on current state"""
        # Calculate control byte 5 (mode flags)
        byte5 = 0
        if self.is_fast_fly:
            byte5 += 1      # Bit 0: Fast Fly (takeoff)
        if self.is_fast_drop:
            byte5 += 2      # Bit 1: Fast Drop (landing)
        if self.is_emergency_stop:
            byte5 += 4      # Bit 2: Emergency Stop
        if self.is_circle_turn_end:
            byte5 += 8      # Bit 3: Circle Turn End
        if self.is_no_head_mode:
            byte5 += 16     # Bit 4: No Head Mode
        if self.is_fast_return or self.is_unlock:
            byte5 += 32     # Bit 5: Fast Return / Unlock
        if self.is_gyro_correction:
            byte5 += 128    # Bit 7: Gyro Correction
        
        # Clamp control values to valid range (1-255)
        self.control_turn = max(1, min(255, self.control_turn))
        self.control_byte1 = max(1, min(255, self.control_byte1))
        self.control_byte2 = max(1, min(255, self.control_byte2))
        
        if self.control_accelerator == 1:
            self.control_accelerator = 0
        
        # Calculate checksum
        checksum = (self.control_byte1 ^ self.control_byte2 ^ 
                   self.control_accelerator ^ self.control_turn ^ (byte5 & 255))
        
        if device_type == DroneType.ADVANCED.value:
            # Advanced drone protocol (8-byte command)
            command_data = bytes([
                102,  # Byte 0: Fixed value
                self.control_byte1 & 0xFF,
                self.control_byte2 & 0xFF, 
                self.control_accelerator & 0xFF,
                self.control_turn & 0xFF,
                byte5 & 0xFF,
                checksum & 0xFF,
                153   # Byte 7: Fixed value
            ])
            # Prefix with command type 3 for flight control
            full_command = bytes([3]) + command_data
        else:
            # Standard drone protocol (20-byte command)
            byte5_std = 0
            if self.is_fast_fly or self.is_fast_drop:
                byte5_std += 1
            if self.is_emergency_stop:
                byte5_std += 2
            if self.is_gyro_correction:
                byte5_std += 4
            if self.is_circle_turn_end:
                byte5_std += 8
            
            byte6_std = 0
            if self.is_no_head_mode:
                byte6_std += 1
            
            checksum_std = (byte5_std ^ 
                           (self.control_byte2 ^ self.control_byte1 ^ 
                            self.control_accelerator ^ self.control_turn) ^ 
                           (byte6_std & 255))
            
            command_data = bytes([
                102, 20,  # Byte 0-1: Fixed values
                self.control_byte1 & 0xFF,
                self.control_byte2 & 0xFF,
                self.control_accelerator & 0xFF, 
                self.control_turn & 0xFF,
                byte5_std & 0xFF,
                byte6_std & 0xFF,
                # Padding bytes
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                checksum_std & 0xFF,
                153  # Byte 19: Fixed value
            ])
            full_command = bytes([3]) + command_data
        
        return full_command
    
    def send_flight_command(self):
        """Send current flight control command"""
        command = self.build_flight_command()
        self.send_command(command)
    
    def takeoff_fast_fly(self):
        """Takeoff using isFastFly flag"""
        print("Initiating takeoff using FastFly...")
        
        # Set FastFly flag for takeoff
        self.is_fast_fly = True
        self.send_flight_command()
        print("FastFly takeoff command sent")
        
        # Keep sending command for a short duration
        start_time = time.time()
        while time.time() - start_time < 1.0:  # Send for 1 second
            self.send_flight_command()
            time.sleep(0.05)
        
        # Reset FastFly flag
        self.is_fast_fly = False
        self.send_flight_command()
        print("FastFly reset - drone should be airborne")
    
    def land_fast_drop(self):
        """Land using isFastDrop flag"""
        print("Initiating landing using FastDrop...")
        
        # Set FastDrop flag for landing
        self.is_fast_drop = True
        self.send_flight_command()
        print("FastDrop landing command sent")
        
        # Keep sending command for a short duration
        start_time = time.time()
        while time.time() - start_time < 1.5:  # Send for 1.5 seconds (longer for landing)
            self.send_flight_command()
            time.sleep(0.05)
        
        # Reset FastDrop flag
        self.is_fast_drop = False
        self.send_flight_command()
        print("FastDrop reset - drone should be landed")
    
    def hover(self):
        """Set hover mode (neutral controls, no special flags)"""
        print("Setting hover mode...")
        # Reset all special flags for stable hover
        self.is_fast_fly = False
        self.is_fast_drop = False
        self.is_emergency_stop = False
        
        # Set neutral control values
        self.control_accelerator = 128  # Center value
        self.control_turn = 128
        self.control_byte1 = 128
        self.control_byte2 = 128
        
        self.send_flight_command()
        print("Hover command sent")
    
    def emergency_stop(self):
        """Send emergency stop command using isEmergencyStop flag"""
        print("EMERGENCY STOP!")
        self.is_emergency_stop = True
        self.send_flight_command()
        
        # Keep emergency stop active for 1 second
        start_time = time.time()
        while time.time() - start_time < 1.0:
            self.send_flight_command()
            time.sleep(0.05)
        
        self.is_emergency_stop = False
        self.send_flight_command()
        print("Emergency stop reset")
    
    def test_fastfly_fastdrop_sequence(self, flight_duration=5):
        """
        Complete test sequence using FastFly for takeoff and FastDrop for landing:
        1. Connect to drone
        2. Start heartbeat
        3. Takeoff using FastFly
        4. Hover for specified duration
        5. Land using FastDrop
        """
        print("=== DRONE FASTFLY/FASTDROP TEST SEQUENCE ===")
        print("Using isFastFly for takeoff and isFastDrop for landing")
        
        # Step 1: Connect
        if not self.connect():
            print("Failed to connect to drone. Aborting test.")
            return False
        
        try:
            # Step 2: Start heartbeat (required for drone to accept commands)
            self.start_heartbeat()
            time.sleep(2)  # Let heartbeat establish
            
            # Step 3: Takeoff using FastFly
            self.takeoff_fast_fly()
            time.sleep(1)  # Allow time for takeoff to complete
            
            # Step 4: Hover at altitude
            self.hover()
            print(f"Hovering for {flight_duration} seconds...")
            
            # Send hover commands periodically during flight
            hover_start = time.time()
            while time.time() - hover_start < flight_duration:
                self.send_flight_command()
                time.sleep(0.1)  # Send commands every 100ms
            
            # Step 5: Land using FastDrop
            self.land_fast_drop()
            time.sleep(1)  # Allow time for landing to complete
            
            # Final hover to ensure stable state
            self.hover()
            
            print("Test sequence completed successfully!")
            return True
            
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
            self.emergency_stop()
            return False
        except Exception as e:
            print(f"Test failed with error: {e}")
            self.emergency_stop()
            return False
        finally:
            # Cleanup
            self.stop_heartbeat()
            if self.socket:
                self.socket.close()
            print("Connection closed")

    def test_individual_fast_commands(self):
        """Test individual FastFly and FastDrop commands for debugging"""
        print("Testing individual FastFly/FastDrop commands...")
        
        if not self.connect():
            return False
        
        try:
            self.start_heartbeat()
            time.sleep(2)
            
            print("Testing FastFly takeoff...")
            self.takeoff_fast_fly()
            time.sleep(3)
            
            print("Testing hover...")
            self.hover()
            time.sleep(2)
            
            print("Testing FastDrop landing...")
            self.land_fast_drop()
            time.sleep(2)
            
            return True
            
        finally:
            self.stop_heartbeat()
            if self.socket:
                self.socket.close()

    def test_quick_tap_commands(self):
        """Test quick tap-style commands (like the original Java implementation)"""
        print("Testing quick-tap FastFly/FastDrop commands...")
        
        if not self.connect():
            return False
        
        try:
            self.start_heartbeat()
            time.sleep(2)
            
            # Quick FastFly tap (like button press)
            print("Quick FastFly tap...")
            self.is_fast_fly = True
            self.send_flight_command()
            time.sleep(0.5)  # Short duration like a button tap
            self.is_fast_fly = False
            self.send_flight_command()
            
            time.sleep(3)  # Hover
            
            # Quick FastDrop tap (like button press)
            print("Quick FastDrop tap...")
            self.is_fast_drop = True
            self.send_flight_command()
            time.sleep(0.5)  # Short duration like a button tap
            self.is_fast_drop = False
            self.send_flight_command()
            
            return True
            
        finally:
            self.stop_heartbeat()
            if self.socket:
                self.socket.close()

def main():
    """Main test function"""
    tester = DroneTester()
    
    print("=== DRONE FASTFLY/FASTDROP TESTER ===")
    print("Using isFastFly flag for takeoff and isFastDrop for landing")
    print()
    print("Choose test mode:")
    print("1. Complete FastFly/FastDrop sequence (5 second hover)")
    print("2. Individual FastFly/FastDrop command test") 
    print("3. Quick-tap command test (button press simulation)")
    print("4. Emergency stop test")
    
    choice = input("Enter choice (1-4): ").strip()
    
    if choice == "1":
        # Complete sequence test
        success = tester.test_fastfly_fastdrop_sequence(flight_duration=5)
        if success:
            print("✅ FastFly/FastDrop test completed successfully!")
        else:
            print("❌ FastFly/FastDrop test failed!")
    
    elif choice == "2":
        # Individual command test
        success = tester.test_individual_fast_commands()
        if success:
            print("✅ Individual FastFly/FastDrop test completed!")
        else:
            print("❌ Individual FastFly/FastDrop test failed!")
    
    elif choice == "3":
        # Quick tap test
        success = tester.test_quick_tap_commands()
        if success:
            print("✅ Quick-tap test completed!")
        else:
            print("❌ Quick-tap test failed!")
    
    elif choice == "4":
        # Emergency stop test
        if tester.connect():
            tester.start_heartbeat()
            time.sleep(2)
            tester.emergency_stop()
            tester.stop_heartbeat()
            print("✅ Emergency stop test completed!")
        else:
            print("❌ Failed to connect for emergency stop test!")
    
    else:
        print("Invalid choice!")

if __name__ == "__main__":
    main()
