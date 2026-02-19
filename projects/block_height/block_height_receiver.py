"""
Author: Swaraj Dangare
"""
import asyncio
import statistics
import math
from bleak import BleakClient, BleakScanner
import numpy as np
from scipy.optimize import least_squares

# BLE Configuration

ESP_NAME = "BlockOffsetEncoder"
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

# ANSI Color Codes
class Colors:
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    MAGENTA = '\033[95m'
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    RESET = '\033[0m'

class HighPrecisionReceiver:
    def __init__(self):
        
        self.latest_angle = None
        self.latest_diagnostics = {}  # Store AGC, MAG, MAGL, MAGH, COF
        self.device = None
        self.message_received = asyncio.Event()
        self.starting_angle = 56.809 #59.2 56.809
        # Block height measurement configuration
        self.link_length = 40.0  # mm
        self.show_diagnostics = False  # Toggle for diagnostic display

    def notification_handler(self, sender, data):
        msg = data.decode()
        try:
            # New Format: "E0:66.4340,AGC:128,MAG:5432,MAGL:0,MAGH:0,COF:0"
            # Parse all fields
            parts = msg.split(',')
            
            # Extract angle from first part
            if ':' in parts[0]:
                _, angle_str = parts[0].split(':')
                self.latest_angle = float(angle_str)
            
            # Extract diagnostic data
            self.latest_diagnostics = {}
            for part in parts[1:]:
                if ':' in part:
                    key, value = part.split(':')
                    self.latest_diagnostics[key] = int(value)
            
            self.message_received.set()
        except Exception as e:
            print(f" [!] Parse Error: {e}")

    async def get_single_reading(self, client):
        self.message_received.clear()
        await client.write_gatt_char(CHAR_UUID, b"X")
        try:
            # The Ultra-Precision firmware takes ~160ms to sample 4096 times (16 blocks)
            await asyncio.wait_for(self.message_received.wait(), timeout=5.0)
            return self.latest_angle
        except asyncio.TimeoutError:
            return None
    
    async def zero_encoder(self, client):
        """Send 'Z' command to reset encoder to zero"""
        await client.write_gatt_char(CHAR_UUID, b"Z")
        await asyncio.sleep(0.1)  # Give firmware time to process
        print(f"{Colors.MAGENTA} [INFO] Zero reset command sent to encoder{Colors.RESET}")
    
    def set_link_length(self):
        """Allow user to configure link length"""
        try:
            new_length = float(input(f"Enter link length in mm (current: {self.link_length}mm): "))
            if new_length > 0:
                self.link_length = new_length
                print(f"{Colors.MAGENTA} [INFO] Link length set to {self.link_length}mm{Colors.RESET}")
            else:
                print(f"{Colors.RED} [ERROR] Link length must be positive{Colors.RESET}")
        except ValueError:
            print(f"{Colors.RED} [ERROR] Invalid input{Colors.RESET}")
    
    def calculate_height(self, angle_deg):
        """Calculate vertical height from angle using trigonometry"""
        # Height = link_length × sin(angle)
        angle_rad = math.radians(self.starting_angle+angle_deg)
        height_mm = (self.link_length * math.cos(math.radians(self.starting_angle)))-(self.link_length * math.cos(angle_rad))
        return height_mm
    
    async def calibrate_starting_angle(self, client):
        """Calibration mode: collect 10 measurements and back-calculate starting_angle"""
        print("=" * 60)
        print(" STARTING ANGLE CALIBRATION MODE")
        print("=" * 60)
        print(f"\n{Colors.YELLOW}Instructions:{Colors.RESET}")
        print("1. Position the link at a known height")
        print("2. Press Enter to measure the angle")
        print("3. Enter the actual height in mm")
        print("4. Repeat for 10 different heights\n")
        
        calibration_data = []  # Store (angle, actual_height) pairs
        
        for i in range(10):
            input(f"\n{Colors.GREEN}[{i+1}/10] Press Enter to measure...{Colors.RESET}")
            
            # Get angle measurement
            results = []
            for _ in range(5):
                val = await self.get_single_reading(client)
                if val is not None:
                    results.append(val)
                await asyncio.sleep(0.1)
            
            if len(results) >= 3:
                # Apply inversion and unwrapping
                inverted_results = [360 - r for r in results]
                has_low = any(r < 10 for r in inverted_results)
                has_high = any(r > 350 for r in inverted_results)
                
                if has_low and has_high:
                    unwrapped = [r if r < 180 else r - 360 for r in inverted_results]
                    angle = statistics.median(unwrapped)
                    if angle < 0:
                        angle += 360
                else:
                    angle = statistics.median(inverted_results)
                
                print(f"  Measured angle: {Colors.CYAN}{angle:.5f}°{Colors.RESET}")
                
                # Get actual height from user
                while True:
                    try:
                        actual_height = float(input(f"  Enter actual height (mm): "))
                        break
                    except ValueError:
                        print(f"  {Colors.RED}Invalid input, try again{Colors.RESET}")
                
                calibration_data.append((angle, actual_height))
                print(f"  {Colors.GREEN}✓ Point {i+1} recorded{Colors.RESET}")
            else:
                print(f"  {Colors.RED}Failed to get reading, try again{Colors.RESET}")
                return
        
        # Back-calculate optimal starting_angle using least-squares
        print(f"\n{Colors.YELLOW}Calculating optimal starting angle...{Colors.RESET}")
        
        # Store original starting angle (don't modify during calibration)
        original_starting_angle = self.starting_angle
        
        def residuals(starting_angle_guess):
            """Calculate residuals (errors) for least_squares optimization"""
            errors = []
            for angle_deg, actual_height in calibration_data:
                # Calculate predicted height
                angle_rad = math.radians(starting_angle_guess[0] + angle_deg)
                predicted_height = (self.link_length * math.cos(math.radians(starting_angle_guess[0]))) - \
                                 (self.link_length * math.cos(angle_rad))
                error = predicted_height - actual_height
                errors.append(error)
            return errors
        
        # Optimize starting_angle using least_squares (Levenberg-Marquardt)
        # This is much faster and more accurate than Nelder-Mead for this problem
        result = least_squares(residuals, [original_starting_angle], method='lm')
        optimal_angle = result.x[0]
        
        print(f"\n{Colors.BOLD}{Colors.GREEN}=" * 60)
        print(" CALIBRATION RESULTS")
        print("=" * 60 + Colors.RESET)
        print(f"\nOld starting angle: {Colors.YELLOW}{original_starting_angle:.3f}°{Colors.RESET}")
        print(f"New starting angle: {Colors.GREEN}{Colors.BOLD}{optimal_angle:.3f}°{Colors.RESET}")
        
        # Show prediction errors
        print(f"\n{Colors.CYAN}Verification:{Colors.RESET}")
        for i, (angle_deg, actual_height) in enumerate(calibration_data, 1):
            angle_rad = math.radians(optimal_angle + angle_deg)
            predicted_height = (self.link_length * math.cos(math.radians(optimal_angle))) - \
                             (self.link_length * math.cos(angle_rad))
            error = predicted_height - actual_height
            print(f"  Point {i}: Actual={actual_height:.2f}mm, Predicted={predicted_height:.2f}mm, Error={error:+.2f}mm")
        
        # Ask user to apply
        apply = input(f"\n{Colors.YELLOW}Apply this calibration? (y/n): {Colors.RESET}").strip().lower()
        if apply == 'y':
            self.starting_angle = optimal_angle
            print(f"{Colors.GREEN}✓ Starting angle updated to {optimal_angle:.3f}°{Colors.RESET}")
            print(f"{Colors.MAGENTA}Note: This value is not saved permanently. Update it in the code if needed.{Colors.RESET}")
        else:
            print(f"{Colors.YELLOW}Calibration discarded{Colors.RESET}")

    async def run(self):
        print(f"Scanning for {ESP_NAME}...")
        self.device = await BleakScanner.find_device_by_name(ESP_NAME)
        
        if not self.device:
            print(f"Error: Could not find device named '{ESP_NAME}'")
            return

        print(f"Found {self.device.name} ({self.device.address}). Connecting...")
        
        # Retry mechanism for connection
        max_retries = 3
        client = None
        
        for attempt in range(max_retries):
            try:
                client = BleakClient(self.device, timeout=10.0)
                await client.connect()
                if client.is_connected:
                    break
            except Exception as e:
                print(f"Connection attempt {attempt+1}/{max_retries} failed: {e}")
                if attempt < max_retries - 1:
                    await asyncio.sleep(2.0)
                else:
                    print("Failed to connect after multiple retries.")
                    return

        # Proceed if connected
        try:
            print(f"Connected: {client.is_connected}")
            await client.start_notify(CHAR_UUID, self.notification_handler)
            
            print("\n" + "="*60)
            print(f"{Colors.BOLD}{Colors.CYAN} BLOCK HEIGHT MEASUREMENT SYSTEM{Colors.RESET}")
            print(f" Link Length: {Colors.YELLOW}{self.link_length}mm{Colors.RESET}")
            print(" Logic: Median of 5 blocks (4096-sample filtered each)")
            print("="*60)
            print(f"\n{Colors.BOLD}Commands:{Colors.RESET}")
            print(f"  {Colors.GREEN}Enter{Colors.RESET} - Measure angle and height")
            print(f"  {Colors.YELLOW}Z{Colors.RESET}     - Zero at ground reference")
            print(f"  {Colors.CYAN}D{Colors.RESET}     - Toggle diagnostic display")
            print(f"  {Colors.MAGENTA}L{Colors.RESET}     - Set link length")
            print(f"  {Colors.BLUE}C{Colors.RESET}     - Calibrate starting angle")
            print(f"  {Colors.RED}Ctrl+C{Colors.RESET} - Quit\n")

            try:
                while True:
                    user_input = input("\n>> Command [Enter/Z/D/L/C]: ").strip().upper()
                    
                    if user_input == 'Z':
                        await self.zero_encoder(client)
                        continue
                    elif user_input == 'D':
                        self.show_diagnostics = not self.show_diagnostics
                        status = "ON" if self.show_diagnostics else "OFF"
                        print(f"{Colors.MAGENTA} [INFO] Diagnostic display: {status}{Colors.RESET}")
                        continue
                    elif user_input == 'L':
                        self.set_link_length()
                        continue
                    elif user_input == 'C':
                        await self.calibrate_starting_angle(client)
                        continue
                    
                    # print("Sampling 5 sets of data from ESP32...", end="", flush=True)
                    
                    results = []
                    for i in range(5):
                        val = await self.get_single_reading(client)
                        if val is not None:
                            results.append(val)
                            print(".", end="", flush=True)
                        await asyncio.sleep(0.1) # Brief gap between requests
                    
                    if len(results) >= 3:
                        # Apply 360° inversion first
                        inverted_results = [360 - r for r in results]
                        
                        # Detect angle wrap-around at 0°/360° boundary
                        # If we have values both near 0° and near 360°, unwrap them
                        has_low = any(r < 10 for r in inverted_results)
                        has_high = any(r > 350 for r in inverted_results)
                        
                        if has_low and has_high:
                            # Unwrap: convert 350-360° to negative equivalent (-10 to 0°)
                            unwrapped = [r if r < 180 else r - 360 for r in inverted_results]
                            final_median = statistics.median(unwrapped)
                            range_val = max(unwrapped) - min(unwrapped)
                            
                            # Convert back to positive 0-360° range if needed
                            if final_median < 0:
                                final_median += 360
                        else:
                            # No wrap-around, use normal calculation
                            final_median = statistics.median(inverted_results)
                            range_val = max(inverted_results) - min(inverted_results)
                        
                        # Calculate vertical height from angle
                        height_mm = self.calculate_height(final_median)
                        
                        # Calculate physical precision for link
                        # angle_deg * (pi/180) * link_length
                        precision_microns = self.link_length * (range_val * 3.14159 / 180.0) * 1000

                        print(f"\n" + "-"*60)
                        print(f"{Colors.CYAN}{Colors.BOLD} ANGLE          : {final_median:.5f} degrees{Colors.RESET}")
                        print(f"{Colors.GREEN}{Colors.BOLD} VERTICAL HEIGHT: {height_mm:.3f} mm{Colors.RESET}")
                        # print(f" RAW JITTER     : {range_val:.5f} deg ({precision_microns:.1f} microns)")
                        # print(f" ALL READINGS   : {[round(r, 5) for r in results]}")
                        
                        # Display diagnostic data only if enabled
                        if self.show_diagnostics and self.latest_diagnostics:
                            print(f"\n{Colors.YELLOW} DIAGNOSTIC DATA:{Colors.RESET}")
                            print(f"   RAW JITTER   : {range_val:.5f} deg ({precision_microns:.1f} microns)")
                            print(f"   ALL READINGS : {[round(r, 5) for r in results]}")
                            print(f"   AGC (Gain)   : {self.latest_diagnostics.get('AGC', 'N/A')}")
                            print(f"   MAG (Magn.)  : {self.latest_diagnostics.get('MAG', 'N/A')}")
                            print(f"   MAGL (High)  : {self.latest_diagnostics.get('MAGL', 'N/A')}")
                            print(f"   MAGH (Low)   : {self.latest_diagnostics.get('MAGH', 'N/A')}")
                            print(f"   COF (Ovflow) : {self.latest_diagnostics.get('COF', 'N/A')}")
                        
                        print("-"*60)
                        
                        # Check for diagnostic warnings
                        if self.latest_diagnostics.get('MAGL', 0) == 1:
                            print(f"{Colors.RED} [WARNING] Magnetic field too HIGH (AGC=0x00)!{Colors.RESET}")
                            print(f"{Colors.RED} Action: Move magnet further from sensor.{Colors.RESET}")
                        elif self.latest_diagnostics.get('MAGH', 0) == 1:
                            print(f"{Colors.RED} [WARNING] Magnetic field too LOW (AGC=0xFF)!{Colors.RESET}")
                            print(f"{Colors.RED} Action: Move magnet closer to sensor.{Colors.RESET}")
                        elif self.latest_diagnostics.get('COF', 0) == 1:
                            print(f"{Colors.RED} [WARNING] CORDIC overflow detected!{Colors.RESET}")
                            print(f"{Colors.RED} Action: Check sensor alignment and magnetic field.{Colors.RESET}")
                        elif precision_microns > 50:
                            print(f"{Colors.YELLOW} [WARNING] Jitter is {precision_microns:.1f}u, which exceeds 50u limit.{Colors.RESET}")
                            print(f"{Colors.YELLOW} Check for vibrations or move the arm slightly.{Colors.RESET}")
                        else:
                            print(f"{Colors.GREEN} Read SUCCESS{Colors.RESET}")
                    else:
                        print("\n[ERROR] Failed to collect enough samples from BLE.")
                        
            except KeyboardInterrupt:
                print("\nDisconnecting...")
            finally:
                if client and client.is_connected:
                    await client.stop_notify(CHAR_UUID)
                    await client.disconnect()

        except Exception as e:
            print(f" [!] Unexpected Error: {e}")

if __name__ == "__main__":
    receiver = HighPrecisionReceiver()
    try:
        asyncio.run(receiver.run())
    except KeyboardInterrupt:
        pass
