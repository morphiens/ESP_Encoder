"""
Author: Swaraj Dangare
"""
import asyncio
import statistics
import math
from bleak import BleakClient, BleakScanner
import numpy as np

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

class PolynomialHeightReceiver:
    def __init__(self):
        
        self.latest_angle = None
        self.latest_diagnostics = {}  # Store AGC, MAG, MAGL, MAGH, COF
        self.device = None
        self.message_received = asyncio.Event()
        
        # Polynomial coefficients (will be set during calibration)
        # Default: 3rd order polynomial [a0, a1, a2, a3] where height = a0 + a1*angle + a2*angle^2 + a3*angle^3
        # Pre-calibrated coefficients (from user's calibration):
        self.poly_coeffs = np.array([-1.065580e-05, 2.174321e-03, 5.983315e-01, -3.448675e-02])
        self.poly_degree = 3  # Default to 3rd order (cubic)
        
        # Configuration
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
    
    def calculate_height(self, angle_deg):
        """Calculate height from angle using polynomial fit"""
        if self.poly_coeffs is None:
            print(f"{Colors.RED} [ERROR] No calibration data! Run calibration first (press 'C'){Colors.RESET}")
            return 0.0
        
        # Evaluate polynomial: height = a0 + a1*x + a2*x^2 + ...
        height_mm = np.polyval(self.poly_coeffs, angle_deg)
        return height_mm
    
    async def calibrate_polynomial(self, client):
        """Calibration mode: collect measurements and fit polynomial curve"""
        print("=" * 60)
        print(" POLYNOMIAL CURVE FITTING CALIBRATION")
        print("=" * 60)
        print(f"\n{Colors.YELLOW}Instructions:{Colors.RESET}")
        print("1. Position the link at a known height")
        print("2. Press Enter to measure the angle")
        print("3. Enter the actual height in mm")
        print("4. Collect at least 5 points (more is better!)")
        print("5. Type 'done' when finished\n")
        
        # Ask for polynomial degree
        while True:
            try:
                degree_input = input(f"Polynomial degree (1=linear, 2=quadratic, 3=cubic) [default=3]: ").strip()
                if degree_input == "":
                    self.poly_degree = 3
                    break
                degree = int(degree_input)
                if 1 <= degree <= 5:
                    self.poly_degree = degree
                    break
                else:
                    print(f"{Colors.RED}Please enter a degree between 1 and 5{Colors.RESET}")
            except ValueError:
                print(f"{Colors.RED}Invalid input{Colors.RESET}")
        
        print(f"\n{Colors.CYAN}Using {self.poly_degree}-degree polynomial{Colors.RESET}\n")
        
        calibration_data = []  # Store (angle, actual_height) pairs
        point_num = 1
        
        while True:
            user_input = input(f"\n{Colors.GREEN}[Point {point_num}] Press Enter to measure (or type 'done' to finish)...{Colors.RESET}").strip().lower()
            
            if user_input == 'done':
                if len(calibration_data) < self.poly_degree + 2:
                    print(f"{Colors.RED}Need at least {self.poly_degree + 2} points for {self.poly_degree}-degree polynomial!{Colors.RESET}")
                    continue
                break
            
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
                print(f"  {Colors.GREEN}✓ Point {point_num} recorded{Colors.RESET}")
                point_num += 1
            else:
                print(f"  {Colors.RED}Failed to get reading, try again{Colors.RESET}")
        
        # Fit polynomial curve
        print(f"\n{Colors.YELLOW}Fitting {self.poly_degree}-degree polynomial...{Colors.RESET}")
        
        angles = np.array([d[0] for d in calibration_data])
        heights = np.array([d[1] for d in calibration_data])
        
        # Fit polynomial using numpy polyfit
        self.poly_coeffs = np.polyfit(angles, heights, self.poly_degree)
        
        # Calculate R² (coefficient of determination)
        predicted_heights = np.polyval(self.poly_coeffs, angles)
        residuals = heights - predicted_heights
        ss_res = np.sum(residuals**2)
        ss_tot = np.sum((heights - np.mean(heights))**2)
        r_squared = 1 - (ss_res / ss_tot)
        rms_error = np.sqrt(np.mean(residuals**2))
        
        print(f"\n{Colors.BOLD}{Colors.GREEN}=" * 60)
        print(" CALIBRATION RESULTS")
        print("=" * 60 + Colors.RESET)
        print(f"\nPolynomial degree: {self.poly_degree}")
        print(f"Number of points: {len(calibration_data)}")
        print(f"R² (fit quality): {Colors.GREEN}{r_squared:.6f}{Colors.RESET} (1.0 = perfect)")
        print(f"RMS Error: {rms_error:.4f} mm")
        
        # Show coefficients
        print(f"\n{Colors.CYAN}Polynomial coefficients:{Colors.RESET}")
        for i, coeff in enumerate(self.poly_coeffs):
            power = self.poly_degree - i
            print(f"  a{power}: {coeff:.6e}")
        
        # Show equation
        equation = "height = "
        for i, coeff in enumerate(self.poly_coeffs):
            power = self.poly_degree - i
            if i > 0:
                equation += " + " if coeff >= 0 else " - "
                equation += f"{abs(coeff):.4f}"
            else:
                equation += f"{coeff:.4f}"
            
            if power > 0:
                equation += f"*angle^{power}" if power > 1 else "*angle"
        print(f"\n{Colors.YELLOW}{equation}{Colors.RESET}")
        
        # Show prediction errors
        print(f"\n{Colors.CYAN}Verification:{Colors.RESET}")
        print(f"{'Point':<8} {'Angle':<12} {'Actual':<10} {'Predicted':<12} {'Error':<10}")
        print("-" * 60)
        
        for i, (angle_deg, actual_height) in enumerate(calibration_data, 1):
            predicted_height = np.polyval(self.poly_coeffs, angle_deg)
            error = predicted_height - actual_height
            print(f"{i:<8} {angle_deg:<12.5f} {actual_height:<10.3f} {predicted_height:<12.3f} {error:+10.3f}")
        
        print(f"\n{Colors.GREEN}✓ Calibration complete! Polynomial coefficients saved.{Colors.RESET}")
        print(f"{Colors.MAGENTA}Note: Coefficients are stored in memory only. They will be lost on restart.{Colors.RESET}")

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
            print(f"{Colors.BOLD}{Colors.CYAN} POLYNOMIAL HEIGHT MEASUREMENT SYSTEM{Colors.RESET}")
            print(" Logic: Polynomial curve fitting (angle → height)")
            print("="*60)
            print(f"\n{Colors.BOLD}Commands:{Colors.RESET}")
            print(f"  {Colors.GREEN}Enter{Colors.RESET} - Measure angle and height")
            print(f"  {Colors.YELLOW}Z{Colors.RESET}     - Zero at ground reference")
            print(f"  {Colors.CYAN}D{Colors.RESET}     - Toggle diagnostic display")
            print(f"  {Colors.BLUE}C{Colors.RESET}     - Calibrate polynomial curve")
            print(f"  {Colors.RED}Ctrl+C{Colors.RESET} - Quit\n")

            try:
                while True:
                    user_input = input("\n>> Command [Enter/Z/D/C]: ").strip().upper()
                    
                    if user_input == 'Z':
                        await self.zero_encoder(client)
                        continue
                    elif user_input == 'D':
                        self.show_diagnostics = not self.show_diagnostics
                        status = "ON" if self.show_diagnostics else "OFF"
                        print(f"{Colors.MAGENTA} [INFO] Diagnostic display: {status}{Colors.RESET}")
                        continue
                    elif user_input == 'C':
                        await self.calibrate_polynomial(client)
                        continue
                    
                    results = []
                    for i in range(5):
                        val = await self.get_single_reading(client)
                        if val is not None:
                            results.append(val)
                            print(".", end="", flush=True)
                        await asyncio.sleep(0.1)
                    
                    if len(results) >= 3:
                        # Apply inversion and unwrapping
                        inverted_results = [360 - r for r in results]
                        has_low = any(r < 10 for r in inverted_results)
                        has_high = any(r > 350 for r in inverted_results)
                        
                        if has_low and has_high:
                            unwrapped = [r if r < 180 else r - 360 for r in inverted_results]
                            final_median = statistics.median(unwrapped)
                            range_val = max(unwrapped) - min(unwrapped)
                            if final_median < 0:
                                final_median += 360
                        else:
                            final_median = statistics.median(inverted_results)
                            range_val = max(inverted_results) - min(inverted_results)
                        
                        # Calculate height using polynomial
                        height_mm = self.calculate_height(final_median)
                        
                        print(f"\n" + "-"*60)
                        print(f"{Colors.CYAN}{Colors.BOLD} ANGLE          : {final_median:.5f} degrees{Colors.RESET}")
                        print(f"{Colors.GREEN}{Colors.BOLD} HEIGHT         : {height_mm:.3f} mm{Colors.RESET}")
                        
                        # Display diagnostic data only if enabled
                        if self.show_diagnostics and self.latest_diagnostics:
                            precision_microns = 40.0 * (range_val * 3.14159 / 180.0) * 1000
                            print(f"\n{Colors.YELLOW} DIAGNOSTIC DATA:{Colors.RESET}")
                            print(f"   RAW JITTER   : {range_val:.5f} deg ({precision_microns:.1f} microns)")
                            print(f"   ALL READINGS : {[round(r, 5) for r in results]}")
                            print(f"   AGC (Gain)   : {self.latest_diagnostics.get('AGC', 'N/A')}")
                            print(f"   MAG (Magn.)  : {self.latest_diagnostics.get('MAG', 'N/A')}")
                            print(f"   MAGL (High)  : {self.latest_diagnostics.get('MAGL', 'N/A')}")
                            print(f"   MAGH (Low)   : {self.latest_diagnostics.get('MAGH', 'N/A')}")
                            print(f"   COF (Ovflow) : {self.latest_diagnostics.get('COF', 'N/A')}")
                        
                        print("-"*60)
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
    receiver = PolynomialHeightReceiver()
    try:
        asyncio.run(receiver.run())
    except KeyboardInterrupt:
        pass
