import serial
import time
import csv
from collections import defaultdict

# --- Configuration ---
PORT_MOTOR = "/dev/ttyACM0"
PORT_ENCODER = "/dev/ttyACM1"
BAUD_RATE = 115200

TOTAL_CYCLES = 500
CSV_FILENAME = "motor_encoder_continuous_calibration.csv"
MOTOR_DELAY = 2  # ~1.75 seconds to settle for 30-degree moves

# Connect to Motor
try:
    ser_motor = serial.Serial(
        PORT_MOTOR, 
        BAUD_RATE, 
        timeout=1,
        dsrdtr=None,
        rtscts=False
    )
    ser_motor.dtr = False
    time.sleep(1)
    ser_motor.dtr = True
    
    print("Waiting 4 seconds for motor driver to boot...")
    time.sleep(4)
    print(f"[SUCCESS] Connected to Motor on {PORT_MOTOR}")
except Exception as e:
    print(f"[ERROR] Could not open Motor port {PORT_MOTOR}: {e}")
    exit()

# Connect to Encoder
# ... [Your connection code remains the same] ...

# Connect to Encoder
try:
    ser_encoder = serial.Serial(PORT_ENCODER, BAUD_RATE, timeout=1)
    time.sleep(2)  
    print(f"[SUCCESS] Connected to Encoder on {PORT_ENCODER}")
except Exception as e:
    print(f"[ERROR] Could not open Encoder port {PORT_ENCODER}: {e}")
    ser_motor.close()
    exit()


# ==================== NEW STARTUP HANDSHAKE ====================
print("\nPreparing motor... Moving to home (0 degrees) to sync...")
# Send 0 command twice to guarantee the motor driver parses it after boot
ser_motor.write(b"<om=0>")
ser_motor.flush()
time.sleep(1.0)
ser_motor.write(b"<om=0>")
ser_motor.flush()

# Give it a generous 4 seconds to do the initial travel and settle completely
time.sleep(4.0)

# Flush any garbage boot-up data out of both serial channels
ser_motor.reset_input_buffer()
ser_encoder.reset_input_buffer()
print("System synchronized. Starting calibration cycles...\n")
# ===============================================================


# Data structure to hold history based on bounded positions
grouped_data = defaultdict(list)
# ... [Rest of the script continues exactly the same] ...

# Data structure to hold history based on bounded positions
grouped_data = defaultdict(list)
BOUNDED_STEPS = [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330]

def print_summary_table():
    """Prints a clean, structured summary of the collected data so far."""
    print("\n" + "="*80)
    print(f"{'Target Position':<18} | {'Measured Positions (Last 5 Cycles Sample)':<50}")
    print("="*80)
    for target in BOUNDED_STEPS:
        measures = grouped_data[target]
        recent_measures = [f"{m:.2f}°" for m in measures[-5:]]
        measures_str = ", ".join(recent_measures)
        if len(measures) > 5:
            measures_str = "... " + measures_str
        print(f" Bounded {target:>3}°    | {measures_str}")
    print("="*80 + "\n")

# Start tracking the true cumulative absolute target position
absolute_target = 0

try:
    with open(CSV_FILENAME, mode='w', newline='', encoding='utf-8') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["Cycle", "Supposed Position", "Raw Angle", "Filtered Angle", "Field Status"])
        
        print(f"Starting test: {TOTAL_CYCLES} cycles. Logging to '{CSV_FILENAME}'...")
        
        for cycle in range(1, TOTAL_CYCLES + 1):
            print(f"--- Starting Cycle {cycle}/{TOTAL_CYCLES} ---")
            
            # Perform 12 moves per cycle to trace a full circle (0 to 330 in 30-degree steps)
            for _ in range(12):
                
                # 1. Send the monotonically increasing absolute value to the motor
                cmd = f"<om={absolute_target}>"
                ser_motor.write(cmd.encode('utf-8'))
                ser_motor.flush()  
                print(f" Sent command: {cmd} -> Motor traveling...")
                
                # 2. Wait for travel transit and dampening
                time.sleep(MOTOR_DELAY)
                
                # 3. Clear out serial cache lines
                ser_encoder.reset_input_buffer()
                
                # 4. REQUEST position snapshot
                print(" Requesting position snapshot...")
                ser_encoder.write(b'?')
                ser_encoder.flush()
                
                raw_val, filt_val, field_status = None, None, None
                while raw_val is None:
                    if ser_encoder.in_waiting > 0:
                        line = ser_encoder.readline().decode('utf-8').strip()
                        try:
                            raw_val, filt_val, field_status = map(float, line.split(','))
                        except ValueError:
                            continue
                
                print(f" Logged position: {filt_val:.2f}°")
                
                # Normalize target for the spreadsheet log (e.g., 390 becomes 30)
                supposed_pos_bounded = absolute_target % 360
                
                # 5. Log data and preserve metrics
                grouped_data[supposed_pos_bounded].append(filt_val)
                csv_writer.writerow([cycle, supposed_pos_bounded, raw_val, filt_val, int(field_status)])
                csv_file.flush()  
                
                print(" Ready for next target position.\n")
                
                # Advance the global absolute step angle forward by 30 degrees
                absolute_target += 30
            
            # Print updated table summary metrics on loop termination
            print_summary_table()
            
    print(f"[SUCCESS] Calibration complete! Full dataset saved to {CSV_FILENAME}")

except KeyboardInterrupt:
    print("\nExecution paused/interrupted by user.")
finally:
    ser_motor.close()
    ser_encoder.close()
    print("Serial ports securely closed.")