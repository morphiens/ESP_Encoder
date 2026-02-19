# Block Height Measurement System

## Overview
Measure vertical height of blocks using a 40mm link attached to an encoder shaft. The system calculates height from rotation angle using trigonometry.

## Files
- `block_height_encoder.ino` - ESP32 firmware (same as ultra precision encoder)
- `block_height_receiver.py` - Python receiver with height calculation

## Setup
1. Attach 40mm link perpendicular to encoder shaft
2. Upload `block_height_encoder.ino` to ESP32C6
3. Install dependencies: `pip install bleak numpy scipy`
4. Run `python block_height_receiver.py`

## Usage

### Measuring Block Height
1. **Zero at ground**: Touch link end to ground, press `Z`
2. **Measure block**: Touch link end to block top, press `Enter`
3. **Read height**: System displays vertical distance in mm

### Commands
- `Enter` - Measure angle and calculate height
- `Z` - Zero encoder at current position (ground reference)
- `D` - Toggle diagnostic data display (ON/OFF)
- `L` - Set link length (default: 40mm)
- `C` - **Calibrate starting angle** (new!)
- `Ctrl+C` - Quit

## Calibration Mode (NEW!)

The `starting_angle` parameter accounts for the initial angle of the link when the encoder is at zero. Use calibration mode to determine the optimal value.

### How to Calibrate

1. Press `C` to enter calibration mode
2. For each of 5 measurements:
   - Position link at a known height
   - Press Enter to measure angle
   - Input the actual height in mm
3. System calculates optimal `starting_angle` using least-squares optimization
4. Review the verification errors
5. Apply or discard the calibration

### Example Calibration Session

```
>> Command [Enter/Z/D/L/C]: c

============================================================
 STARTING ANGLE CALIBRATION MODE
============================================================

Instructions:
1. Position the link at a known height
2. Press Enter to measure the angle
3. Enter the actual height in mm
4. Repeat for 5 different heights

[1/5] Press Enter to measure...
  Measured angle: 0.15234°
  Enter actual height (mm): 5.0
  ✓ Point 1 recorded

[2/5] Press Enter to measure...
  Measured angle: 2.45123°
  Enter actual height (mm): 10.0
  ✓ Point 2 recorded

... (3 more measurements)

Calculating optimal starting angle...

============================================================
 CALIBRATION RESULTS
============================================================

Old starting angle: 59.200°
New starting angle: 58.743°

Verification:
  Point 1: Actual=5.00mm, Predicted=5.02mm, Error=+0.02mm
  Point 2: Actual=10.00mm, Predicted=9.98mm, Error=-0.02mm
  Point 3: Actual=15.00mm, Predicted=15.01mm, Error=+0.01mm
  Point 4: Actual=20.00mm, Predicted=19.99mm, Error=-0.01mm
  Point 5: Actual=25.00mm, Predicted=25.00mm, Error=+0.00mm

Apply this calibration? (y/n): y
✓ Starting angle updated to 58.743°
Note: This value is not saved permanently. Update it in the code if needed.
```

## Mathematics

### Height Calculation Formula
```
height = (L × cos(θ₀)) - (L × cos(θ₀ + θ))

Where:
  L  = link length (40mm)
  θ₀ = starting_angle (calibrated parameter)
  θ  = measured angle from encoder
```

### Calibration Algorithm

The calibration uses **least-squares optimization** to find the `starting_angle` that minimizes prediction errors:

1. Collect N measurements: (θᵢ, hᵢ) where hᵢ is actual height
2. Define error function:
   ```
   E(θ₀) = Σ [h_predicted(θ₀, θᵢ) - hᵢ]²
   ```
3. Minimize E(θ₀) using scipy.optimize.minimize (Nelder-Mead method)
4. Return optimal θ₀

**Why this works:** The starting angle affects all measurements systematically. By measuring multiple known heights, we can back-calculate the angle that makes all predictions match reality.

## Features

### Color-Coded Output
- **Cyan (Bold)** - Angle measurements
- **Green (Bold)** - Height measurements  
- **Yellow** - Diagnostic data
- **Magenta** - Info messages
- **Red** - Critical warnings
- **Blue** - Calibration mode

### Diagnostic Toggle
- Press 'D' to show/hide diagnostic data
- Keeps display clean during measurements
- Still monitors for warnings (MAGL, MAGH, COF)

### Angle Unwrapping
- Automatically handles 0°/360° boundary crossings
- Prevents false high jitter readings near zero
- Ensures accurate statistics

### High Precision
- 4096-sample averaging per measurement
- Median of 5 measurements
- Sub-micron angle precision
- Height precision depends on link length and angle

## Output Example

```
------------------------------------------------------------
 ANGLE          : 15.23400 degrees
 VERTICAL HEIGHT: 10.523 mm
------------------------------------------------------------
 Read SUCCESS
```

With diagnostics enabled (press 'D'):
```
------------------------------------------------------------
 ANGLE          : 15.23400 degrees
 VERTICAL HEIGHT: 10.523 mm

 DIAGNOSTIC DATA:
   RAW JITTER   : 0.00120 deg (0.8 microns)
   ALL READINGS : [15.23222, 15.23252, 15.23459, 15.23559, 15.23407]
   AGC (Gain)   : 70
   MAG (Magn.)  : 4640
   MAGL (High)  : 0
   MAGH (Low)   : 0
   COF (Ovflow) : 0
------------------------------------------------------------
 Read SUCCESS
```

## Notes
- Firmware includes angle inversion (360° - angle) for correct orientation
- Zero offset stored in RAM (resets on ESP32 power cycle)
- **Starting angle calibration not saved permanently** - update `self.starting_angle` in code after calibration
- Link must be perpendicular to shaft for accurate measurements
- AGC value of ~128 indicates optimal magnetic field strength
- For best calibration results, use heights spanning your measurement range

---

**Author:** Swaraj Dangare
