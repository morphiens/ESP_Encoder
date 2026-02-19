# Polynomial Curve Fitting Version

## Overview
This is a **simplified version** that uses polynomial regression to directly map angle → height, **without any trigonometry or starting_angle parameter**.

## Key Differences from Trigonometric Version

| Feature | Trigonometric Version | Polynomial Version |
|---------|----------------------|-------------------|
| **Model** | `height = L×cos(θ₀) - L×cos(θ₀+θ)` | `height = a₀ + a₁×θ + a₂×θ² + ...` |
| **Parameters** | `starting_angle`, `link_length` | Polynomial coefficients |
| **Calibration** | Optimizes `starting_angle` | Fits polynomial curve |
| **Complexity** | Physics-based (requires understanding) | Pure data fitting (black box) |
| **Extrapolation** | Good (follows physics) | Poor (can diverge outside range) |

## Usage

### 1. Run the Script
```bash
python3 block_height_receiver_polynomial.py
```

### 2. Calibrate First (REQUIRED)
Press `C` to enter calibration mode:
- Choose polynomial degree (1-5, default=2)
  - **1 = Linear**: Simple, but may not fit well
  - **2 = Quadratic**: Good balance (recommended)
  - **3 = Cubic**: More flexible
  - **4-5 = Higher order**: Risk of overfitting
- Measure at least `degree + 2` points (e.g., 4 points for quadratic)
- More points = better fit
- Type `done` when finished

### 3. Measure Heights
After calibration, press `Enter` to measure.

## Commands
- `Enter` - Measure angle and calculate height
- `Z` - Zero encoder at current position
- `D` - Toggle diagnostic display
- `C` - Calibrate polynomial curve
- `Ctrl+C` - Quit

## Example Calibration

```
>> Command [Enter/Z/D/C]: c

============================================================
 POLYNOMIAL CURVE FITTING CALIBRATION
============================================================

Polynomial degree (1=linear, 2=quadratic, 3=cubic) [default=2]: 2

Using 2-degree polynomial

[Point 1] Press Enter to measure...
  Measured angle: 0.00000°
  Enter actual height (mm): 0.0
  ✓ Point 1 recorded

[Point 2] Press Enter to measure...
  Measured angle: 4.98470°
  Enter actual height (mm): 3.0
  ✓ Point 2 recorded

... (collect more points)

[Point 10] Press Enter to measure...
  Measured angle: 22.52680°
  Enter actual height (mm): 14.455
  ✓ Point 10 recorded

[Point 11] Press Enter to measure (or type 'done' to finish)...done

Fitting 2-degree polynomial...

============================================================
 CALIBRATION RESULTS
============================================================

Polynomial degree: 2
Number of points: 10
R² (fit quality): 0.999876 (1.0 = perfect)
RMS Error: 0.0523 mm

Polynomial coefficients:
  a2: -1.234567e-03
  a1: 6.543210e-01
  a0: 1.234567e-02

height = 0.0123 + 0.6543*angle - 0.0012*angle^2

Verification:
Point    Angle        Actual     Predicted    Error     
------------------------------------------------------------
1        0.00000      0.000      0.012        +0.012
2        4.98470      3.000      2.997        -0.003
...
10       22.52680     14.455     14.450       -0.005

✓ Calibration complete! Polynomial coefficients saved.
```

## Advantages
✅ **No physics knowledge needed** - Just fit a curve to data
✅ **No starting_angle** - One less parameter to worry about
✅ **Flexible** - Can fit any smooth curve
✅ **Simple** - Easy to understand

## Disadvantages
❌ **Black box** - No physical meaning
❌ **Poor extrapolation** - Don't measure outside calibrated range
❌ **Overfitting risk** - High-degree polynomials can be unstable
❌ **Needs recalibration** - If link length changes

## Recommendations
- Use **2nd degree (quadratic)** for most cases
- Calibrate with **8-15 points** spread across your measurement range
- Include points at 0mm (ground reference)
- Don't extrapolate beyond calibrated range
- R² > 0.999 indicates excellent fit

## When to Use This Version
- You want simplicity over physics
- You don't care about the physical model
- You're measuring within a fixed range
- You want to avoid trigonometry

## When to Use Trigonometric Version
- You want physically meaningful parameters
- You need to extrapolate beyond calibrated range
- You want to understand the system behavior
- Link length might change
