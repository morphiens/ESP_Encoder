#!/usr/bin/env python3
"""
Offline calibration script to calculate optimal starting_angle from collected data
Author: Swaraj Dangare
"""

import math
from scipy.optimize import least_squares

# Calibration data: (angle_deg, actual_height_mm)
calibration_data = [
    (359.99749, 0.00),
    (0.86023, 0.5),
    (1.69302, 1.005),
    (1.79689, 1.05),
    (1.99437, 1.180),
    (4.98561, 3.00),
    (8.52139, 5.185),
    (12.08255, 7.50),
    (15.24499, 9.055),
    (22.52680, 14.455),
]

# System parameters
link_length = 40.0  # mm
initial_guess = 59.2  # degrees (current starting_angle)

def residuals(starting_angle_guess):
    """Calculate residuals (errors) for least_squares optimization"""
    errors = []
    for angle_deg, actual_height in calibration_data:
        # Calculate predicted height
        angle_rad = math.radians(starting_angle_guess[0] + angle_deg)
        predicted_height = (link_length * math.cos(math.radians(starting_angle_guess[0]))) - \
                         (link_length * math.cos(angle_rad))
        error = predicted_height - actual_height
        errors.append(error)
    return errors

# Optimize starting_angle using least_squares (Levenberg-Marquardt)
print("Calculating optimal starting angle...")
result = least_squares(residuals, [initial_guess], method='lm')
optimal_angle = result.x[0]

print("\n" + "="*60)
print(" CALIBRATION RESULTS")
print("="*60)
print(f"\nOld starting angle: {initial_guess:.3f}°")
print(f"New starting angle: {optimal_angle:.3f}°")

# Show prediction errors
print(f"\nVerification:")
print(f"{'Point':<8} {'Angle':<12} {'Actual':<10} {'Predicted':<12} {'Error':<10}")
print("-" * 60)

total_squared_error = 0
for i, (angle_deg, actual_height) in enumerate(calibration_data, 1):
    angle_rad = math.radians(optimal_angle + angle_deg)
    predicted_height = (link_length * math.cos(math.radians(optimal_angle))) - \
                     (link_length * math.cos(angle_rad))
    error = predicted_height - actual_height
    total_squared_error += error**2
    print(f"{i:<8} {angle_deg:<12.5f} {actual_height:<10.3f} {predicted_height:<12.3f} {error:+10.3f}")

rms_error = math.sqrt(total_squared_error / len(calibration_data))
print("-" * 60)
print(f"RMS Error: {rms_error:.4f} mm")

print(f"\n✓ Update self.starting_angle = {optimal_angle:.3f} in your code")
