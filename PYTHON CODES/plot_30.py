import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# --- Configuration ---
# CSV_FILENAME = "motor_encoder_continuous_calibration.csv"
CSV_FILENAME = "TLE_2Stage_motor_encoder.csv"

# 1. Load and Clean Data
try:
    df = pd.read_csv(CSV_FILENAME)
except FileNotFoundError:
    print(f"[ERROR] Could not find '{CSV_FILENAME}'. Please run this script in the same directory.")
    exit()

# Group by the target/supposed positions (0, 30, 60, ..., 330)
# Map 360 to 0 for consistent angular grouping
df['Supposed Position'] = df['Supposed Position'].replace(360, 0)
unique_positions = sorted(df['Supposed Position'].unique())
num_positions = len(unique_positions)

# --- 2. Calculate and Display Key Statistics ---
print("=" * 85)
print(f"{'Target (deg)':<15} | {'Min Value':<12} | {'Max Value':<12} | {'Delta (Max-Min)':<15} | {'Variance':<12}")
print("=" * 85)

stats_summary = {}
for pos in unique_positions:
    pos_data = df[df['Supposed Position'] == pos]['Filtered Angle']
    
    min_val = pos_data.min()
    max_val = pos_data.max()
    delta = max_val - min_val
    variance = pos_data.var()
    std_dev = pos_data.std()
    
    stats_summary[pos] = {
        'min': min_val, 'max': max_val, 'delta': delta, 
        'variance': variance, 'std_dev': std_dev, 'data': pos_data
    }
    
    print(f"{pos:>3}° (Supposed) | {min_val:>10.3f}° | {max_val:>10.3f}° | {delta:>13.4f}° | {variance:>10.6f}")

print("=" * 85)


# --- 3. Plotting Setup ---
sns.set_theme(style="whitegrid")
fig = plt.figure(figsize=(16, 11))
grid = fig.add_gridspec(2, 2, hspace=0.35, wspace=0.25)

# Generate 12 distinct colors
colors = sns.color_palette("husl", num_positions)
x_labels = [f"{pos}°" for pos in unique_positions]

# --- Plot A: Variance Comparison (Bar Chart) ---
ax_var = fig.add_subplot(grid[0, 0])
variances = [stats_summary[pos]['variance'] for pos in unique_positions]
std_devs = [stats_summary[pos]['std_dev'] for pos in unique_positions]

bars = ax_var.bar(x_labels, variances, color=colors, edgecolor='black', alpha=0.8, width=0.6)
ax_var.set_title("Variance (Noise Floor) at Each Angle", fontsize=13, fontweight='bold', pad=12)
ax_var.set_ylabel("Variance (Degrees²)", fontsize=11)
ax_var.set_xlabel("Target Bounded Angle", fontsize=11)
ax_var.tick_params(axis='x', rotation=45)

# Annotate bars with standard deviation (using smaller font to fit 12 columns cleanly)
for bar, std in zip(bars, std_devs):
    height = bar.get_height()
    ax_var.annotate(f"±{std:.3f}°",
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    xytext=(0, 3),  
                    textcoords="offset points",
                    ha='center', va='bottom', fontsize=8, fontweight='bold')


# --- Plot B: Range Deltas (Boxplot Spread) ---
ax_box = fig.add_subplot(grid[0, 1])
boxplot_data = [stats_summary[pos]['data'] for pos in unique_positions]

box = ax_box.boxplot(boxplot_data, tick_labels=x_labels, patch_artist=True,
               medianprops=dict(color='red', linewidth=1.5))

# Color individual boxes
for patch, color in zip(box['boxes'], colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.6)

ax_box.set_title("Full Spread (Boxplot) at Each Target Position", fontsize=13, fontweight='bold', pad=12)
ax_box.set_ylabel("Measured Encoder Angle (Degrees)", fontsize=11)
ax_box.set_xlabel("Target Bounded Angle", fontsize=11)
ax_box.tick_params(axis='x', rotation=45)


# --- Plot C: Distributions / Histograms ---
ax_hist = fig.add_subplot(grid[1, :])

for i, pos in enumerate(unique_positions):
    # Normalize the data around its mean to overlay and inspect distribution profiles
    zero_centered_data = stats_summary[pos]['data'] - stats_summary[pos]['data'].mean()
    sns.kdeplot(zero_centered_data, fill=True, ax=ax_hist, label=f"{pos}°", 
                color=colors[i], alpha=0.15, linewidth=1.5)

ax_hist.set_title("Overlay of Measurement Noise Distributions (Mean Centered to 0.0°)", fontsize=13, fontweight='bold', pad=12)
ax_hist.set_xlabel("Deviation from Target Mean (Degrees)", fontsize=11)
ax_hist.set_ylabel("Density", fontsize=11)

# Position legend to the side so it doesn't cover data curves
ax_hist.legend(title="Targets", bbox_to_anchor=(1.01, 1), loc='upper left', fontsize=9)

# Calculate total completed cycles dynamically
total_cycles = len(df) // num_positions
plt.suptitle(f"TLE Rotary Axis 30 Degree, 2 stage filter ({total_cycles} Complete {num_positions}-Step Cycles)", 
             fontsize=16, fontweight='bold', y=0.96)

plt.show()