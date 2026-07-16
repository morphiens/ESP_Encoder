import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# --- Configuration ---
CSV_FILENAME = "motor_encoder_continuous_calibration.csv"

# 1. Load and Clean Data
try:
    df = pd.read_csv(CSV_FILENAME)
except FileNotFoundError:
    print(f"[ERROR] Could not find '{CSV_FILENAME}'. Please run this script in the same directory.")
    exit()

# Group by the target/supposed positions (0, 90, 180, 270)
# Note: In case your CSV contains 360, we map it to 0 for consistent angular grouping
df['Supposed Position'] = df['Supposed Position'].replace(360, 0)
unique_positions = sorted(df['Supposed Position'].unique())

# --- 2. Calculate and Display Key Statistics ---
print("=" * 75)
print(f"{'Target (mm/deg)':<15} | {'Min Value':<12} | {'Max Value':<12} | {'Delta (Max-Min)':<15} | {'Variance':<12}")
print("=" * 75)

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

print("=" * 75)


# --- 3. Plotting Setup ---
# Set styling for clear charts
sns.set_theme(style="whitegrid")
fig = plt.figure(figsize=(16, 10))
grid = fig.add_gridspec(2, 2, hspace=0.3, wspace=0.25)

# --- Plot A: Variance Comparison (Bar Chart) ---
ax_var = fig.add_subplot(grid[0, 0])
variances = [stats_summary[pos]['variance'] for pos in unique_positions]
std_devs = [stats_summary[pos]['std_dev'] for pos in unique_positions]
x_labels = [f"{pos}°" for pos in unique_positions]

bars = ax_var.bar(x_labels, variances, color='#3498db', edgecolor='black', alpha=0.85, width=0.5)
ax_var.set_title("Variance (Noise Floor) at Each Angle", fontsize=13, fontweight='bold', pad=12)
ax_var.set_ylabel("Variance (Degrees²)", fontsize=11)
ax_var.set_xlabel("Target Bounded Angle", fontsize=11)

# Annotate bars with standard deviation for easier interpretation
for bar, std in zip(bars, std_devs):
    height = bar.get_height()
    ax_var.annotate(f"SD: ±{std:.3f}°",
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom', fontsize=10, fontweight='bold')


# --- Plot B: Range Deltas (Boxplot Spread) ---
ax_box = fig.add_subplot(grid[0, 1])
# Create boxplot of the filtered angles per position
boxplot_data = [stats_summary[pos]['data'] for pos in unique_positions]
ax_box.boxplot(boxplot_data, tick_labels=x_labels, patch_artist=True,
               boxprops=dict(facecolor='#2ecc71', color='black', alpha=0.7),
               medianprops=dict(color='red', linewidth=1.5))
ax_box.set_title("Full Spread (Boxplot) at Each Target Position", fontsize=13, fontweight='bold', pad=12)
ax_box.set_ylabel("Measured Encoder Angle (Degrees)", fontsize=11)
ax_box.set_xlabel("Target Bounded Angle", fontsize=11)


# --- Plot C & D: Distributions / Histograms ---
# Placing histograms side-by-side or stacked using subgrids
hist_colors = ['#e74c3c', '#9b59b6', '#f1c40f', '#1abc9c']
ax_hist = fig.add_subplot(grid[1, :])

for i, pos in enumerate(unique_positions):
    # Normalize the data around its mean so we can overlay them and inspect distribution shapes
    zero_centered_data = stats_summary[pos]['data'] - stats_summary[pos]['data'].mean()
    sns.kdeplot(zero_centered_data, fill=True, ax=ax_hist, label=f"{pos}° target", 
                color=hist_colors[i % len(hist_colors)], alpha=0.4, linewidth=2)

ax_hist.set_title("Overlay of Measurement Noise Distributions (Mean Centered to 0.0°)", fontsize=13, fontweight='bold', pad=12)
ax_hist.set_xlabel("Deviation from Target Mean (Degrees)", fontsize=11)
ax_hist.set_ylabel("Density", fontsize=11)
ax_hist.legend(fontsize=10)

plt.suptitle(f"TLE5012B Linear Axis Repeatability Report ({len(df)//4} Complete Cycles)", fontsize=16, fontweight='bold', y=0.96)
plt.show()

# ===========================================================================
# Target (mm/deg) | Min Value    | Max Value    | Delta (Max-Min) | Variance    
# ===========================================================================
#   0° (Supposed) |     73.851° |     73.909° |        0.0580° |   0.000148
#  90° (Supposed) |    344.148° |    344.202° |        0.0540° |   0.000134
# 180° (Supposed) |    253.128° |    253.189° |        0.0610° |   0.000207
# 270° (Supposed) |    162.777° |    162.843° |        0.0660° |   0.000211
# ===========================================================================
