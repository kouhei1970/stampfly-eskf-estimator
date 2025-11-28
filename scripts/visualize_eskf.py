#!/usr/bin/env python3
"""
ESKF Estimator Simulation Results Visualizer

This script visualizes the results from complex_simulation_eskf.cpp.
It creates comprehensive plots showing:
- Position comparison (truth vs estimate)
- Velocity comparison
- Attitude comparison
- Position error with uncertainty bounds
- 3D trajectory

Usage:
    python3 visualize_eskf.py simulation_results.csv

Output:
    eskf_results.png - Combined visualization
"""

import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d import Axes3D

# Flight phase names
PHASE_NAMES = {
    0: 'Ground',
    1: 'Takeoff',
    2: 'Hover 1',
    3: 'Forward',
    4: 'Hover 2',
    5: 'Yaw',
    6: 'Landing',
    7: 'Complete'
}

# Phase colors for background shading
PHASE_COLORS = {
    0: '#f0f0f0',  # Ground - light gray
    1: '#ffe6e6',  # Takeoff - light red
    2: '#e6ffe6',  # Hover 1 - light green
    3: '#e6e6ff',  # Forward - light blue
    4: '#e6ffe6',  # Hover 2 - light green
    5: '#fff0e6',  # Yaw - light orange
    6: '#ffe6e6',  # Landing - light red
    7: '#f0f0f0',  # Complete - light gray
}


def load_data(csv_path):
    """Load CSV data into pandas DataFrame."""
    df = pd.read_csv(csv_path)
    return df


def add_phase_background(ax, df):
    """Add colored background for different flight phases."""
    phases = df['phase'].values
    times = df['time'].values

    current_phase = phases[0]
    phase_start = times[0]

    for i in range(1, len(phases)):
        if phases[i] != current_phase or i == len(phases) - 1:
            phase_end = times[i]
            color = PHASE_COLORS.get(current_phase, '#ffffff')
            ax.axvspan(phase_start, phase_end, alpha=0.3, color=color)
            current_phase = phases[i]
            phase_start = times[i]


def create_position_plot(ax, df):
    """Create position comparison plot."""
    add_phase_background(ax, df)

    # Truth
    ax.plot(df['time'], df['truth_px'], 'r-', label='Truth X', linewidth=1.5)
    ax.plot(df['time'], df['truth_py'], 'g-', label='Truth Y', linewidth=1.5)
    ax.plot(df['time'], df['truth_pz'], 'b-', label='Truth Z', linewidth=1.5)

    # Estimate
    ax.plot(df['time'], df['est_px'], 'r--', label='Est X', linewidth=1.0, alpha=0.8)
    ax.plot(df['time'], df['est_py'], 'g--', label='Est Y', linewidth=1.0, alpha=0.8)
    ax.plot(df['time'], df['est_pz'], 'b--', label='Est Z', linewidth=1.0, alpha=0.8)

    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Position [m]')
    ax.set_title('Position: Truth vs Estimate')
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.3)


def create_velocity_plot(ax, df):
    """Create velocity comparison plot."""
    add_phase_background(ax, df)

    # Truth
    ax.plot(df['time'], df['truth_vx'], 'r-', label='Truth Vx', linewidth=1.5)
    ax.plot(df['time'], df['truth_vy'], 'g-', label='Truth Vy', linewidth=1.5)
    ax.plot(df['time'], df['truth_vz'], 'b-', label='Truth Vz', linewidth=1.5)

    # Estimate
    ax.plot(df['time'], df['est_vx'], 'r--', label='Est Vx', linewidth=1.0, alpha=0.8)
    ax.plot(df['time'], df['est_vy'], 'g--', label='Est Vy', linewidth=1.0, alpha=0.8)
    ax.plot(df['time'], df['est_vz'], 'b--', label='Est Vz', linewidth=1.0, alpha=0.8)

    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Velocity [m/s]')
    ax.set_title('Velocity: Truth vs Estimate')
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.3)


def create_attitude_plot(ax, df):
    """Create attitude comparison plot."""
    add_phase_background(ax, df)

    # Truth
    ax.plot(df['time'], df['truth_roll'], 'r-', label='Truth Roll', linewidth=1.5)
    ax.plot(df['time'], df['truth_pitch'], 'g-', label='Truth Pitch', linewidth=1.5)
    ax.plot(df['time'], df['truth_yaw'], 'b-', label='Truth Yaw', linewidth=1.5)

    # Estimate
    ax.plot(df['time'], df['est_roll'], 'r--', label='Est Roll', linewidth=1.0, alpha=0.8)
    ax.plot(df['time'], df['est_pitch'], 'g--', label='Est Pitch', linewidth=1.0, alpha=0.8)
    ax.plot(df['time'], df['est_yaw'], 'b--', label='Est Yaw', linewidth=1.0, alpha=0.8)

    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Angle [deg]')
    ax.set_title('Attitude: Truth vs Estimate')
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.3)


def create_error_plot(ax, df):
    """Create position error with uncertainty bounds."""
    add_phase_background(ax, df)

    # Error components
    ax.plot(df['time'], df['err_px'], 'r-', label='Error X', linewidth=1.0)
    ax.plot(df['time'], df['err_py'], 'g-', label='Error Y', linewidth=1.0)
    ax.plot(df['time'], df['err_pz'], 'b-', label='Error Z', linewidth=1.0)

    # 3-sigma bounds (±3σ)
    ax.fill_between(df['time'], -3*df['std_px'], 3*df['std_px'],
                    alpha=0.2, color='red', label='±3σ X')
    ax.fill_between(df['time'], -3*df['std_py'], 3*df['std_py'],
                    alpha=0.2, color='green', label='±3σ Y')
    ax.fill_between(df['time'], -3*df['std_pz'], 3*df['std_pz'],
                    alpha=0.2, color='blue', label='±3σ Z')

    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Error [m]')
    ax.set_title('Position Error with 3σ Bounds')
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.3)


def create_error_norm_plot(ax, df):
    """Create total position error magnitude plot."""
    add_phase_background(ax, df)

    ax.plot(df['time'], df['err_norm'], 'k-', label='|Error|', linewidth=1.5)

    # Compute 3-sigma bound for total error
    sigma_total = np.sqrt(df['std_px']**2 + df['std_py']**2 + df['std_pz']**2)
    ax.fill_between(df['time'], 0, 3*sigma_total,
                    alpha=0.3, color='gray', label='3σ bound')

    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Error [m]')
    ax.set_title('Position Error Magnitude')
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)


def create_3d_trajectory(ax, df):
    """Create 3D trajectory plot."""
    # Truth trajectory
    ax.plot(df['truth_px'], df['truth_py'], df['truth_pz'],
            'b-', label='Truth', linewidth=2.0)

    # Estimate trajectory
    ax.plot(df['est_px'], df['est_py'], df['est_pz'],
            'r--', label='Estimate', linewidth=1.5, alpha=0.8)

    # Start and end markers
    ax.scatter([df['truth_px'].iloc[0]], [df['truth_py'].iloc[0]],
               [df['truth_pz'].iloc[0]], c='green', s=100, marker='o', label='Start')
    ax.scatter([df['truth_px'].iloc[-1]], [df['truth_py'].iloc[-1]],
               [df['truth_pz'].iloc[-1]], c='red', s=100, marker='x', label='End')

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('3D Trajectory')
    ax.legend(loc='upper left', fontsize=8)

    # Set axis limits based on actual data range (including estimates)
    # Add small padding for visibility
    padding = 0.2

    x_min = min(df['truth_px'].min(), df['est_px'].min()) - padding
    x_max = max(df['truth_px'].max(), df['est_px'].max()) + padding
    y_min = min(df['truth_py'].min(), df['est_py'].min()) - padding
    y_max = max(df['truth_py'].max(), df['est_py'].max()) + padding
    z_min = min(df['truth_pz'].min(), df['est_pz'].min()) - padding
    z_max = max(df['truth_pz'].max(), df['est_pz'].max()) + padding

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_zlim(z_min, z_max)

    # Try to set equal aspect ratio (may not work perfectly in all matplotlib versions)
    try:
        ax.set_box_aspect([x_max - x_min, y_max - y_min, z_max - z_min])
    except AttributeError:
        pass  # Older matplotlib versions don't support this


def create_phase_legend():
    """Create a legend for flight phases."""
    patches = [Patch(facecolor=color, alpha=0.3, label=name)
               for phase, (name, color) in enumerate(zip(
                   ['Ground', 'Takeoff', 'Hover', 'Forward', 'Yaw', 'Landing'],
                   ['#f0f0f0', '#ffe6e6', '#e6ffe6', '#e6e6ff', '#fff0e6', '#ffe6e6']
               ))]
    return patches


def compute_statistics(df):
    """Compute and return statistics."""
    stats = {
        'mean_error': df['err_norm'].mean(),
        'max_error': df['err_norm'].max(),
        'final_error': df['err_norm'].iloc[-1],
        'rmse_x': np.sqrt((df['err_px']**2).mean()),
        'rmse_y': np.sqrt((df['err_py']**2).mean()),
        'rmse_z': np.sqrt((df['err_pz']**2).mean()),
        'rmse_total': np.sqrt((df['err_norm']**2).mean()),
    }
    return stats


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 visualize_eskf.py <csv_file>")
        print("Example: python3 visualize_eskf.py simulation_results.csv")
        sys.exit(1)

    csv_path = sys.argv[1]
    print(f"Loading data from: {csv_path}")

    try:
        df = load_data(csv_path)
    except Exception as e:
        print(f"Error loading CSV: {e}")
        sys.exit(1)

    print(f"Loaded {len(df)} data points")
    print(f"Time range: {df['time'].min():.2f}s to {df['time'].max():.2f}s")

    # Compute statistics
    stats = compute_statistics(df)
    print("\n=== Error Statistics ===")
    print(f"Mean position error: {stats['mean_error']:.4f} m")
    print(f"Max position error:  {stats['max_error']:.4f} m")
    print(f"Final position error: {stats['final_error']:.4f} m")
    print(f"RMSE (X, Y, Z): ({stats['rmse_x']:.4f}, {stats['rmse_y']:.4f}, {stats['rmse_z']:.4f}) m")
    print(f"RMSE (total): {stats['rmse_total']:.4f} m")

    # Create figure
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle('ESKF Estimator Simulation Results', fontsize=14, fontweight='bold')

    # Create subplots
    ax1 = fig.add_subplot(2, 3, 1)  # Position
    ax2 = fig.add_subplot(2, 3, 2)  # Velocity
    ax3 = fig.add_subplot(2, 3, 3)  # Attitude
    ax4 = fig.add_subplot(2, 3, 4)  # Error components
    ax5 = fig.add_subplot(2, 3, 5)  # Error magnitude
    ax6 = fig.add_subplot(2, 3, 6, projection='3d')  # 3D trajectory

    # Create plots
    create_position_plot(ax1, df)
    create_velocity_plot(ax2, df)
    create_attitude_plot(ax3, df)
    create_error_plot(ax4, df)
    create_error_norm_plot(ax5, df)
    create_3d_trajectory(ax6, df)

    # Add statistics text box
    stats_text = (
        f"Statistics:\n"
        f"Mean Error: {stats['mean_error']:.3f} m\n"
        f"Max Error: {stats['max_error']:.3f} m\n"
        f"RMSE: {stats['rmse_total']:.3f} m"
    )
    fig.text(0.02, 0.02, stats_text, fontsize=9,
             verticalalignment='bottom', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # Adjust layout
    plt.tight_layout(rect=[0, 0.05, 1, 0.97])

    # Save figure
    output_path = 'eskf_results.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nFigure saved to: {output_path}")

    # Show the plot if running interactively (not in headless mode)
    # Comment out or use plt.show() manually if you want to see the plot
    # plt.show()
    print("(To view interactively, uncomment plt.show() in the script)")


if __name__ == "__main__":
    main()
