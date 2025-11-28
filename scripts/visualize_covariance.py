#!/usr/bin/env python3
"""
ESKF Error Covariance Matrix Heatmap Visualizer

This script visualizes the error covariance matrix from the ESKF estimator.
It creates heatmaps showing:
- Full 15x15 covariance matrix at selected time points
- Time evolution of diagonal elements (variances)
- Correlation matrix evolution

Usage:
    python3 visualize_covariance.py covariance_data.csv

Output:
    covariance_heatmap.png - Combined visualization
"""

import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm, SymLogNorm
from matplotlib.patches import Rectangle

# State variable names (15 error states)
STATE_NAMES = [
    r'$\delta p_x$', r'$\delta p_y$', r'$\delta p_z$',      # Position (0-2)
    r'$\delta v_x$', r'$\delta v_y$', r'$\delta v_z$',      # Velocity (3-5)
    r'$\delta\theta_x$', r'$\delta\theta_y$', r'$\delta\theta_z$',  # Attitude (6-8)
    r'$\delta b_{gx}$', r'$\delta b_{gy}$', r'$\delta b_{gz}$',     # Gyro bias (9-11)
    r'$\delta b_{ax}$', r'$\delta b_{ay}$', r'$\delta b_{az}$'      # Accel bias (12-14)
]

STATE_SHORT_NAMES = [
    'px', 'py', 'pz',
    'vx', 'vy', 'vz',
    'θx', 'θy', 'θz',
    'bgx', 'bgy', 'bgz',
    'bax', 'bay', 'baz'
]

# State groups for block visualization
STATE_GROUPS = [
    ('Position', 0, 3),
    ('Velocity', 3, 6),
    ('Attitude', 6, 9),
    ('Gyro Bias', 9, 12),
    ('Accel Bias', 12, 15)
]

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


def load_covariance_data(csv_path):
    """Load covariance CSV data."""
    df = pd.read_csv(csv_path)
    return df


def extract_covariance_matrix(row):
    """Extract 15x15 covariance matrix from a row."""
    P = np.zeros((15, 15))
    for i in range(15):
        for j in range(15):
            col_name = f'P_{i}_{j}'
            P[i, j] = row[col_name]
    return P


def extract_correlation_matrix(P):
    """Convert covariance matrix to correlation matrix."""
    diag = np.sqrt(np.diag(P))
    # Avoid division by zero
    diag[diag == 0] = 1e-10
    D_inv = np.diag(1.0 / diag)
    return D_inv @ P @ D_inv


def create_covariance_heatmap(ax, P, title, use_log=True):
    """Create a heatmap of the covariance matrix."""
    # Use symmetric log scale for better visualization of small/large values
    if use_log:
        # Handle negative values in off-diagonal
        vmax = np.max(np.abs(P))
        vmin = -vmax
        norm = SymLogNorm(linthresh=1e-10, vmin=vmin, vmax=vmax)
        im = ax.imshow(P, cmap='RdBu_r', norm=norm, aspect='equal')
    else:
        im = ax.imshow(P, cmap='RdBu_r', aspect='equal')

    plt.colorbar(im, ax=ax, shrink=0.8)

    # Labels
    ax.set_xticks(range(15))
    ax.set_yticks(range(15))
    ax.set_xticklabels(STATE_SHORT_NAMES, fontsize=7, rotation=45, ha='right')
    ax.set_yticklabels(STATE_SHORT_NAMES, fontsize=7)
    ax.set_title(title, fontsize=10)

    # Draw group boundaries
    for name, start, end in STATE_GROUPS:
        rect = Rectangle((start - 0.5, start - 0.5), end - start, end - start,
                         fill=False, edgecolor='black', linewidth=1.5)
        ax.add_patch(rect)


def create_diagonal_evolution(ax, df, indices, labels, title):
    """Plot time evolution of specific diagonal elements."""
    times = df['time'].values

    for idx, label in zip(indices, labels):
        col_name = f'P_{idx}_{idx}'
        values = np.sqrt(df[col_name].values)  # Convert variance to std dev
        ax.semilogy(times, values, label=label, linewidth=1.5)

    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Std Dev')
    ax.set_title(title)
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)


def create_correlation_heatmap(ax, corr, title):
    """Create a heatmap of the correlation matrix."""
    im = ax.imshow(corr, cmap='RdBu_r', vmin=-1, vmax=1, aspect='equal')
    plt.colorbar(im, ax=ax, shrink=0.8)

    # Labels
    ax.set_xticks(range(15))
    ax.set_yticks(range(15))
    ax.set_xticklabels(STATE_SHORT_NAMES, fontsize=7, rotation=45, ha='right')
    ax.set_yticklabels(STATE_SHORT_NAMES, fontsize=7)
    ax.set_title(title, fontsize=10)

    # Draw group boundaries
    for name, start, end in STATE_GROUPS:
        rect = Rectangle((start - 0.5, start - 0.5), end - start, end - start,
                         fill=False, edgecolor='black', linewidth=1.5)
        ax.add_patch(rect)


def create_block_trace_evolution(ax, df):
    """Plot time evolution of block traces (sum of variances for each state group)."""
    times = df['time'].values

    colors = ['red', 'blue', 'green', 'orange', 'purple']

    for (name, start, end), color in zip(STATE_GROUPS, colors):
        traces = []
        for _, row in df.iterrows():
            block_trace = 0
            for i in range(start, end):
                block_trace += row[f'P_{i}_{i}']
            traces.append(np.sqrt(block_trace))  # RMS of block
        ax.semilogy(times, traces, label=name, linewidth=1.5, color=color)

    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Block RMS Std Dev')
    ax.set_title('State Group Uncertainty Evolution')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)


def create_animated_heatmap_frames(df, output_prefix):
    """Create multiple heatmap frames for animation (optional)."""
    # Select key time points
    times_of_interest = [0.0, 1.0, 3.0, 5.0, 10.0, 15.0, 19.0]

    for t_target in times_of_interest:
        # Find closest time
        idx = (df['time'] - t_target).abs().idxmin()
        row = df.iloc[idx]
        actual_time = row['time']
        phase = int(row['phase'])

        P = extract_covariance_matrix(row)

        fig, ax = plt.subplots(figsize=(10, 8))
        create_covariance_heatmap(ax, P,
            f'Error Covariance at t={actual_time:.1f}s ({PHASE_NAMES.get(phase, "Unknown")})')

        output_path = f'{output_prefix}_t{actual_time:.1f}s.png'
        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"  Saved: {output_path}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 visualize_covariance.py <csv_file>")
        print("Example: python3 visualize_covariance.py covariance_data.csv")
        sys.exit(1)

    csv_path = sys.argv[1]
    print(f"Loading covariance data from: {csv_path}")

    try:
        df = load_covariance_data(csv_path)
    except Exception as e:
        print(f"Error loading CSV: {e}")
        sys.exit(1)

    print(f"Loaded {len(df)} time points")
    print(f"Time range: {df['time'].min():.2f}s to {df['time'].max():.2f}s")

    # ========== Create Main Figure ==========
    fig = plt.figure(figsize=(18, 12))
    fig.suptitle('ESKF Error Covariance Analysis', fontsize=14, fontweight='bold')

    # Select key time points for heatmaps
    time_points = [0.0, 5.0, 10.0, 19.0]  # Ground, Hover, Forward end, Final

    # Row 1: Covariance heatmaps at different times
    for i, t_target in enumerate(time_points):
        ax = fig.add_subplot(3, 4, i + 1)
        idx = (df['time'] - t_target).abs().idxmin()
        row = df.iloc[idx]
        P = extract_covariance_matrix(row)
        phase = int(row['phase'])
        create_covariance_heatmap(ax, P,
            f't={row["time"]:.1f}s ({PHASE_NAMES.get(phase, "?")})')

    # Row 2: Diagonal element evolution
    ax5 = fig.add_subplot(3, 4, 5)
    create_diagonal_evolution(ax5, df, [0, 1, 2],
        [r'$\sigma_{px}$', r'$\sigma_{py}$', r'$\sigma_{pz}$'],
        'Position Std Dev')

    ax6 = fig.add_subplot(3, 4, 6)
    create_diagonal_evolution(ax6, df, [3, 4, 5],
        [r'$\sigma_{vx}$', r'$\sigma_{vy}$', r'$\sigma_{vz}$'],
        'Velocity Std Dev')

    ax7 = fig.add_subplot(3, 4, 7)
    create_diagonal_evolution(ax7, df, [6, 7, 8],
        [r'$\sigma_{\theta x}$', r'$\sigma_{\theta y}$', r'$\sigma_{\theta z}$'],
        'Attitude Std Dev')

    ax8 = fig.add_subplot(3, 4, 8)
    create_block_trace_evolution(ax8, df)

    # Row 3: Correlation matrices at different times
    for i, t_target in enumerate([0.0, 5.0, 10.0, 19.0]):
        ax = fig.add_subplot(3, 4, 9 + i)
        idx = (df['time'] - t_target).abs().idxmin()
        row = df.iloc[idx]
        P = extract_covariance_matrix(row)
        corr = extract_correlation_matrix(P)
        phase = int(row['phase'])
        create_correlation_heatmap(ax, corr,
            f'Correlation t={row["time"]:.1f}s')

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    output_path = 'covariance_heatmap.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nMain figure saved to: {output_path}")

    # ========== Create Individual Frame Heatmaps (Optional) ==========
    print("\nGenerating individual time-point heatmaps...")
    create_animated_heatmap_frames(df, 'cov_frame')

    print("\nDone!")


if __name__ == "__main__":
    main()
