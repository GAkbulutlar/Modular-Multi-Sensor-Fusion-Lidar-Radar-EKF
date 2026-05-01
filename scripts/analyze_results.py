#!/usr/bin/env python3
"""
Multi-Sensor Fusion EKF Analysis Script

This script loads the CSV results from the C++ simulation and generates
visualization plots comparing:
- Ground Truth vs. Fused Estimate
- Lidar-only vs. Fused vs. Radar-only
- Position RMSE comparison
- Velocity estimates
- Filter trajectories on XY plane
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path
import sys


def load_results(csv_file):
    """Load simulation results from CSV file."""
    if not Path(csv_file).exists():
        print(f"Error: File '{csv_file}' not found.")
        print("Make sure to run the C++ simulation first and it generates 'fusion_results.csv'")
        sys.exit(1)
    
    df = pd.read_csv(csv_file)
    return df


def calculate_metrics(df):
    """Calculate error metrics for each filter."""
    metrics = {}
    
    # Position errors
    fused_pos_error = np.sqrt((df['fused_x'] - df['gt_x'])**2 + 
                              (df['fused_y'] - df['gt_y'])**2)
    lidar_pos_error = np.sqrt((df['lidar_x'] - df['gt_x'])**2 + 
                              (df['lidar_y'] - df['gt_y'])**2)
    radar_pos_error = np.sqrt((df['radar_x'] - df['gt_x'])**2 + 
                              (df['radar_y'] - df['gt_y'])**2)
    
    metrics['fused_rmse_pos'] = np.sqrt(np.mean(fused_pos_error**2))
    metrics['lidar_rmse_pos'] = np.sqrt(np.mean(lidar_pos_error**2))
    metrics['radar_rmse_pos'] = np.sqrt(np.mean(radar_pos_error**2))
    
    metrics['fused_mean_pos_error'] = np.mean(fused_pos_error)
    metrics['lidar_mean_pos_error'] = np.mean(lidar_pos_error)
    metrics['radar_mean_pos_error'] = np.mean(radar_pos_error)
    
    metrics['fused_max_pos_error'] = np.max(fused_pos_error)
    metrics['lidar_max_pos_error'] = np.max(lidar_pos_error)
    metrics['radar_max_pos_error'] = np.max(radar_pos_error)
    
    # Velocity errors
    fused_vel_error = np.sqrt((df['fused_vx'] - df['gt_vx'])**2 + 
                              (df['fused_vy'] - df['gt_vy'])**2)
    
    metrics['fused_rmse_vel'] = np.sqrt(np.mean(fused_vel_error**2))
    metrics['fused_mean_vel_error'] = np.mean(fused_vel_error)
    
    return metrics


def print_metrics(metrics):
    """Print error metrics."""
    print("\n" + "="*70)
    print("ERROR METRICS ANALYSIS")
    print("="*70)
    
    print("\nPosition RMSE (meters):")
    print(f"  Fused:       {metrics['fused_rmse_pos']:8.4f}")
    print(f"  Lidar-only:  {metrics['lidar_rmse_pos']:8.4f}")
    print(f"  Radar-only:  {metrics['radar_rmse_pos']:8.4f}")
    
    if metrics['lidar_rmse_pos'] > 1e-6:
        improvement = (metrics['lidar_rmse_pos'] - metrics['fused_rmse_pos']) / metrics['lidar_rmse_pos'] * 100
        print(f"  Improvement over Lidar-only: {improvement:.1f}%")
    
    if metrics['radar_rmse_pos'] > 1e-6:
        improvement = (metrics['radar_rmse_pos'] - metrics['fused_rmse_pos']) / metrics['radar_rmse_pos'] * 100
        print(f"  Improvement over Radar-only: {improvement:.1f}%")
    
    print("\nPosition Mean Error (meters):")
    print(f"  Fused:       {metrics['fused_mean_pos_error']:8.4f}")
    print(f"  Lidar-only:  {metrics['lidar_mean_pos_error']:8.4f}")
    print(f"  Radar-only:  {metrics['radar_mean_pos_error']:8.4f}")
    
    print("\nPosition Max Error (meters):")
    print(f"  Fused:       {metrics['fused_max_pos_error']:8.4f}")
    print(f"  Lidar-only:  {metrics['lidar_max_pos_error']:8.4f}")
    print(f"  Radar-only:  {metrics['radar_max_pos_error']:8.4f}")
    
    print("\nVelocity RMSE (m/s):")
    print(f"  Fused:       {metrics['fused_rmse_vel']:8.4f}")
    print(f"  Mean error:  {metrics['fused_mean_vel_error']:8.4f}")
    
    print("="*70)


def plot_trajectories(df):
    """Plot trajectory comparison: Ground Truth vs. Fused vs. Sensors."""
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    
    # Plot 1: XY trajectory
    ax = axes[0]
    ax.plot(df['gt_x'], df['gt_y'], 'k-', linewidth=2, label='Ground Truth', alpha=0.7)
    ax.plot(df['fused_x'], df['fused_y'], 'b-', linewidth=1.5, label='Fused EKF', alpha=0.8)
    ax.plot(df['lidar_x'], df['lidar_y'], 'g--', linewidth=1, label='Lidar-only', alpha=0.6)
    ax.plot(df['radar_x'], df['radar_y'], 'r--', linewidth=1, label='Radar-only', alpha=0.6)
    
    ax.set_xlabel('X Position (m)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Y Position (m)', fontsize=11, fontweight='bold')
    ax.set_title('2D Trajectories: Ground Truth vs. Fused vs. Sensors', fontsize=12, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # Plot 2: Position error over time
    ax = axes[1]
    fused_error = np.sqrt((df['fused_x'] - df['gt_x'])**2 + (df['fused_y'] - df['gt_y'])**2)
    lidar_error = np.sqrt((df['lidar_x'] - df['gt_x'])**2 + (df['lidar_y'] - df['gt_y'])**2)
    radar_error = np.sqrt((df['radar_x'] - df['gt_x'])**2 + (df['radar_y'] - df['gt_y'])**2)
    
    ax.plot(df['time'], fused_error, 'b-', linewidth=1.5, label='Fused EKF', alpha=0.8)
    ax.plot(df['time'], lidar_error, 'g--', linewidth=1, label='Lidar-only', alpha=0.6)
    ax.plot(df['time'], radar_error, 'r--', linewidth=1, label='Radar-only', alpha=0.6)
    
    ax.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Position Error (m)', fontsize=11, fontweight='bold')
    ax.set_title('Position Error Over Time', fontsize=12, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)
    
    plt.tight_layout()
    plt.savefig('1_trajectories.png', dpi=150, bbox_inches='tight')
    print("Saved: 1_trajectories.png")
    plt.close()


def plot_velocity(df):
    """Plot velocity comparison."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # VX components
    ax = axes[0, 0]
    ax.plot(df['time'], df['gt_vx'], 'k-', linewidth=2, label='Ground Truth', alpha=0.7)
    ax.plot(df['time'], df['fused_vx'], 'b-', linewidth=1.5, label='Fused EKF', alpha=0.8)
    ax.set_xlabel('Time (s)', fontsize=10, fontweight='bold')
    ax.set_ylabel('VX (m/s)', fontsize=10, fontweight='bold')
    ax.set_title('Velocity X Component', fontsize=11, fontweight='bold')
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, alpha=0.3)
    
    # VY components
    ax = axes[0, 1]
    ax.plot(df['time'], df['gt_vy'], 'k-', linewidth=2, label='Ground Truth', alpha=0.7)
    ax.plot(df['time'], df['fused_vy'], 'b-', linewidth=1.5, label='Fused EKF', alpha=0.8)
    ax.set_xlabel('Time (s)', fontsize=10, fontweight='bold')
    ax.set_ylabel('VY (m/s)', fontsize=10, fontweight='bold')
    ax.set_title('Velocity Y Component', fontsize=11, fontweight='bold')
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, alpha=0.3)
    
    # Speed magnitude
    ax = axes[1, 0]
    gt_speed = np.sqrt(df['gt_vx']**2 + df['gt_vy']**2)
    fused_speed = np.sqrt(df['fused_vx']**2 + df['fused_vy']**2)
    ax.plot(df['time'], gt_speed, 'k-', linewidth=2, label='Ground Truth', alpha=0.7)
    ax.plot(df['time'], fused_speed, 'b-', linewidth=1.5, label='Fused EKF', alpha=0.8)
    ax.set_xlabel('Time (s)', fontsize=10, fontweight='bold')
    ax.set_ylabel('Speed (m/s)', fontsize=10, fontweight='bold')
    ax.set_title('Velocity Magnitude', fontsize=11, fontweight='bold')
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, alpha=0.3)
    
    # Velocity error
    ax = axes[1, 1]
    vel_error = np.sqrt((df['fused_vx'] - df['gt_vx'])**2 + (df['fused_vy'] - df['gt_vy'])**2)
    ax.plot(df['time'], vel_error, 'b-', linewidth=1.5, alpha=0.8)
    ax.set_xlabel('Time (s)', fontsize=10, fontweight='bold')
    ax.set_ylabel('Velocity Error (m/s)', fontsize=10, fontweight='bold')
    ax.set_title('Velocity Error (Fused)', fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)
    
    plt.tight_layout()
    plt.savefig('2_velocity.png', dpi=150, bbox_inches='tight')
    print("Saved: 2_velocity.png")
    plt.close()


def plot_error_distribution(df):
    """Plot error distribution histograms."""
    fused_error = np.sqrt((df['fused_x'] - df['gt_x'])**2 + (df['fused_y'] - df['gt_y'])**2)
    lidar_error = np.sqrt((df['lidar_x'] - df['gt_x'])**2 + (df['lidar_y'] - df['gt_y'])**2)
    radar_error = np.sqrt((df['radar_x'] - df['gt_x'])**2 + (df['radar_y'] - df['gt_y'])**2)
    
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    
    # Fused histogram
    ax = axes[0]
    ax.hist(fused_error, bins=30, color='blue', alpha=0.7, edgecolor='black')
    ax.set_xlabel('Position Error (m)', fontsize=10, fontweight='bold')
    ax.set_ylabel('Frequency', fontsize=10, fontweight='bold')
    ax.set_title(f'Fused EKF Position Error\nRMSE={np.sqrt(np.mean(fused_error**2)):.4f} m', 
                 fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # Lidar-only histogram
    ax = axes[1]
    ax.hist(lidar_error, bins=30, color='green', alpha=0.7, edgecolor='black')
    ax.set_xlabel('Position Error (m)', fontsize=10, fontweight='bold')
    ax.set_ylabel('Frequency', fontsize=10, fontweight='bold')
    ax.set_title(f'Lidar-only Position Error\nRMSE={np.sqrt(np.mean(lidar_error**2)):.4f} m', 
                 fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # Radar-only histogram
    ax = axes[2]
    ax.hist(radar_error, bins=30, color='red', alpha=0.7, edgecolor='black')
    ax.set_xlabel('Position Error (m)', fontsize=10, fontweight='bold')
    ax.set_ylabel('Frequency', fontsize=10, fontweight='bold')
    ax.set_title(f'Radar-only Position Error\nRMSE={np.sqrt(np.mean(radar_error**2)):.4f} m', 
                 fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig('3_error_distribution.png', dpi=150, bbox_inches='tight')
    print("Saved: 3_error_distribution.png")
    plt.close()


def plot_performance_comparison(df):
    """Plot performance comparison bars."""
    fused_error = np.sqrt((df['fused_x'] - df['gt_x'])**2 + (df['fused_y'] - df['gt_y'])**2)
    lidar_error = np.sqrt((df['lidar_x'] - df['gt_x'])**2 + (df['lidar_y'] - df['gt_y'])**2)
    radar_error = np.sqrt((df['radar_x'] - df['gt_x'])**2 + (df['radar_y'] - df['gt_y'])**2)
    
    metrics = {
        'RMSE': [np.sqrt(np.mean(fused_error**2)), 
                 np.sqrt(np.mean(lidar_error**2)), 
                 np.sqrt(np.mean(radar_error**2))],
        'Mean Error': [np.mean(fused_error), 
                      np.mean(lidar_error), 
                      np.mean(radar_error)],
        'Max Error': [np.max(fused_error), 
                     np.max(lidar_error), 
                     np.max(radar_error)],
    }
    
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    x_labels = ['Fused EKF', 'Lidar-only', 'Radar-only']
    colors = ['blue', 'green', 'red']
    
    for idx, (metric_name, values) in enumerate(metrics.items()):
        ax = axes[idx]
        bars = ax.bar(x_labels, values, color=colors, alpha=0.7, edgecolor='black', linewidth=1.5)
        
        # Add value labels on bars
        for bar, value in zip(bars, values):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{value:.4f}', ha='center', va='bottom', fontweight='bold')
        
        ax.set_ylabel('Error (m)', fontsize=10, fontweight='bold')
        ax.set_title(f'{metric_name}', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3, axis='y')
        ax.set_ylim(bottom=0)
    
    plt.tight_layout()
    plt.savefig('4_performance_comparison.png', dpi=150, bbox_inches='tight')
    print("Saved: 4_performance_comparison.png")
    plt.close()


def plot_position_components(df):
    """Plot X and Y position components separately."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # X Position
    ax = axes[0, 0]
    ax.plot(df['time'], df['gt_x'], 'k-', linewidth=2, label='Ground Truth', alpha=0.7)
    ax.plot(df['time'], df['fused_x'], 'b-', linewidth=1.5, label='Fused EKF', alpha=0.8)
    ax.plot(df['time'], df['lidar_x'], 'g--', linewidth=1, label='Lidar-only', alpha=0.6)
    ax.plot(df['time'], df['radar_x'], 'r--', linewidth=1, label='Radar-only', alpha=0.6)
    ax.set_xlabel('Time (s)', fontsize=10, fontweight='bold')
    ax.set_ylabel('X Position (m)', fontsize=10, fontweight='bold')
    ax.set_title('X Position Component', fontsize=11, fontweight='bold')
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, alpha=0.3)
    
    # Y Position
    ax = axes[0, 1]
    ax.plot(df['time'], df['gt_y'], 'k-', linewidth=2, label='Ground Truth', alpha=0.7)
    ax.plot(df['time'], df['fused_y'], 'b-', linewidth=1.5, label='Fused EKF', alpha=0.8)
    ax.plot(df['time'], df['lidar_y'], 'g--', linewidth=1, label='Lidar-only', alpha=0.6)
    ax.plot(df['time'], df['radar_y'], 'r--', linewidth=1, label='Radar-only', alpha=0.6)
    ax.set_xlabel('Time (s)', fontsize=10, fontweight='bold')
    ax.set_ylabel('Y Position (m)', fontsize=10, fontweight='bold')
    ax.set_title('Y Position Component', fontsize=11, fontweight='bold')
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, alpha=0.3)
    
    # X Position Error
    ax = axes[1, 0]
    x_error = np.abs(df['fused_x'] - df['gt_x'])
    lidar_x_error = np.abs(df['lidar_x'] - df['gt_x'])
    radar_x_error = np.abs(df['radar_x'] - df['gt_x'])
    ax.plot(df['time'], x_error, 'b-', linewidth=1.5, label='Fused', alpha=0.8)
    ax.plot(df['time'], lidar_x_error, 'g--', linewidth=1, label='Lidar-only', alpha=0.6)
    ax.plot(df['time'], radar_x_error, 'r--', linewidth=1, label='Radar-only', alpha=0.6)
    ax.set_xlabel('Time (s)', fontsize=10, fontweight='bold')
    ax.set_ylabel('X Position Error (m)', fontsize=10, fontweight='bold')
    ax.set_title('X Position Error', fontsize=11, fontweight='bold')
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)
    
    # Y Position Error
    ax = axes[1, 1]
    y_error = np.abs(df['fused_y'] - df['gt_y'])
    lidar_y_error = np.abs(df['lidar_y'] - df['gt_y'])
    radar_y_error = np.abs(df['radar_y'] - df['gt_y'])
    ax.plot(df['time'], y_error, 'b-', linewidth=1.5, label='Fused', alpha=0.8)
    ax.plot(df['time'], lidar_y_error, 'g--', linewidth=1, label='Lidar-only', alpha=0.6)
    ax.plot(df['time'], radar_y_error, 'r--', linewidth=1, label='Radar-only', alpha=0.6)
    ax.set_xlabel('Time (s)', fontsize=10, fontweight='bold')
    ax.set_ylabel('Y Position Error (m)', fontsize=10, fontweight='bold')
    ax.set_title('Y Position Error', fontsize=11, fontweight='bold')
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)
    
    plt.tight_layout()
    plt.savefig('5_position_components.png', dpi=150, bbox_inches='tight')
    print("Saved: 5_position_components.png")
    plt.close()


def main():
    """Main analysis function."""
    print("\n" + "="*70)
    print("Multi-Sensor Fusion EKF Analysis")
    print("="*70)
    
    # Load results
    csv_file = 'fusion_results.csv'
    print(f"\nLoading results from '{csv_file}'...")
    df = load_results(csv_file)
    
    print(f"Loaded {len(df)} samples")
    print(f"Simulation time: {df['time'].min():.2f}s to {df['time'].max():.2f}s")
    
    # Calculate metrics
    metrics = calculate_metrics(df)
    print_metrics(metrics)
    
    # Generate plots
    print("\nGenerating visualization plots...")
    print("-" * 70)
    
    plot_trajectories(df)
    plot_velocity(df)
    plot_error_distribution(df)
    plot_performance_comparison(df)
    plot_position_components(df)
    
    print("-" * 70)
    print("\nAnalysis complete! Generated 5 PNG files:")
    print("  1. 1_trajectories.png - XY trajectories and error over time")
    print("  2. 2_velocity.png - Velocity components and magnitude")
    print("  3. 3_error_distribution.png - Error histograms")
    print("  4. 4_performance_comparison.png - Performance bars")
    print("  5. 5_position_components.png - Position components and errors")
    print("\n" + "="*70)


if __name__ == '__main__':
    main()
