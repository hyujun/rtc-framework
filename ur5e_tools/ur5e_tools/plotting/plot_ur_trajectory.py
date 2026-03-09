#!/usr/bin/env python3
"""
plot_ur_trajectory.py - v1

Visualize UR5e trajectory data from CSV logs.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import argparse


def load_csv_log(filepath):
    """Load CSV log file."""
    df = pd.read_csv(filepath)
    return df


def plot_joint_positions(df, joint_idx, save_path=None):
    """Plot position tracking for a specific joint."""
    plt.figure(figsize=(12, 6))
    
    current_col = f'current_pos_{joint_idx}'
    target_col = f'target_pos_{joint_idx}'
    command_col = f'command_{joint_idx}'
    
    plt.plot(df['timestamp'], df[current_col], 
             label='Current', linewidth=2)
    plt.plot(df['timestamp'], df[target_col], 
             label='Target', linestyle='--', linewidth=2)
    plt.plot(df['timestamp'], df[command_col], 
             label='Command', linestyle=':', alpha=0.7)
    
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Position (rad)', fontsize=12)
    plt.title(f'Joint {joint_idx} Position Tracking', fontsize=14)
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3)
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f'Saved: {save_path}')
    else:
        plt.show()
    
    plt.close()


def plot_all_joints(df, save_dir=None):
    """Plot all 6 joints in subplots."""
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    axes = axes.flatten()
    
    joint_names = ['Base', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3']
    
    for i in range(6):
        ax = axes[i]
        current_col = f'current_pos_{i}'
        target_col = f'target_pos_{i}'
        
        ax.plot(df['timestamp'], df[current_col], label='Current')
        ax.plot(df['timestamp'], df[target_col], label='Target', linestyle='--')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (rad)')
        ax.set_title(f'Joint {i}: {joint_names[i]}')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_dir:
        save_path = Path(save_dir) / 'all_joints.png'
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f'Saved: {save_path}')
    else:
        plt.show()
    
    plt.close()


def plot_tracking_error(df, save_path=None):
    """Plot tracking error for all joints."""
    plt.figure(figsize=(12, 6))
    
    for i in range(6):
        current_col = f'current_pos_{i}'
        target_col = f'target_pos_{i}'
        error = df[target_col] - df[current_col]
        plt.plot(df['timestamp'], error, label=f'Joint {i}', alpha=0.7)
    
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Tracking Error (rad)', fontsize=12)
    plt.title('Joint Position Tracking Errors', fontsize=14)
    plt.legend(fontsize=10, ncol=2)
    plt.grid(True, alpha=0.3)
    plt.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f'Saved: {save_path}')
    else:
        plt.show()
    
    plt.close()


def print_statistics(df):
    """Print trajectory statistics."""
    print('\n=== Trajectory Statistics ===')
    print(f'Duration: {df["timestamp"].iloc[-1] - df["timestamp"].iloc[0]:.2f} s')
    print(f'Samples: {len(df)}')
    print(f'Average rate: {len(df) / (df["timestamp"].iloc[-1] - df["timestamp"].iloc[0]):.1f} Hz')
    
    print('\nTracking Errors (RMS):')
    for i in range(6):
        current_col = f'current_pos_{i}'
        target_col = f'target_pos_{i}'
        error = df[target_col] - df[current_col]
        rms_error = np.sqrt(np.mean(error**2))
        print(f'  Joint {i}: {rms_error:.4f} rad ({np.rad2deg(rms_error):.2f}°)')


def main():
    parser = argparse.ArgumentParser(
        description='Plot UR5e trajectory from CSV log')
    parser.add_argument('csv_file', type=str, help='Path to CSV log file')
    parser.add_argument('--joint', type=int, default=None,
                       help='Plot specific joint (0-5)')
    parser.add_argument('--save-dir', type=str, default=None,
                       help='Directory to save plots')
    parser.add_argument('--stats', action='store_true',
                       help='Print statistics only')
    
    args = parser.parse_args()
    
    # Load data
    print(f'Loading: {args.csv_file}')
    df = load_csv_log(args.csv_file)
    
    # Print statistics
    print_statistics(df)
    
    if args.stats:
        return
    
    # Create save directory
    if args.save_dir:
        Path(args.save_dir).mkdir(parents=True, exist_ok=True)
    
    # Plot
    if args.joint is not None:
        # Single joint
        save_path = None
        if args.save_dir:
            save_path = Path(args.save_dir) / f'joint_{args.joint}.png'
        plot_joint_positions(df, args.joint, save_path)
    else:
        # All joints
        plot_all_joints(df, args.save_dir)
        
        # Tracking error
        save_path = None
        if args.save_dir:
            save_path = Path(args.save_dir) / 'tracking_error.png'
        plot_tracking_error(df, save_path)


if __name__ == '__main__':
    main()
