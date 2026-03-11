#!/usr/bin/env python3

"""plot_wrench_csv.py

Plot wrench data from CSV file without pandas dependency.
"""

import csv
import matplotlib.pyplot as plt
import argparse
from collections import defaultdict


def plot_wrench_data(csv_file):
    """Plot wrench data from CSV file."""
    
    # Data storage organized by sensor
    data = defaultdict(lambda: {
        'time': [],
        'fx': [], 'fy': [], 'fz': [],
        'tx': [], 'ty': [], 'tz': []
    })
    
    # Read CSV file
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        
        for row in reader:
            sensor = row['sensor']
            time = float(row['time'])
            
            data[sensor]['time'].append(time)
            data[sensor]['fx'].append(float(row['fx']))
            data[sensor]['fy'].append(float(row['fy']))
            data[sensor]['fz'].append(float(row['fz']))
            data[sensor]['tx'].append(float(row['tx']))
            data[sensor]['ty'].append(float(row['ty']))
            data[sensor]['tz'].append(float(row['tz']))
    
    sensors = list(data.keys())
    print(f"Found sensors: {sensors}")
    print(f"Data points per sensor:")
    for sensor in sensors:
        print(f"  - {sensor}: {len(data[sensor]['time'])} points")
    
    # Create figure with 2 subplots
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    
    # Plot forces
    ax_force = axes[0]
    ax_force.set_title('Forces Comparison', fontsize=14, fontweight='bold')
    ax_force.set_xlabel('Time (s)')
    ax_force.set_ylabel('Force (N)')
    ax_force.grid(True, alpha=0.3)
    
    # Plot torques
    ax_torque = axes[1]
    ax_torque.set_title('Torques Comparison', fontsize=14, fontweight='bold')
    ax_torque.set_xlabel('Time (s)')
    ax_torque.set_ylabel('Torque (Nm)')
    ax_torque.grid(True, alpha=0.3)
    
    # Define line styles for each sensor
    styles = {
        'coinft': '-',
        'flexiv': '--',
        'force_torque': ':',
        'ft': ':'  # Alternative name
    }
    
    colors = {
        'fx': '#1f77b4',
        'fy': '#ff7f0e',
        'fz': '#2ca02c',
        'tx': '#d62728',
        'ty': '#9467bd',
        'tz': '#8c564b'
    }
    
    # Plot each sensor
    for sensor in sensors:
        style = styles.get(sensor, '-')
        sensor_data = data[sensor]
        
        # Plot forces
        ax_force.plot(sensor_data['time'], sensor_data['fx'], 
                     style, color=colors['fx'], label=f'{sensor} Fx', linewidth=1.5)
        ax_force.plot(sensor_data['time'], sensor_data['fy'], 
                     style, color=colors['fy'], label=f'{sensor} Fy', linewidth=1.5)
        ax_force.plot(sensor_data['time'], sensor_data['fz'], 
                     style, color=colors['fz'], label=f'{sensor} Fz', linewidth=1.5)
        
        # Plot torques
        ax_torque.plot(sensor_data['time'], sensor_data['tx'], 
                      style, color=colors['tx'], label=f'{sensor} Tx', linewidth=1.5)
        ax_torque.plot(sensor_data['time'], sensor_data['ty'], 
                      style, color=colors['ty'], label=f'{sensor} Ty', linewidth=1.5)
        ax_torque.plot(sensor_data['time'], sensor_data['tz'], 
                      style, color=colors['tz'], label=f'{sensor} Tz', linewidth=1.5)
    
    ax_force.legend(loc='upper right', ncol=3, fontsize=8)
    ax_torque.legend(loc='upper right', ncol=3, fontsize=8)
    
    plt.tight_layout()
    
    # Save figure option
    save_path = csv_file.replace('.csv', '_plot.png')
    fig.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"\n✓ Plot saved to: {save_path}")
    
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Plot wrench data from CSV')
    parser.add_argument('csv_file', help='Path to CSV file')
    
    args = parser.parse_args()
    
    try:
        plot_wrench_data(args.csv_file)
    except FileNotFoundError:
        print(f"Error: Could not find file '{args.csv_file}'")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == '__main__':
    main()