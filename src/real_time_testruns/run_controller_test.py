#!/usr/bin/env python3

import os
import math
import yaml
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory


class TestCycleNode(Node):
    def __init__(self):
        super().__init__('test_cycle_node')
        
        # -------------------------------------------------------
        # 1) Load the YAML configuration
        # -------------------------------------------------------
        package_share_dir = get_package_share_directory('control_lib_ros2')
        config_file_name = 'config.yaml' 
        config_file_path = os.path.join(package_share_dir, 'config', config_file_name)
        
        self.get_logger().info(f'Loading YAML config: {config_file_path}')
        try:
            with open(config_file_path, 'r') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML file: {e}")
            raise
        
        test_cycle_cfg = self.config['test_cycle']
        
        # 2) Extract test parameters
        self.delta_t = float(test_cycle_cfg.get('delta_t', 0.1))
        self.time_per_segment = float(test_cycle_cfg.get('time_per_segment', 5.0))
        self.targets = test_cycle_cfg.get('targets', [0.0])
        self.nr_inputs = test_cycle_cfg.get('nr_inputs', 1)
        self.nr_outputs = test_cycle_cfg.get('nr_outputs', 1)
        
        self.num_segments = len(self.targets)
        
        self.get_logger().info(
            f"TestCycleNode config: delta_t={self.delta_t}, "
            f"time_per_segment={self.time_per_segment}, "
            f"targets={self.targets}, nr_inputs={self.nr_inputs}, nr_outputs={self.nr_outputs}"
        )
        
        # -------------------------------------------------------
        # 3) Publishers & Subscribers
        # -------------------------------------------------------
        # We assume your controller node listens on "control_target" for the desired setpoint
        # and publishes the measured "control_input" (or output).
        
        self.target_pub = self.create_publisher(Float64MultiArray, 'control_target', 10)
        self.input_sub = self.create_subscription(
            Float64MultiArray, 'optimal_control_input', self.input_callback, 10)
        
        # -------------------------------------------------------
        # 4) Storage for data
        # -------------------------------------------------------
        self.timestamps = []
        self.measured_inputs = []
        self.desired_targets = []
        self.errors = []
        self.segment_indices = []
        
        # The segment index we are currently on:
        self.current_segment = 0
        
        # We note the "start time" as soon as the node starts:
        self.start_time = self.get_clock().now()
        
        # -------------------------------------------------------
        # 5) Create a timer for stepping through each target
        # -------------------------------------------------------
        self.timer_ = self.create_timer(self.delta_t, self.timer_callback)
        self.get_logger().info("TestCycleNode initialized and timer started.")
    
    def timer_callback(self):
        """Called at a fixed interval (delta_t). Publishes the current target,
        and advances to the next target after time_per_segment seconds.
        """
        # Publish the current target
        target_msg = Float64MultiArray()
        target_msg.data = [float(self.targets[self.current_segment])]
        self.target_pub.publish(target_msg)
        
        # Check if we should move to the next segment
        now = self.get_clock().now()
        elapsed_secs = (now - self.start_time).nanoseconds * 1e-9
        if elapsed_secs >= (self.time_per_segment * (self.current_segment + 1)):
            self.current_segment += 1
            
            if self.current_segment >= self.num_segments:
                self.get_logger().info("All target segments have been executed.")
                
                # Stop the timer
                self.timer_.cancel()
                
                # Calculate statistics
                self.calculate_statistics()
                
                # Plot results
                self.plot_results()
                
                # Shutdown
                self.get_logger().info("Shutting down TestCycleNode.")
                rclpy.shutdown()

    def input_callback(self, msg: Float64MultiArray):
        """Whenever a new control_input is received, store the error vs the target."""
        # For 1D case: we only read the first value
        measured_value = msg.data[0]
        current_target = float(self.targets[self.current_segment])
        
        error = current_target - measured_value
        
        now = self.get_clock().now()
        t_sec = (now - self.start_time).nanoseconds * 1e-9
        
        self.timestamps.append(t_sec)
        self.measured_inputs.append(measured_value)
        self.desired_targets.append(current_target)
        self.errors.append(error)
        self.segment_indices.append(self.current_segment)
    
    def calculate_statistics(self):
        """Compute mean error, MSE, RMSE for each segment and overall."""
        seg_array = np.array(self.segment_indices, dtype=int)
        err_array = np.array(self.errors, dtype=float)
        
        # Overall:
        overall_mse = np.mean(err_array**2) if len(err_array) > 0 else float('nan')
        overall_rmse = math.sqrt(overall_mse) if len(err_array) > 0 else float('nan')
        overall_mean_err = np.mean(err_array) if len(err_array) > 0 else float('nan')
        
        self.get_logger().info(
            f"Overall statistics => MSE: {overall_mse:.4f}, "
            f"RMSE: {overall_rmse:.4f}, Mean Error: {overall_mean_err:.4f}"
        )
        
        # Per segment:
        for seg_idx in range(self.num_segments):
            mask = (seg_array == seg_idx)
            seg_errs = err_array[mask]
            if len(seg_errs) < 1:
                continue
            
            mse = np.mean(seg_errs**2)
            rmse = math.sqrt(mse)
            mean_err = np.mean(seg_errs)
            self.get_logger().info(
                f"Segment {seg_idx} (target={self.targets[seg_idx]}) => "
                f"MSE: {mse:.4f}, RMSE: {rmse:.4f}, Mean Error: {mean_err:.4f}"
            )
    
    def plot_results(self):
        """Plot the measured inputs vs. time, plus the error timeline."""
        times = np.array(self.timestamps)
        measured = np.array(self.measured_inputs)
        desired = np.array(self.desired_targets)
        errors = np.array(self.errors)
        
        plt.figure(figsize=(10, 6))
        
        # Subplot 1: Measured vs Desired
        plt.subplot(2, 1, 1)
        plt.plot(times, measured, label='Measured (control_input)')
        plt.plot(times, desired, label='Desired (target)')
        plt.xlabel('Time [s]')
        plt.ylabel('Value')
        plt.title('Control Input vs. Target')
        plt.legend()
        
        # Subplot 2: Error
        plt.subplot(2, 1, 2)
        plt.plot(times, errors, label='Error (target - input)')
        plt.xlabel('Time [s]')
        plt.ylabel('Error')
        plt.title('Error Over Time')
        plt.legend()
        
        plt.tight_layout()
        plt.savefig('test_cycle_results.png')
        self.get_logger().info("Saved plot to 'test_cycle_results.png'.")
        
        # Optionally display the figure
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = TestCycleNode()
    rclpy.spin(node)
    # Normally, spin() blocks until shutdown, but we also call shutdown above 
    # after finishing all segments. So in practice, we may never get here.
    rclpy.shutdown()


if __name__ == '__main__':
    main()
