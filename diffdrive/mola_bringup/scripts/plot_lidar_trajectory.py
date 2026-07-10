#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import math
import tkinter as tk
from tkinter import ttk
import threading

from std_msgs.msg import String  
import re

# Import MOLA service
from mola_msgs.srv import MolaRuntimeParamSet

def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        self.x_data, self.y_data, self.z_data = [], [], []
        self.roll_data, self.pitch_data, self.yaw_data = [], [], []
        self.icp_quality_history = []  
        
        # Subscribe to odometry
        self.create_subscription(Odometry, '/state_estimator/pose', self.odometry_callback, 10)
        
        # Subscribe to diagnostics
        self.create_subscription(
            String,
            '/mola_diagnostics/lidar_odom/status',
            self.status_callback, 
            10)
        
        # Publisher for initial pose with best-effort QoS
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos
        )
        
        # Create service client for MOLA runtime parameters
        self.mola_param_client = self.create_client(
            MolaRuntimeParamSet,
            '/mola_runtime_param_set'
        )
        
        self.get_logger().info("Trajectory Plotter with Initial Pose Publisher started")

    def odometry_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        self.x_data.append(pos.x)
        self.y_data.append(pos.y)
        self.z_data.append(pos.z)

        roll, pitch, yaw = quaternion_to_euler(ori.x, ori.y, ori.z, ori.w)
        self.roll_data.append(roll)
        self.pitch_data.append(pitch)
        self.yaw_data.append(yaw)

    def status_callback(self, msg):
        text = msg.data
        match = re.search(r"icp_quality:\s*([\d\.]+)", text)
        
        if match:
            try:
                quality_value = float(match.group(1))
                self.icp_quality_history.append(quality_value)
            except ValueError:
                self.get_logger().warn("Could not parse ICP quality value.")

    def get_data(self):
        return (
            np.array(self.x_data),
            np.array(self.y_data),
            np.array(self.z_data),
            np.array(self.roll_data),
            np.array(self.pitch_data),
            np.array(self.yaw_data),
            np.array(self.icp_quality_history),
        )
    
    def set_mola_parameter(self, param_yaml):
        """Call MOLA runtime parameter service"""
        if not self.mola_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("MOLA parameter service not available")
            return False
        
        request = MolaRuntimeParamSet.Request()
        request.parameters = param_yaml
        
        future = self.mola_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(f"MOLA parameter set - Success: {response.success}, Message: {response.error_message}")
                return True
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                return False
        else:
            self.get_logger().warn("Service call timed out")
            return False
    
    def activate_mola(self):
        """Set MOLA to active with localization mode"""
        success = True
        
        # 1. Set active: true
        param1 = "mola::LidarOdometry:lidar_odom:\n  active: true\n"
        if self.set_mola_parameter(param1):
            self.get_logger().info("✓ MOLA activated")
        else:
            success = False
        
        # 2. Set mapping_enabled: false
        param2 = "mola::LidarOdometry:lidar_odom:\n  mapping_enabled: false\n"
        if self.set_mola_parameter(param2):
            self.get_logger().info("✓ Mapping disabled")
        else:
            success = False
        
        # 3. Set generate_simplemap: false
        param3 = "mola::LidarOdometry:lidar_odom:\n  generate_simplemap: false\n"
        if self.set_mola_parameter(param3):
            self.get_logger().info("✓ Simplemap generation disabled")
        else:
            success = False
        
        return success
    
    def set_enforce_planar_motion(self, enable):
        """Enable or disable planar motion constraint"""
        # Module instance name format is: classname:label
        # Try StateEstimationSimple first (default), then StateEstimationSmoother

        # Try StateEstimationSimple
        param_simple = f"mola::state_estimation_simple::StateEstimationSimple:state_estimation:\n  enforce_planar_motion: {str(enable).lower()}\n"
        if self.set_mola_parameter(param_simple):
            state = "enabled" if enable else "disabled"
            self.get_logger().info(f"✓ Planar motion constraint {state} (StateEstimationSimple)")
            return True

        # Try StateEstimationSmoother
        param_smoother = f"mola::state_estimation_smoother::StateEstimationSmoother:state_estimation:\n  enforce_planar_motion: {str(enable).lower()}\n"
        if self.set_mola_parameter(param_smoother):
            state = "enabled" if enable else "disabled"
            self.get_logger().info(f"✓ Planar motion constraint {state} (StateEstimationSmoother)")
            return True

        self.get_logger().warn("Failed to set planar motion constraint - state estimator module not found")
        return False
    
    def publish_initial_pose(self, x, y, z, yaw_degrees):
        """Publish initial pose for localization and activate MOLA"""
        # First activate MOLA
        self.get_logger().info("Activating MOLA...")
        mola_success = self.activate_mola()
        
        # Then publish the initial pose
        msg = PoseWithCovarianceStamped()
        
        # Header
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        
        # Convert Yaw to Quaternion
        yaw_rad = math.radians(yaw_degrees)
        q_w = math.cos(yaw_rad * 0.5)
        q_x = 0.0
        q_y = 0.0
        q_z = math.sin(yaw_rad * 0.5)
        
        # Orientation
        msg.pose.pose.orientation.w = q_w
        msg.pose.pose.orientation.x = q_x
        msg.pose.pose.orientation.y = q_y
        msg.pose.pose.orientation.z = q_z
        
        # Covariance matrix (6x6 = 36 elements)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.06, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.06, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06
        ]
        
        self.pose_pub.publish(msg)
        
        if mola_success:
            self.get_logger().info(f"✓ Published initial pose: x={x}, y={y}, z={z}, yaw={yaw_degrees}° (MOLA activated)")
        else:
            self.get_logger().warn(f"⚠ Published initial pose: x={x}, y={y}, z={z}, yaw={yaw_degrees}° (MOLA activation had issues)")
        
        return mola_success
    
    def clear_trajectory(self):
        """Clear all trajectory data"""
        self.x_data.clear()
        self.y_data.clear()
        self.z_data.clear()
        self.roll_data.clear()
        self.pitch_data.clear()
        self.yaw_data.clear()
        self.icp_quality_history.clear()
        self.get_logger().info("Trajectory data cleared")


class TrajectoryGUI:
    def __init__(self, node):
        self.node = node
        self.window = tk.Tk()
        self.window.title("LIDAR Trajectory Plotter & Initial Pose")
        self.window.geometry("1200x750")
        
        # Style
        style = ttk.Style()
        style.theme_use('clam')
        
        # Create notebook (tab container)
        notebook = ttk.Notebook(self.window)
        notebook.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Create tabs
        self.graph_tab = ttk.Frame(notebook)
        self.pose_tab = ttk.Frame(notebook)
        
        notebook.add(self.graph_tab, text='  Trajectory Graphs  ')
        notebook.add(self.pose_tab, text='  Initial Pose & Settings  ')
        
        # Setup tabs
        self.setup_graph_tab()
        self.setup_pose_tab()
        
        # Enable planar motion at startup
        self.window.after(1000, self.initialize_planar_mode)
        
        # Handle window close
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Animation
        self.ani = None
        self.start_animation()
    
    def setup_graph_tab(self):
        """Setup the matplotlib graphs tab"""
        # Create matplotlib figure
        self.fig, self.axes = plt.subplots(2, 3, figsize=(14, 7))
        self.fig.suptitle("LIDAR Odometry + Orientation (RPY) + ICP Quality", 
                          fontsize=14, fontweight='bold')
        
        # XY plane
        self.line_xy, = self.axes[0, 0].plot([], [], 'b-', linewidth=1.5, alpha=0.8)
        self.axes[0, 0].set_title("XY Plane (Top View)")
        self.axes[0, 0].set_xlabel("X (m)")
        self.axes[0, 0].set_ylabel("Y (m)")
        self.axes[0, 0].grid(True, alpha=0.3)

        # Combined XZ + YZ plane
        self.line_xz, = self.axes[0, 1].plot([], [], 'r-', linewidth=1.5, alpha=0.8, label='XZ')
        self.line_yz, = self.axes[0, 1].plot([], [], 'g--', linewidth=1.2, alpha=0.8, label='YZ')
        self.axes[0, 1].set_title("XZ + YZ Planes (Side Views)")
        self.axes[0, 1].set_xlabel("X/Y (m)")
        self.axes[0, 1].set_ylabel("Z (m)")
        self.axes[0, 1].grid(True, alpha=0.3)
        self.axes[0, 1].legend()

        # ICP Quality graph
        self.line_quality, = self.axes[0, 2].plot([], [], 'm-', linewidth=1.5)
        self.axes[0, 2].set_title("ICP Quality Over Time")
        self.axes[0, 2].set_xlabel("Time (samples)")
        self.axes[0, 2].set_ylabel("ICP Quality")
        self.axes[0, 2].grid(True, alpha=0.3)

        # Orientation (Roll, Pitch, Yaw)
        labels = ['Roll (rad)', 'Pitch (rad)', 'Yaw (rad)']
        colors = ['c', 'y', 'orange']
        self.ori_lines = []
        for i, (label, color) in enumerate(zip(labels, colors)):
            line, = self.axes[1, i].plot([], [], color=color, linewidth=1.5)
            self.axes[1, i].set_title(label)
            self.axes[1, i].set_xlabel("Time (samples)")
            self.axes[1, i].set_ylabel("Angle (rad)")
            self.axes[1, i].grid(True, alpha=0.3)
            self.ori_lines.append(line)
        
        self.fig.tight_layout()
        
        # Create control frame at the bottom
        control_frame = ttk.Frame(self.graph_tab)
        control_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)
        
        # Clear button
        clear_btn = ttk.Button(control_frame, text="Clear Trajectory", 
                               command=self.clear_trajectory, width=15)
        clear_btn.pack(side=tk.LEFT, padx=5)
        
        # Embed matplotlib figure in tkinter
        canvas = FigureCanvasTkAgg(self.fig, master=self.graph_tab)
        canvas.draw()
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    
    def setup_pose_tab(self):
        """Setup the initial pose publisher tab"""
        main_frame = ttk.Frame(self.pose_tab, padding="20")
        main_frame.pack(fill='both', expand=True)
        
        # Title
        title = ttk.Label(main_frame, text="Set Initial Pose for Localization", 
                          font=('Arial', 16, 'bold'))
        title.grid(row=0, column=0, columnspan=2, pady=(0, 20))
        
        # X input
        ttk.Label(main_frame, text="X Position:", font=('Arial', 12)).grid(
            row=1, column=0, sticky=tk.W, pady=8)
        self.x_var = tk.StringVar(value="0.0")
        x_entry = ttk.Entry(main_frame, textvariable=self.x_var, width=18, 
                            font=('Arial', 12))
        x_entry.grid(row=1, column=1, sticky=tk.E, pady=8, padx=(15, 0))
        
        # Y input
        ttk.Label(main_frame, text="Y Position:", font=('Arial', 12)).grid(
            row=2, column=0, sticky=tk.W, pady=8)
        self.y_var = tk.StringVar(value="0.0")
        y_entry = ttk.Entry(main_frame, textvariable=self.y_var, width=18, 
                            font=('Arial', 12))
        y_entry.grid(row=2, column=1, sticky=tk.E, pady=8, padx=(15, 0))
        
        # Z input
        ttk.Label(main_frame, text="Z Position:", font=('Arial', 12)).grid(
            row=3, column=0, sticky=tk.W, pady=8)
        self.z_var = tk.StringVar(value="0.0")
        z_entry = ttk.Entry(main_frame, textvariable=self.z_var, width=18, 
                            font=('Arial', 12))
        z_entry.grid(row=3, column=1, sticky=tk.E, pady=8, padx=(15, 0))

        # Yaw input
        ttk.Label(main_frame, text="Yaw (Degrees):", font=('Arial', 12)).grid(
            row=4, column=0, sticky=tk.W, pady=8)
        self.yaw_var = tk.StringVar(value="0.0")
        yaw_entry = ttk.Entry(main_frame, textvariable=self.yaw_var, width=18, 
                            font=('Arial', 12))
        yaw_entry.grid(row=4, column=1, sticky=tk.E, pady=8, padx=(15, 0))
        
        # Status label
        self.status_var = tk.StringVar(value="Ready to publish")
        status_label = ttk.Label(main_frame, textvariable=self.status_var, 
                                 font=('Arial', 11), foreground='green')
        status_label.grid(row=5, column=0, columnspan=2, pady=(15, 10))
        
        # Publish button
        publish_btn = ttk.Button(main_frame, text="Publish Initial Pose & Activate MOLA", 
                                 command=self.publish_pose, width=30)
        publish_btn.grid(row=6, column=0, columnspan=2, pady=(5, 20))
        
        # Separator
        separator = ttk.Separator(main_frame, orient='horizontal')
        separator.grid(row=7, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=15)
        
        # Planar Motion Section
        planar_title = ttk.Label(main_frame, text="Motion Constraints", 
                                 font=('Arial', 14, 'bold'))
        planar_title.grid(row=8, column=0, columnspan=2, pady=(0, 15))
        
        # Planar motion status
        self.planar_status_var = tk.StringVar(value="Planar mode: Initializing...")
        planar_status_label = ttk.Label(main_frame, textvariable=self.planar_status_var, 
                                        font=('Arial', 10), foreground='blue')
        planar_status_label.grid(row=9, column=0, columnspan=2, pady=(0, 10))
        
        # Planar motion buttons frame
        planar_btn_frame = ttk.Frame(main_frame)
        planar_btn_frame.grid(row=10, column=0, columnspan=2, pady=(0, 15))
        
        enable_planar_btn = ttk.Button(planar_btn_frame, text="Enable Planar Mode", 
                                       command=self.enable_planar, width=20)
        enable_planar_btn.pack(side=tk.LEFT, padx=5)
        
        disable_planar_btn = ttk.Button(planar_btn_frame, text="Disable Planar Mode", 
                                        command=self.disable_planar, width=20)
        disable_planar_btn.pack(side=tk.LEFT, padx=5)
        
        # Info frame
        info_frame = ttk.LabelFrame(main_frame, text="Configuration Info", padding="15")
        info_frame.grid(row=11, column=0, columnspan=2, pady=(10, 0), sticky=(tk.W, tk.E))
        
        info_lines = [
            "Initial Pose:",
            "  • Topic: /initialpose",
            "  • Frame ID: map",
            "  • QoS: Best Effort",
            "",
            "MOLA Activation:",
            "  • active: true",
            "  • mapping_enabled: false",
            "  • generate_simplemap: false",
            "",
            "Planar Mode:",
            "  • Auto-enabled at startup",
            "  • Constrains Z, pitch, roll to zero",
            "  • Use for flat ground robots",
            "  • Disable for 3D motion/uneven terrain"
        ]
        
        for line in info_lines:
            label = ttk.Label(info_frame, text=line, font=('Arial', 9))
            label.pack(anchor=tk.W, pady=1)
        
        # Configure grid
        main_frame.columnconfigure(1, weight=1)
        
        # Bind Enter key to publish
        self.window.bind('<Return>', lambda e: self.publish_pose())
    
    def update_graphs(self, frame):
        """Update all graphs with new data"""
        rclpy.spin_once(self.node, timeout_sec=0.001)
        x, y, z, roll, pitch, yaw, icp_hist = self.node.get_data()

        # XY plane 
        if len(x) > 0:
            self.line_xy.set_data(x, y)
            self.axes[0, 0].relim()
            self.axes[0, 0].autoscale_view()

            # Combined XZ + YZ 
            self.line_xz.set_data(x, z)
            self.line_yz.set_data(y, z)
            self.axes[0, 1].relim()
            self.axes[0, 1].autoscale_view()

        # ICP Quality plot
        if len(icp_hist) > 0: 
            self.line_quality.set_data(np.arange(len(icp_hist)), icp_hist)
            self.axes[0, 2].relim()
            self.axes[0, 2].autoscale_view()

        # Orientation (RPY)
        ori_data = [roll, pitch, yaw]
        for i, line in enumerate(self.ori_lines):
            if len(ori_data[i]) > 0:
                line.set_data(np.arange(len(ori_data[i])), ori_data[i])
                self.axes[1, i].relim()
                self.axes[1, i].autoscale_view()

        return [self.line_xy, self.line_xz, self.line_yz, self.line_quality, *self.ori_lines]
    
    def start_animation(self):
        """Start the matplotlib animation"""
        self.ani = FuncAnimation(self.fig, self.update_graphs, 
                                 blit=False, interval=100, cache_frame_data=False)
    
    def clear_trajectory(self):
        """Clear trajectory data"""
        self.node.clear_trajectory()
        # Clear the plots
        self.line_xy.set_data([], [])
        self.line_xz.set_data([], [])
        self.line_yz.set_data([], [])
        self.line_quality.set_data([], [])
        for line in self.ori_lines:
            line.set_data([], [])
        self.fig.canvas.draw()
    
    def publish_pose(self):
        """Publish initial pose and activate MOLA"""
        try:
            x = float(self.x_var.get())
            y = float(self.y_var.get())
            z = float(self.z_var.get())
            yaw = float(self.yaw_var.get())
            
            # This will activate MOLA and publish the pose
            success = self.node.publish_initial_pose(x, y, z, yaw)
            
            if success:
                self.status_var.set(f"✓ Published & MOLA activated: X={x}, Y={y}, Z={z}, Yaw={yaw}°")
            else:
                self.status_var.set(f"⚠ Published but MOLA service issue: X={x}, Y={y}, Z={z}, Yaw={yaw}°")
            
            self.window.after(4000, lambda: self.status_var.set("Ready to publish"))
            
        except ValueError:
            self.status_var.set("⚠ Error: Invalid number format")
            self.window.after(3000, lambda: self.status_var.set("Ready to publish"))
    
    def initialize_planar_mode(self):
        """Initialize planar motion constraint at startup"""
        self.planar_status_var.set("Planar mode: Enabling...")
        success = self.node.set_enforce_planar_motion(True)
        if success:
            self.planar_status_var.set("Planar mode: ENABLED ✓ (Auto-initialized)")
        else:
            self.planar_status_var.set("Planar mode: Auto-enable failed ⚠")
        self.window.after(3000, lambda: self.planar_status_var.set("Planar mode: ENABLED" if success else "Planar mode: Unknown"))
    
    def enable_planar(self):
        """Enable planar motion constraint"""
        success = self.node.set_enforce_planar_motion(True)
        if success:
            self.planar_status_var.set("Planar mode: ENABLED ✓")
        else:
            self.planar_status_var.set("Planar mode: Failed to enable ⚠")
        self.window.after(3000, lambda: self.planar_status_var.set("Planar mode: Check logs"))
    
    def disable_planar(self):
        """Disable planar motion constraint"""
        success = self.node.set_enforce_planar_motion(False)
        if success:
            self.planar_status_var.set("Planar mode: DISABLED ✓")
        else:
            self.planar_status_var.set("Planar mode: Failed to disable ⚠")
        self.window.after(3000, lambda: self.planar_status_var.set("Planar mode: Check logs"))
    
    def on_closing(self):
        """Handle window closing"""
        if self.ani:
            self.ani.event_source.stop()
        plt.close(self.fig)
        self.window.quit()
        self.window.destroy()
    
    def run(self):
        """Run the GUI main loop"""
        self.window.mainloop()


def ros_spin(node):
    """Spin ROS node in a separate thread"""
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    
    # Start ROS spinning in a separate thread
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()
    
    # Run GUI in main thread
    gui = TrajectoryGUI(node)
    
    try:
        gui.run()
    except KeyboardInterrupt:
        print("\n[TrajectoryPlotter] Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()