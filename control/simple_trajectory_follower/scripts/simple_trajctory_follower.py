import rclpy
import tf2_ros
import tf2_py
import numpy as np

from rclpy.node import Node

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_planning_msgs.msg import TrajectoryPoint
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from tier4_planning_msgs.msg import VelocityLimit


class SimpleTrajectoryFollower(Node):

    def __init__(self):
        super().__init__('simple_trajectory_follower')
        self.pub_cmd = self.create_publisher(AckermannControlCommand, '/simulation/input/manual_ackermann_control_command', 1)
        self.sub_traj = self.create_subscription(Trajectory, '/planning/scenario_planning/trajectory', self.on_trajectory, 1)
        self.sub_ego = self.create_subscription(Odometry, '/localization/kinematics', self.on_kinematics, 1)
        self.timer = self.create_timer(0.1 , self.timer_callback) # 0.1 sec
        self.ego_pose = Pose()
        self.ego_twist = Twist()
        self.trajectory = Trajectory()
        self.target_point = TrajectoryPoint()

    def on_trajectory(self, msg):
        self.trajectory = msg

    def on_kinematics(self, msg):
        self.ego_pose = msg.pose
        self.ego_twist = msg.twist

    def timer_callback(self):
        self.get_logger().info("timer run")

        if len(self.trajectory.points) == 0:
            return

        self.calc_target_point()

        lat_err, yaw_err, vel_err = self.calc_errors()
        self.get_logger().info("error: lat={}, yaw={}, vel={}".format(lat_err, yaw_err, vel_err))

        cmd = AckermannControlCommand()
        cmd.stamp = self.get_clock().now().to_msg()
        cmd.lateral.steering_tire_angle = self.calc_steer_feedback(lat_err, yaw_err)
        cmd.lateral.steering_tire_rotation_rate = 0.0
        cmd.longitudinal.speed = self.target_point.longitudinal_velocity_mps
        cmd.longitudinal.acceleration = self.calc_vel_feedback(vel_err)

        self.get_logger().info("cmd: steer={}, speed={}, acc={}".format(cmd.lateral.steering_tire_angle, cmd.longitudinal.speed, cmd.longitudinal.acceleration))

        self.pub_cmd.publish(cmd)

    def calc_steer_feedback(self, lat_err, yaw_err):
        kp = 1.0
        kd = 0.5
        fb_steer = -kp * lat_err - kd * yaw_err
        return fb_steer

    def calc_vel_feedback(self, vel_err):
        kp = 1.0
        return -kp * vel_err

    def calc_target_point(self):
        closest = self.calc_closest_trajectory(self.trajectory)
        self.target_point = self.trajectory.points[closest]

    def calc_errors(self):

        # lateral error
        lat_err = self.calc_dist2d(self.target_point.pose, self.ego_pose)

        # yaw error
        ego_yaw = self.to_yaw(self.ego_pose.orientation)
        target_yaw = self.to_yaw(self.target_point.pose.orientation)
        yaw_err = target_yaw - ego_yaw
        yaw_err = np.arctan2(np.sin(yaw_err), np.cos(yaw_err))

        # vel_err
        vel_err = self.target_point.lateral_velocity_mps - self.ego_twist.linear.x

        return lat_err, yaw_err, vel_err


    def to_yaw(self, q):
        roll, pitch, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
        return yaw

    def calc_closest_trajectory(self, path):
        closest = -1
        min_dist_squared = 1.0e10
        for i in range(0, len(path.points)):
            dist_sq = self.calc_squared_dist2d(self.ego_pose, path.points[i].pose)
            if dist_sq < min_dist_squared:
                min_dist_squared = dist_sq
                closest = i
        return closest

    def calc_dist2d(self, p1, p2):
        return np.sqrt(self.calc_squared_dist2d(p1, p2))

    def calc_squared_dist2d(self, p1, p2):
        dx = p1.position.x - p2.position.x
        dy = p1.position.y - p2.position.y
        return dx * dx + dy * dy

    def euler_from_quaternion(self, x,y,z,w):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)

    node = SimpleTrajectoryFollower()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
