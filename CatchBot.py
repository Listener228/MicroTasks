#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi
import random
from turtlesim.srv import Spawn, Kill
from rclpy.qos import QoSProfile, ReliabilityPolicy
from enum import Enum
import functools

class TargetStatus(Enum):
    DEAD = 0
    ALIVE = 1
    SPAWNING = 2
    KILLING = 3

class CatchBot(Node):

    def __init__(self):
        super().__init__('catch_two_bot')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE

        self.velocity_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_refresh, qos_profile
        )
        self.pose = Pose()

        self.cmd_vel_turtle2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.cmd_vel_turtle3 = self.create_publisher(Twist, '/turtle3/cmd_vel', 10)

        self.pose_sub_turtle2 = self.create_subscription(
            Pose, '/turtle2/pose', self.prey2_pose_callback, qos_profile
        )
        self.pose_sub_turtle3 = self.create_subscription(
            Pose, '/turtle3/pose', self.prey3_pose_callback, qos_profile
        )


        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for kill service...')


        self.targets = {
            'turtle2': {
                'pose': Pose(),
                'status': TargetStatus.DEAD,
                'angular_z': 0.0
            },
            'turtle3': {
                'pose': Pose(),
                'status': TargetStatus.DEAD,
                'angular_z': 0.0
            }
        }

        self.distance_tolerance = 0.5
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.main_loop)

        self.hero_linear_speed = 2.0

        self.get_logger().info('Catch Bot initialized – hero moves at constant speed!')

    def prey2_pose_callback(self, msg):
        self.targets['turtle2']['pose'] = msg

    def prey3_pose_callback(self, msg):
        self.targets['turtle3']['pose'] = msg

    def pose_refresh(self, data):
        self.pose = data


    def main_loop(self):

        for name in ['turtle2', 'turtle3']:
            if self.targets[name]['status'] == TargetStatus.DEAD:
                self.spawn_target(name)


        self.move_prey('turtle2')
        self.move_prey('turtle3')


        nearest_target = None
        min_dist = float('inf')
        for name in ['turtle2', 'turtle3']:
            if self.targets[name]['status'] == TargetStatus.ALIVE:
                dist = self.euclidean_distance_to_pose(self.targets[name]['pose'])
                if dist < min_dist:
                    min_dist = dist
                    nearest_target = name

        if nearest_target is None:
            return

        target_pose = self.targets[nearest_target]['pose']

        if min_dist < self.distance_tolerance:
            stop_msg = Twist()
            self.velocity_publisher.publish(stop_msg)
            self.kill_target(nearest_target)
            return

        vel_msg = Twist()
        vel_msg.linear.x = self.hero_linear_speed          
        vel_msg.angular.z = self.angular_vel(target_pose)  
        self.velocity_publisher.publish(vel_msg)

    def move_prey(self, name):
        if self.targets[name]['status'] != TargetStatus.ALIVE:
            return

        twist = Twist()
        twist.linear.x = 1.0
        if random.random() < 0.1:
            self.targets[name]['angular_z'] = random.uniform(-2.0, 2.0)
        twist.angular.z = self.targets[name]['angular_z']

        if name == 'turtle2':
            self.cmd_vel_turtle2.publish(twist)
        else:
            self.cmd_vel_turtle3.publish(twist)

    def spawn_target(self, name):
        self.targets[name]['status'] = TargetStatus.SPAWNING
        x = float(random.randrange(2, 11))
        y = float(random.randrange(2, 11))
        self.get_logger().info(f'Spawning {name} at ({x:.1f}, {y:.1f})')

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = name

        future = self.spawn_client.call_async(request)
        future.add_done_callback(
            functools.partial(self.spawn_callback, target_name=name, spawn_x=x, spawn_y=y)
        )

    def spawn_callback(self, future, target_name, spawn_x, spawn_y):
        try:
            future.result()
            self.get_logger().info(f'Successfully spawned {target_name}')
            self.targets[target_name]['pose'].x = spawn_x
            self.targets[target_name]['pose'].y = spawn_y
            self.targets[target_name]['status'] = TargetStatus.ALIVE
            self.targets[target_name]['angular_z'] = 0.0
        except Exception as e:
            self.get_logger().error(f'Spawn failed for {target_name}: {e}')
            self.targets[target_name]['status'] = TargetStatus.DEAD

    def kill_target(self, name):
        if self.targets[name]['status'] != TargetStatus.ALIVE:
            return
        self.get_logger().info(f'Killing {name}')
        self.targets[name]['status'] = TargetStatus.KILLING

        request = Kill.Request()
        request.name = name

        future = self.kill_client.call_async(request)
        future.add_done_callback(
            functools.partial(self.kill_callback, target_name=name)
        )

    def kill_callback(self, future, target_name):
        try:
            future.result()
            self.get_logger().info(f'Successfully killed {target_name}')
            self.targets[target_name]['status'] = TargetStatus.DEAD
        except Exception as e:
            self.get_logger().error(f'Kill failed for {target_name}: {e}')
            self.targets[target_name]['status'] = TargetStatus.DEAD

    def euclidean_distance_to_pose(self, target_pose):
        return sqrt(pow((target_pose.x - self.pose.x), 2) +
                    pow((target_pose.y - self.pose.y), 2))

    def steering_angle(self, target_pose):
        return atan2(target_pose.y - self.pose.y, target_pose.x - self.pose.x)

    def angular_vel(self, target_pose, constant=6.0):
        angle_diff = self.steering_angle(target_pose) - self.pose.theta
        while angle_diff > pi:
            angle_diff -= 2 * pi
        while angle_diff < -pi:
            angle_diff += 2 * pi
        return constant * angle_diff


def main(args=None):
    rclpy.init(args=args)
    catch_bot = CatchBot()
    try:
        rclpy.spin(catch_bot)
    except KeyboardInterrupt:
        pass
    finally:
        catch_bot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()