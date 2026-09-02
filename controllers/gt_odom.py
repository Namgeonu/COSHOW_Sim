#!/usr/bin/env python3
"""COSHOW LIMO GT(Ground-Truth) odom 플러그인.

드론 cf_node 와 동일한 패턴.
webots_ros2_control(바퀴 구동) 옆에 이 커스텀 플러그인을 함께 붙여,
plugin 내부에서 getDevice 로 gps(위치) + inertial_unit(방향)을 직접 읽어
계산 없이 정확한 odom + odom->base_link TF 를 발행한다.

InertialUnit 은 quaternion 을 그대로 주므로 방향 계산이 전혀 없음.
"""

import math

import rclpy
from builtin_interfaces.msg import Time as RosTime
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


class GTOdom:
    """webots_ros2_driver 플러그인 진입점 (init/step)."""

    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.timestep = int(self.robot.getBasicTimeStep())
        self.robot_name = self.robot.getName()   # "limo_a"

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node('gt_odom', namespace=self.robot_name)

        # --- 센서 (드론 cf_node 와 동일한 getDevice 방식) ---
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.imu = self.robot.getDevice('imu inertial')   # LIMO PROTO 의 InertialUnit 이름
        self.imu.enable(self.timestep)
        self.gyro = self.robot.getDevice('imu gyro')      # 각속도(twist.angular)용
        self.gyro.enable(self.timestep)

        # --- 발행 ---
        self.odom_pub = self.node.create_publisher(Odometry, f'/{self.robot_name}/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self.node)

        self.node.get_logger().info(
            f'[{self.robot_name}] GT odom 플러그인 init OK '
            f'(gps + inertial_unit -> /odom, odom->base_link TF)')

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        # --- 위치 (GPS 원시값, 계산·보정 없음) ---
        # proto/월드에서 로봇을 +x 정면으로 바로 배치했으므로 GPS 좌표가 곧 map 좌표.
        x, y, _z = self.gps.getValues()
        # --- 방향 (InertialUnit quaternion 원시값, 계산·보정 없음) ---
        # base_link 가 월드와 정렬돼 있어 quaternion 을 그대로 사용.
        q = self.imu.getQuaternion()             # [x, y, z, w]

        # --- 속도 (twist, base_link 기준) ---
        # GPS getSpeedVector 는 월드 좌표계 속도 → 로봇 heading(yaw)에 투영해 base_link 전진속도로.
        # 차동구동이라 옆이동(linear.y)=0, 각속도는 gyro z 그대로.
        vx_w, vy_w, _vz = self.gps.getSpeedVector()
        yaw = math.atan2(2.0 * q[3] * q[2], 1.0 - 2.0 * q[2] * q[2])
        v_forward = vx_w * math.cos(yaw) + vy_w * math.sin(yaw)
        wz = self.gyro.getValues()[2]

        # Webots 시뮬 시각으로 스탬프 (Ros2Supervisor의 /clock 과 동일 기준).
        # get_clock().now() 는 use_sim_time 미적용 노드라 wall time 이 되어
        # sim time 을 쓰는 Nav2 와 TF 시각이 어긋남 -> robot.getTime() 로 통일.
        _t = self.robot.getTime()
        now = RosTime(sec=int(_t), nanosec=int((_t - int(_t)) * 1e9))

        # --- Odometry ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = f'{self.robot_name}/odom'
        odom.child_frame_id = f'{self.robot_name}/base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        # twist: base_link 기준 전진속도 + 각속도 (차동구동: linear.y=0)
        odom.twist.twist.linear.x = v_forward
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        # --- odom -> base_link TF ---
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = f'{self.robot_name}/odom'
        tf.child_frame_id = f'{self.robot_name}/base_link'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.tf_broadcaster.sendTransform(tf)
