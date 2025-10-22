#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from postech_flipper_msgs.msg import FlipperVelocityCommand, FlipperAngleCommand, FlipperAngleState
import numpy as np
from collections import deque
import time

class FlipperVelocityIntegrator(Node):
    """
    Flipper velocity를 적분하여 angle position으로 변환하는 노드
    
    입력: /postech/flipper_velocity_command (rad/s)
    출력: /postech/flipper_angle_command (degree)
    
    안정적인 적분을 위해:
    1. 적응적 시간 스텝 사용
    2. 이동평균 필터 적용
    3. 각도 제한 및 클리핑
    """
    
    def __init__(self):
        super().__init__('flipper_velocity_integrator')
        
        # 파라미터 선언
        self.declare_parameter('input_topic', '/postech/flipper_velocity_command')
        self.declare_parameter('output_topic', '/postech/flipper_angle_command')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('angle_min_deg', -90.0)  # 최소 각도
        self.declare_parameter('angle_max_deg', 0.0)    # 최대 각도
        self.declare_parameter('filter_window', 5)      # 이동평균 윈도우 크기
        
        # 파라미터 가져오기
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.angle_min_deg = self.get_parameter('angle_min_deg').value
        self.angle_max_deg = self.get_parameter('angle_max_deg').value
        self.filter_window = self.get_parameter('filter_window').value
        
        # 상태 변수
        self.current_angles_rad = np.array([0.0, 0.0, 0.0, 0.0])  # FL, FR, RL, RR
        self.last_time = time.time()
        self.velocity_buffer = deque(maxlen=self.filter_window)
        
        # 퍼블리셔/구독자
        self.velocity_sub = self.create_subscription(
            FlipperVelocityCommand,
            self.input_topic,
            self.velocity_callback,
            10
        )
        
        # 현재 각도 상태 구독 (초기값 설정용)
        self.angle_state_sub = self.create_subscription(
            FlipperAngleState,
            '/kiro/flipper_angle_state',
            self.angle_state_callback,
            10
        )
        
        self.angle_pub = self.create_publisher(
            FlipperAngleCommand,
            self.output_topic,
            10
        )
        
        # 주기적 퍼블리시 타이머
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_angles)
        
        self.get_logger().info(f"Flipper Velocity Integrator 시작!")
        self.get_logger().info(f"  Velocity 입력: {self.input_topic} (rad/s)")
        self.get_logger().info(f"  Angle 상태 입력: /kiro/flipper_angle_state (degree)")
        self.get_logger().info(f"  출력: {self.output_topic} (degree)")
        self.get_logger().info(f"  퍼블리시 주기: {self.publish_rate} Hz")
        self.get_logger().info(f"  각도 제한: [{self.angle_min_deg}, {self.angle_max_deg}] degree")
    
    def angle_state_callback(self, msg):
        """현재 각도 상태를 받아서 적분 초기값으로 설정"""
        # degree를 radian으로 변환하여 현재 각도 업데이트
        self.current_angles_rad = np.array([
            np.radians(msg.fl),
            np.radians(msg.fr), 
            np.radians(msg.rl),
            np.radians(msg.rr)
        ])
        
        # 각도 제한 적용
        angle_min_rad = np.radians(self.angle_min_deg)
        angle_max_rad = np.radians(self.angle_max_deg)
        self.current_angles_rad = np.clip(
            self.current_angles_rad,
            angle_min_rad,
            angle_max_rad
        )
        
        # 로그 (5초마다)
        current_time = time.time()
        if not hasattr(self, 'last_angle_state_time'):
            self.last_angle_state_time = current_time
        
        if current_time - self.last_angle_state_time >= 5.0:
            self.get_logger().info(
                f"각도 상태 업데이트 - FL: {msg.fl:.1f}°, FR: {msg.fr:.1f}°, "
                f"RL: {msg.rl:.1f}°, RR: {msg.rr:.1f}°"
            )
            self.last_angle_state_time = current_time
    
    def velocity_callback(self, msg):
        """Velocity 명령을 받아서 적분"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        # 시간 간격이 너무 크면 스킵 (시스템 재시작 등)
        if dt > 1.0:
            self.last_time = current_time
            return
        
        # Velocity 명령 받기 (rad/s)
        velocities = np.array([msg.fl, msg.fr, msg.rl, msg.rr])
        
        # 이동평균 필터 적용 (노이즈 제거)
        self.velocity_buffer.append(velocities)
        if len(self.velocity_buffer) > 1:
            filtered_velocities = np.mean(list(self.velocity_buffer), axis=0)
        else:
            filtered_velocities = velocities
        
        # 적분 (Euler 방법)
        angle_deltas = filtered_velocities * dt
        self.current_angles_rad += angle_deltas
        
        # 각도 제한 적용 (radian으로 변환)
        angle_min_rad = np.radians(self.angle_min_deg)
        angle_max_rad = np.radians(self.angle_max_deg)
        
        self.current_angles_rad = np.clip(
            self.current_angles_rad,
            angle_min_rad,
            angle_max_rad
        )
        
        self.last_time = current_time
        
        # 디버그 로그 (1초마다)
        current_time = time.time()
        if not hasattr(self, 'last_debug_time'):
            self.last_debug_time = current_time
        
        if current_time - self.last_debug_time >= 1.0:
            self.get_logger().info(
                f"Velocity 적분 중 - FL: {np.degrees(self.current_angles_rad[0]):.1f}°, "
                f"FR: {np.degrees(self.current_angles_rad[1]):.1f}°, "
                f"RL: {np.degrees(self.current_angles_rad[2]):.1f}°, "
                f"RR: {np.degrees(self.current_angles_rad[3]):.1f}°"
            )
            self.last_debug_time = current_time
    
    def publish_angles(self):
        """현재 각도를 degree로 변환하여 퍼블리시"""
        angle_msg = FlipperAngleCommand()
        
        # radian을 degree로 변환
        angle_msg.fl = np.degrees(self.current_angles_rad[0])
        angle_msg.fr = np.degrees(self.current_angles_rad[1])
        angle_msg.rl = np.degrees(self.current_angles_rad[2])
        angle_msg.rr = np.degrees(self.current_angles_rad[3])
        
        self.angle_pub.publish(angle_msg)
        
        # 퍼블리시 상태 로그 (5초마다)
        current_time = time.time()
        if not hasattr(self, 'last_publish_time'):
            self.last_publish_time = current_time
        
        if current_time - self.last_publish_time >= 5.0:
            self.get_logger().info(
                f"퍼블리시 중 - /postech/flipper_angle_command: "
                f"FL={angle_msg.fl:.1f}°, FR={angle_msg.fr:.1f}°, "
                f"RL={angle_msg.rl:.1f}°, RR={angle_msg.rr:.1f}°"
            )
            self.last_publish_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = FlipperVelocityIntegrator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
