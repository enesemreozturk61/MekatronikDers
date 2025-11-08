#!/usr/bin/env python3
"""
DANS 2 SCRİPTİ (DALGA)
=====================
Robot dalga efekti ile dans eder.

Hızlandırılmış versiyon: 0.2 saniye/adım
Geliştirici: OZTURK
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math


class Dance1Control(Node):
    """Dans 2 kontrolcüsü."""
    
    def __init__(self):
        """Başlangıç ayarları."""
        super().__init__('dance1')
        
        # Her bacak için publisher
        self.leg_publishers = {}
        for leg_num in range(1, 7):
            topic = f'/leg{leg_num}_controller/joint_trajectory'
            self.leg_publishers[leg_num] = self.create_publisher(
                JointTrajectory, topic, 10
            )
        
        self.get_logger().info('✅ Dans 2 başlatıldı - Dalga (HIZ: 0.2s)')
        
        # Dans parametreleri
        self.step_duration = 0.2    # HIZLANDIRILDI! (önceden 0.3)
        self.step_count = 0
        self.max_steps = 16
        
        # Zamanlayıcı
        self.timer = self.create_timer(self.step_duration, self.wave_dance)
    
    def create_trajectory(self, leg_num, positions, duration):
        """Trajectory mesajı oluştur."""
        traj = JointTrajectory()
        traj.joint_names = [
            f'joint1_{leg_num}',
            f'joint2_{leg_num}',
            f'joint3_{leg_num}'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9)
        )
        
        traj.points.append(point)
        return traj
    
    def wave_dance(self):
        """Dalga dans adımı."""
        
        if self.step_count >= self.max_steps:
            self.get_logger().info('✅ Dans tamamlandı')
            self.timer.cancel()
            return
        
        # Her bacak sinüs dalgası ile hareket eder
        for leg_num in range(1, 7):
            phase = (self.step_count + leg_num) * (math.pi / 4)
            
            height = 0.15 + 0.3 * math.sin(phase)
            swing = 0.2 * math.cos(phase)
            
            pos = [swing, -height, 1.2]
            traj = self.create_trajectory(leg_num, pos, self.step_duration)
            self.leg_publishers[leg_num].publish(traj)
        
        self.get_logger().info(f'🌊 Dalga adımı: {self.step_count}')
        self.step_count += 1


def main(args=None):
    """Ana fonksiyon."""
    rclpy.init(args=args)
    node = Dance1Control()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
