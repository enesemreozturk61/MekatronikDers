#!/usr/bin/env python3
"""
DANS 1 SCRİPTİ
=============
Robot ritmik dans eder.

Hızlandırılmış versiyon: 0.25 saniye/adım
Geliştirici: OZTURK
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class DanceControl(Node):
    """Dans 1 kontrolcüsü."""
    
    def __init__(self):
        """Başlangıç ayarları."""
        super().__init__('dance')
        
        # Her bacak için publisher
        self.leg_publishers = {}
        for leg_num in range(1, 7):
            topic = f'/leg{leg_num}_controller/joint_trajectory'
            self.leg_publishers[leg_num] = self.create_publisher(
                JointTrajectory, topic, 10
            )
        
        self.get_logger().info('✅ Dans 1 başlatıldı (HIZ: 0.25s)')
        
        # Dans parametreleri
        self.step_duration = 0.25    # HIZLANDIRILDI! (önceden 0.4)
        self.step_count = 0
        self.max_steps = 12
        
        # Zamanlayıcı
        self.timer = self.create_timer(self.step_duration, self.dance_step)
    
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
    
    def dance_step(self):
        """Dans adımı."""
        
        if self.step_count >= self.max_steps:
            self.get_logger().info('✅ Dans tamamlandı')
            self.timer.cancel()
            return
        
        pattern = self.step_count % 4
        
        if pattern == 0:
            # Tüm bacaklar yukarı
            for leg in range(1, 7):
                pos = [0.0, -0.3, 1.0]
                traj = self.create_trajectory(leg, pos, self.step_duration)
                self.leg_publishers[leg].publish(traj)
            self.get_logger().info(f'💃 Adım {self.step_count}: Yukarı')
        
        elif pattern == 1:
            # Sol yukarı, sağ aşağı
            for leg in [1, 3, 5]:
                pos = [0.0, -0.5, 0.8]
                traj = self.create_trajectory(leg, pos, self.step_duration)
                self.leg_publishers[leg].publish(traj)
            for leg in [2, 4, 6]:
                pos = [0.0, 0.2, 1.5]
                traj = self.create_trajectory(leg, pos, self.step_duration)
                self.leg_publishers[leg].publish(traj)
            self.get_logger().info(f'💃 Adım {self.step_count}: Sol yukarı')
        
        elif pattern == 2:
            # Tüm bacaklar aşağı
            for leg in range(1, 7):
                pos = [0.0, 0.15, 1.4]
                traj = self.create_trajectory(leg, pos, self.step_duration)
                self.leg_publishers[leg].publish(traj)
            self.get_logger().info(f'💃 Adım {self.step_count}: Aşağı')
        
        else:
            # Sağ yukarı, sol aşağı
            for leg in [2, 4, 6]:
                pos = [0.0, -0.5, 0.8]
                traj = self.create_trajectory(leg, pos, self.step_duration)
                self.leg_publishers[leg].publish(traj)
            for leg in [1, 3, 5]:
                pos = [0.0, 0.2, 1.5]
                traj = self.create_trajectory(leg, pos, self.step_duration)
                self.leg_publishers[leg].publish(traj)
            self.get_logger().info(f'💃 Adım {self.step_count}: Sağ yukarı')
        
        self.step_count += 1


def main(args=None):
    """Ana fonksiyon."""
    rclpy.init(args=args)
    node = DanceControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
