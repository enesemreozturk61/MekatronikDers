#!/usr/bin/env python3
"""
SELAM VERME SCRİPTİ
==================
Robot ön sağ bacağını sallayarak selam verir.

Hızlandırılmış versiyon: 0.3 saniye/adım
Geliştirici: OZTURK
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class HiControl(Node):
    """Selam verme kontrolcüsü."""
    
    def __init__(self):
        """Başlangıç ayarları."""
        super().__init__('hi')
        
        # Sadece bacak 2 için publisher
        self.leg2_publisher = self.create_publisher(
            JointTrajectory,
            '/leg2_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info('✅ Selam verme başlatıldı (HIZ: 0.3s)')
        
        # Hareket parametreleri
        self.step_duration = 0.3    # HIZLANDIRILDI! (önceden 0.5)
        self.step_count = 0
        self.max_steps = 5
        
        # Zamanlayıcı
        self.timer = self.create_timer(self.step_duration, self.wave)
    
    def create_trajectory(self, positions, duration):
        """Trajectory mesajı oluştur."""
        traj = JointTrajectory()
        traj.joint_names = ['joint1_2', 'joint2_2', 'joint3_2']
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9)
        )
        
        traj.points.append(point)
        return traj
    
    def wave(self):
        """Selam adımı."""
        
        if self.step_count == 0:
            pos = [0.0, -0.5, 1.0]
            self.get_logger().info('👋 Bacak kaldırılıyor')
        elif self.step_count == 1:
            pos = [0.4, -0.5, 1.0]
            self.get_logger().info('→ Sağa')
        elif self.step_count == 2:
            pos = [-0.4, -0.5, 1.0]
            self.get_logger().info('← Sola')
        elif self.step_count == 3:
            pos = [0.4, -0.5, 1.0]
            self.get_logger().info('→ Tekrar sağa')
        else:
            pos = [0.0, 0.15, 1.4]
            self.get_logger().info('✅ Selam tamamlandı')
            self.timer.cancel()
        
        traj = self.create_trajectory(pos, self.step_duration)
        self.leg2_publisher.publish(traj)
        
        self.step_count += 1


def main(args=None):
    """Ana fonksiyon."""
    rclpy.init(args=args)
    node = HiControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
