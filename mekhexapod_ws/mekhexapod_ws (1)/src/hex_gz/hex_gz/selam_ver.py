#!/usr/bin/env python3
"""
SELAM VERME HAREKET SCRİPTİ
===========================
Robot ön sağ bacağını kaldırıp sallayarak selam verir.

Orijinal: hi.py
Türkçeleştiren: [Öğrenci Adı]
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class SelamVerKontrol(Node):
    """Selam verme hareketini kontrol eder."""
    
    def __init__(self):
        """Node'u başlatır."""
        super().__init__('selam_ver_node')
        
        # Sadece bacak 2 (ön sağ) için publisher
        self.bacak2_yayinlayici = self.create_publisher(
            JointTrajectory,
            '/leg2_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info('Selam verme başlatıldı')
        
        # Hareket parametreleri
        self.hareket_suresi = 0.5
        self.adim = 0
        
        # Selam hareketini başlat
        self.timer = self.create_timer(self.hareket_suresi, self.selam_ver)
    
    def eklem_komutu_olustur(self, konum_listesi, sure):
        """Bacak 2 için trajectory oluşturur."""
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1_2', 'joint2_2', 'joint3_2']
        
        nokta = JointTrajectoryPoint()
        nokta.positions = konum_listesi
        nokta.time_from_start = Duration(
            sec=int(sure),
            nanosec=int((sure % 1) * 1e9)
        )
        
        trajectory.points.append(nokta)
        return trajectory
    
    def selam_ver(self):
        """
        Selam verme hareketi.
        
        Adım 0: Bacağı kaldır
        Adım 1: Sağa doğru salla
        Adım 2: Sola doğru salla
        Adım 3: Sağa doğru salla
        Adım 4: Normal pozisyona dön
        """
        
        if self.adim == 0:
            # Bacağı kaldır
            konum = [0.0, -0.5, 1.0]
            self.get_logger().info('Bacak kaldırılıyor...')
        
        elif self.adim == 1:
            # Sağa salla
            konum = [0.4, -0.5, 1.0]
            self.get_logger().info('Sağa sallanıyor...')
        
        elif self.adim == 2:
            # Sola salla
            konum = [-0.4, -0.5, 1.0]
            self.get_logger().info('Sola sallanıyor...')
        
        elif self.adim == 3:
            # Tekrar sağa
            konum = [0.4, -0.5, 1.0]
            self.get_logger().info('Tekrar sağa...')
        
        else:
            # Normal pozisyon
            konum = [0.0, 0.15, 1.4]
            self.get_logger().info('Normal pozisyon - Selam tamamlandı!')
            self.timer.cancel()  # Hareketi bitir
        
        # Komutu gönder
        trajectory = self.eklem_komutu_olustur(konum, self.hareket_suresi)
        self.bacak2_yayinlayici.publish(trajectory)
        
        self.adim += 1


def main(args=None):
    """Ana fonksiyon."""
    rclpy.init(args=args)
    selam_node = SelamVerKontrol()
    
    try:
        rclpy.spin(selam_node)
    except KeyboardInterrupt:
        pass
    finally:
        selam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
