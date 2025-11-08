#!/usr/bin/env python3
"""
DANS 2 HAREKET SCRİPTİ
=====================
Robot daha karmaşık bir dans pattern'i sergiler.

Orijinal: dance1.py
Türkçeleştiren: [Öğrenci Adı]
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math


class DansIkiKontrol(Node):
    """Dans 2 hareketini kontrol eder."""
    
    def __init__(self):
        """Node'u başlatır."""
        super().__init__('dans_iki_node')
        
        # Tüm bacaklar için publisher
        self.bacak_yayinlayicilar = {}
        for bacak_no in range(1, 7):
            topic_ismi = f'/leg{bacak_no}_controller/joint_trajectory'
            self.bacak_yayinlayicilar[bacak_no] = self.create_publisher(
                JointTrajectory,
                topic_ismi,
                10
            )
        
        self.get_logger().info('Dans 2 başlatıldı')
        
        # Dans parametreleri
        self.hareket_suresi = 0.3
        self.adim = 0
        self.maksimum_adim = 16
        
        # Dans döngüsünü başlat
        self.timer = self.create_timer(self.hareket_suresi, self.dans_et)
    
    def eklem_komutu_olustur(self, bacak_no, konum_listesi, sure):
        """Trajectory komutu oluşturur."""
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            f'joint1_{bacak_no}',
            f'joint2_{bacak_no}',
            f'joint3_{bacak_no}'
        ]
        
        nokta = JointTrajectoryPoint()
        nokta.positions = konum_listesi
        nokta.time_from_start = Duration(
            sec=int(sure),
            nanosec=int((sure % 1) * 1e9)
        )
        
        trajectory.points.append(nokta)
        return trajectory
    
    def dans_et(self):
        """
        Dans hareketini gerçekleştirir.
        
        Dalga şeklinde hareket eden bacaklar.
        """
        
        if self.adim >= self.maksimum_adim:
            self.get_logger().info('Dans 2 tamamlandı!')
            self.timer.cancel()
            return
        
        # Her bacak için sinüs dalgası şeklinde hareket
        for bacak_no in range(1, 7):
            # Her bacak farklı fazda hareket eder (dalga efekti)
            faz = (self.adim + bacak_no) * (math.pi / 4)
            
            # Sinüs dalgası ile yukarı-aşağı hareket
            yukseklik = 0.15 + 0.3 * math.sin(faz)
            
            # Coxa ile sağa-sola sallanma
            sallama = 0.2 * math.cos(faz)
            
            konum = [
                sallama,      # Coxa: sağa-sola
                -yukseklik,   # Femur: yukarı-aşağı
                1.2           # Tibia: sabit
            ]
            
            trajectory = self.eklem_komutu_olustur(
                bacak_no, konum, self.hareket_suresi
            )
            self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
        
        self.get_logger().info(f'Dans adımı: {self.adim}')
        self.adim += 1


def main(args=None):
    """Ana fonksiyon."""
    rclpy.init(args=args)
    dans_node = DansIkiKontrol()
    
    try:
        rclpy.spin(dans_node)
    except KeyboardInterrupt:
        pass
    finally:
        dans_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
