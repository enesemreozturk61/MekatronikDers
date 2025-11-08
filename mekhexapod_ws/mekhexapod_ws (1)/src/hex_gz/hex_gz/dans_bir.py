#!/usr/bin/env python3
"""
DANS 1 HAREKET SCRİPTİ
=====================
Robot ritmik bacak hareketleri ile dans eder.

Orijinal: dance.py
Türkçeleştiren: [Öğrenci Adı]
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class DansBirKontrol(Node):
    """Dans 1 hareketini kontrol eder."""
    
    def __init__(self):
        """Node'u başlatır."""
        super().__init__('dans_bir_node')
        
        # Tüm bacaklar için publisher
        self.bacak_yayinlayicilar = {}
        for bacak_no in range(1, 7):
            topic_ismi = f'/leg{bacak_no}_controller/joint_trajectory'
            self.bacak_yayinlayicilar[bacak_no] = self.create_publisher(
                JointTrajectory,
                topic_ismi,
                10
            )
        
        self.get_logger().info('Dans 1 başlatıldı')
        
        # Dans parametreleri
        self.hareket_suresi = 0.4
        self.adim = 0
        self.maksimum_adim = 12  # Toplam dans adımı
        
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
        
        Her adımda farklı bacak grupları hareket eder.
        """
        
        if self.adim >= self.maksimum_adim:
            self.get_logger().info('Dans tamamlandı!')
            self.timer.cancel()
            return
        
        # Adım numarasına göre hangi bacakların hareket edeceği
        if self.adim % 4 == 0:
            # Tüm bacaklar yukarı
            for bacak_no in range(1, 7):
                konum = [0.0, -0.3, 1.0]
                trajectory = self.eklem_komutu_olustur(
                    bacak_no, konum, self.hareket_suresi
                )
                self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
            self.get_logger().info(f'Adım {self.adim}: Tüm bacaklar yukarı')
        
        elif self.adim % 4 == 1:
            # Sol bacaklar yukarı, sağ bacaklar aşağı
            for bacak_no in [1, 3, 5]:  # Sol
                konum = [0.0, -0.5, 0.8]
                trajectory = self.eklem_komutu_olustur(
                    bacak_no, konum, self.hareket_suresi
                )
                self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
            
            for bacak_no in [2, 4, 6]:  # Sağ
                konum = [0.0, 0.2, 1.5]
                trajectory = self.eklem_komutu_olustur(
                    bacak_no, konum, self.hareket_suresi
                )
                self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
            self.get_logger().info(f'Adım {self.adim}: Sol yukarı, sağ aşağı')
        
        elif self.adim % 4 == 2:
            # Tüm bacaklar aşağı
            for bacak_no in range(1, 7):
                konum = [0.0, 0.15, 1.4]
                trajectory = self.eklem_komutu_olustur(
                    bacak_no, konum, self.hareket_suresi
                )
                self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
            self.get_logger().info(f'Adım {self.adim}: Tüm bacaklar aşağı')
        
        else:
            # Sağ bacaklar yukarı, sol bacaklar aşağı
            for bacak_no in [2, 4, 6]:  # Sağ
                konum = [0.0, -0.5, 0.8]
                trajectory = self.eklem_komutu_olustur(
                    bacak_no, konum, self.hareket_suresi
                )
                self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
            
            for bacak_no in [1, 3, 5]:  # Sol
                konum = [0.0, 0.2, 1.5]
                trajectory = self.eklem_komutu_olustur(
                    bacak_no, konum, self.hareket_suresi
                )
                self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
            self.get_logger().info(f'Adım {self.adim}: Sağ yukarı, sol aşağı')
        
        self.adim += 1


def main(args=None):
    """Ana fonksiyon."""
    rclpy.init(args=args)
    dans_node = DansBirKontrol()
    
    try:
        rclpy.spin(dans_node)
    except KeyboardInterrupt:
        pass
    finally:
        dans_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
