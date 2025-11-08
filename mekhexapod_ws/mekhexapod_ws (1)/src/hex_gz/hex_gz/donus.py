#!/usr/bin/env python3
"""
DÖNÜŞ HAREKET SCRİPTİ (ROTATION)
================================
Hexapod robotun kendi etrafında dönmesini sağlar.

Prensip:
    Robot merkez noktası etrafında döner (in-place rotation).
    Tüm bacaklar koordineli şekilde dönüş yönünde hareket eder.

Orijinal: rotation.py
Türkçeleştiren: [Öğrenci Adı]
Tarih: Ekim 2025
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math


class DonusKontrol(Node):
    """
    Dönüş hareketini kontrol eden ROS2 node sınıfı.
    """
    
    def __init__(self):
        """Node'u başlatır."""
        super().__init__('donus_node')
        
        # Publisher'ları oluştur
        self.bacak_yayinlayicilar = {}
        for bacak_no in range(1, 7):
            topic_ismi = f'/leg{bacak_no}_controller/joint_trajectory'
            self.bacak_yayinlayicilar[bacak_no] = self.create_publisher(
                JointTrajectory,
                topic_ismi,
                10
            )
        
        self.get_logger().info('Dönüş sistemi başlatıldı')
        
        # Dönüş parametreleri
        self.donus_acisi = 0.35  # Her adımda dönüş açısı (radyan)
        self.hareket_suresi = 1.0  # Hareket süresi
        
        # Dönüş hareketini başlat
        self.timer = self.create_timer(self.hareket_suresi, self.don)
        self.yon = 1  # 1: sağa, -1: sola (A/D tuşuna göre değişir)
    
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
    
    def don(self):
        """
        Tüm bacakları koordineli şekilde döndürür.
        
        Her bacak dönüş yönünde uygun açıyla konumlandırılır.
        """
        
        # Her bacak için dönüş açısını hesapla ve uygula
        for bacak_no in range(1, 7):
            # Bacak pozisyonuna göre dönüş açısı ayarla
            if bacak_no in [1, 3, 5]:  # Sol bacaklar
                aci = self.donus_acisi * self.yon
            else:  # Sağ bacaklar
                aci = -self.donus_acisi * self.yon
            
            # Dönüş pozisyonu
            konum = [
                aci,      # Coxa: dönüş açısı
                0.15,     # Femur: hafif yukarı
                1.4       # Tibia: normal
            ]
            
            trajectory = self.eklem_komutu_olustur(
                bacak_no, konum, self.hareket_suresi
            )
            self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
        
        yon_text = "sağa" if self.yon > 0 else "sola"
        self.get_logger().info(f'Robot {yon_text} dönüyor')


def main(args=None):
    """Ana fonksiyon."""
    rclpy.init(args=args)
    donus_node = DonusKontrol()
    
    try:
        rclpy.spin(donus_node)
    except KeyboardInterrupt:
        pass
    finally:
        donus_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
