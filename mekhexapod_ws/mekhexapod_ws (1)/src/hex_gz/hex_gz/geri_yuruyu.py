#!/usr/bin/env python3
"""
GERİ YÜRÜYÜŞ HAREKET SCRİPTİ (BİPOD GAİT)
========================================
Hexapod robotun geriye doğru yürümesini sağlayan Bipod (ikiayak) algoritması.

Algoritma Prensibi:
    Sol Grup: Bacak 1, 3, 5 (Sol taraf)
    Sağ Grup: Bacak 2, 4, 6 (Sağ taraf)
    
    Bir taraftaki 3 bacak birlikte hareket eder.

Orijinal: bi_gate.py
Türkçeleştiren: [Öğrenci Adı]
Tarih: Ekim 2025
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class GeriYuruyusKontrol(Node):
    """
    Geri yürüyüş hareketini kontrol eden ROS2 node sınıfı.
    
    Bipod gait algoritması kullanarak robotun geriye yürümesini sağlar.
    """
    
    def __init__(self):
        """Node'u başlatır ve publisher'ları oluşturur."""
        super().__init__('geri_yuruyu_node')
        
        # Her bacak için publisher
        self.bacak_yayinlayicilar = {}
        for bacak_no in range(1, 7):
            topic_ismi = f'/leg{bacak_no}_controller/joint_trajectory'
            self.bacak_yayinlayicilar[bacak_no] = self.create_publisher(
                JointTrajectory,
                topic_ismi,
                10
            )
        
        self.get_logger().info('Geri yürüyüş sistemi başlatıldı')
        
        # Hareket parametreleri
        self.adim_uzunlugu = 0.25
        self.adim_yuksekligi = 0.35
        self.hareket_suresi = 0.6
        
        # Hareket döngüsünü başlat
        self.timer = self.create_timer(self.hareket_suresi, self.bipod_yuruyu)
        self.faz = 0  # Sol (0) veya Sağ (1)
    
    def eklem_komutu_olustur(self, bacak_no, konum_listesi, sure):
        """
        Bacak için trajectory komutu oluşturur.
        
        Args:
            bacak_no (int): Bacak numarası
            konum_listesi (list): Joint açıları
            sure (float): Süre
        """
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
    
    def bipod_yuruyu(self):
        """
        Bipod gait algoritmasını uygular.
        
        Faz 0: Sol bacaklar (1,3,5) hareket eder
        Faz 1: Sağ bacaklar (2,4,6) hareket eder
        """
        
        if self.faz == 0:
            # SOL BACAKLAR (1, 3, 5) - Geriye hareket
            for bacak_no in [1, 3, 5]:
                konum = [
                    -self.adim_uzunlugu,     # Geriye
                    -self.adim_yuksekligi,   # Yukarı
                    1.2
                ]
                trajectory = self.eklem_komutu_olustur(
                    bacak_no, konum, self.hareket_suresi
                )
                self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
            
            # SAĞ BACAKLAR (2, 4, 6) - Yerde kalır
            for bacak_no in [2, 4, 6]:
                konum = [
                    self.adim_uzunlugu,      # Öne (itme)
                    0.15,
                    1.4
                ]
                trajectory = self.eklem_komutu_olustur(
                    bacak_no, konum, self.hareket_suresi
                )
                self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
            
            self.get_logger().info('Faz 0: Sol bacaklar geriye')
            self.faz = 1
        
        else:
            # SAĞ BACAKLAR (2, 4, 6) - Geriye hareket
            for bacak_no in [2, 4, 6]:
                konum = [
                    -self.adim_uzunlugu,
                    -self.adim_yuksekligi,
                    1.2
                ]
                trajectory = self.eklem_komutu_olustur(
                    bacak_no, konum, self.hareket_suresi
                )
                self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
            
            # SOL BACAKLAR (1, 3, 5) - Yerde kalır
            for bacak_no in [1, 3, 5]:
                konum = [
                    self.adim_uzunlugu,
                    0.15,
                    1.4
                ]
                trajectory = self.eklem_komutu_olustur(
                    bacak_no, konum, self.hareket_suresi
                )
                self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
            
            self.get_logger().info('Faz 1: Sağ bacaklar geriye')
            self.faz = 0


def main(args=None):
    """Ana fonksiyon."""
    rclpy.init(args=args)
    geri_yuruyu_node = GeriYuruyusKontrol()
    
    try:
        rclpy.spin(geri_yuruyu_node)
    except KeyboardInterrupt:
        pass
    finally:
        geri_yuruyu_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
