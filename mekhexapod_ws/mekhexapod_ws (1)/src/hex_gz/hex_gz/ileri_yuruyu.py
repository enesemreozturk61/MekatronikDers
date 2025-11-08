#!/usr/bin/env python3
"""
İLERİ YÜRÜYÜŞ HAREKET SCRİPTİ (TRIPOD GAİT)
==========================================
Hexapod robotun ileri doğru yürümesini sağlayan Tripod (üçayak) algoritması.

Algoritma Prensibi:
    Grup 1: Bacak 1, 4, 5 (Sol-Ön, Sağ-Orta, Sol-Arka)
    Grup 2: Bacak 2, 3, 6 (Sağ-Ön, Sol-Orta, Sağ-Arka)
    
    Her grupta 3 bacak aynı anda hareket eder, bu da robot için
    maksimum stabilite sağlar (her zaman 3 nokta yerde).

Orijinal: tri_gate.py
Türkçeleştiren: [Öğrenci Adı]
Tarih: Ekim 2025
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np


class IleriYuruyusKontrol(Node):
    """
    İleri yürüyüş hareketini kontrol eden ROS2 node sınıfı.
    
    Bu sınıf Tripod gait algoritmasını kullanarak robotun
    ileri doğru yürümesini sağlar.
    """
    
    def __init__(self):
        """Node'u başlatır ve gerekli publisher'ları oluşturur."""
        super().__init__('ileri_yuruyu_node')
        
        # Her bacak için trajectory publisher oluştur
        self.bacak_yayinlayicilar = {}
        for bacak_no in range(1, 7):
            topic_ismi = f'/leg{bacak_no}_controller/joint_trajectory'
            self.bacak_yayinlayicilar[bacak_no] = self.create_publisher(
                JointTrajectory,
                topic_ismi,
                10
            )
        
        self.get_logger().info('İleri yürüyüş sistemi başlatıldı')
        
        # Hareket parametreleri
        self.adim_uzunlugu = 0.3  # Her adımın uzunluğu (radyan)
        self.adim_yuksekligi = 0.4  # Bacağın kalma yüksekliği
        self.hareket_suresi = 0.5  # Her fazın süresi (saniye)
        
        # Hareket döngüsünü başlat
        self.timer = self.create_timer(self.hareket_suresi, self.tripod_yuruyu)
        self.faz = 0  # Hangi fazda olduğumuzu tutar (0 veya 1)
    
    def eklem_komutu_olustur(self, bacak_no, konum_listesi, sure):
        """
        Bir bacak için trajectory komutu oluşturur.
        
        Args:
            bacak_no (int): Bacak numarası (1-6)
            konum_listesi (list): [coxa, femur, tibia] joint açıları
            sure (float): Hareketi tamamlama süresi (saniye)
        
        Returns:
            JointTrajectory: Hazırlanmış trajectory mesajı
        """
        trajectory = JointTrajectory()
        
        # Joint isimlerini tanımla
        trajectory.joint_names = [
            f'joint1_{bacak_no}',  # Coxa (yatay dönüş)
            f'joint2_{bacak_no}',  # Femur (yukarı-aşağı)
            f'joint3_{bacak_no}'   # Tibia (diz)
        ]
        
        # Hedef noktayı oluştur
        nokta = JointTrajectoryPoint()
        nokta.positions = konum_listesi
        nokta.time_from_start = Duration(
            sec=int(sure),
            nanosec=int((sure % 1) * 1e9)
        )
        
        trajectory.points.append(nokta)
        return trajectory
    
    def tripod_yuruyu(self):
        """
        Tripod gait algoritmasını uygular.
        
        Faz 0: Grup 1 (1,4,5) havada, Grup 2 (2,3,6) yerde
        Faz 1: Grup 2 (2,3,6) havada, Grup 1 (1,4,5) yerde
        """
        
        if self.faz == 0:
            # FAZ 0: Grup 1 havada ve öne, Grup 2 yerde ve gövdeyi iter
            
            # Grup 1 (1, 4, 5) - Havada ve öne hareket
            for bacak_no in [1, 4, 5]:
                # Bacağı kaldır ve öne taşı
                konum = [
                    self.adim_uzunlugu,      # Coxa: öne
                    -self.adim_yuksekligi,   # Femur: yukarı
                    1.2                       # Tibia: hafif bükük
                ]
                trajectory = self.eklem_komutu_olustur(
                    bacak_no, konum, self.hareket_suresi
                )
                self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
            
            # Grup 2 (2, 3, 6) - Yerde ve vücudu öne iter
            for bacak_no in [2, 3, 6]:
                # Bacak yerde kalır, gövdeyi iterek öne taşır
                konum = [
                    -self.adim_uzunlugu,     # Coxa: geriye (itme)
                    0.15,                     # Femur: yerde
                    1.4                       # Tibia: düz
                ]
                trajectory = self.eklem_komutu_olustur(
                    bacak_no, konum, self.hareket_suresi
                )
                self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
            
            self.get_logger().info('Faz 0: Grup 1 ilerliyor')
            self.faz = 1
        
        else:
            # FAZ 1: Grup 2 havada ve öne, Grup 1 yerde ve gövdeyi iter
            
            # Grup 2 (2, 3, 6) - Havada ve öne hareket
            for bacak_no in [2, 3, 6]:
                konum = [
                    self.adim_uzunlugu,      # Coxa: öne
                    -self.adim_yuksekligi,   # Femur: yukarı
                    1.2                       # Tibia: hafif bükük
                ]
                trajectory = self.eklem_komutu_olustur(
                    bacak_no, konum, self.hareket_suresi
                )
                self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
            
            # Grup 1 (1, 4, 5) - Yerde ve vücudu öne iter
            for bacak_no in [1, 4, 5]:
                konum = [
                    -self.adim_uzunlugu,     # Coxa: geriye (itme)
                    0.15,                     # Femur: yerde
                    1.4                       # Tibia: düz
                ]
                trajectory = self.eklem_komutu_olustur(
                    bacak_no, konum, self.hareket_suresi
                )
                self.bacak_yayinlayicilar[bacak_no].publish(trajectory)
            
            self.get_logger().info('Faz 1: Grup 2 ilerliyor')
            self.faz = 0


def main(args=None):
    """Ana fonksiyon - ROS2 node'unu başlatır."""
    rclpy.init(args=args)
    
    # İleri yürüyüş node'unu oluştur
    ileri_yuruyu_node = IleriYuruyusKontrol()
    
    try:
        # Node'u çalıştır
        rclpy.spin(ileri_yuruyu_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Temizlik
        ileri_yuruyu_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
