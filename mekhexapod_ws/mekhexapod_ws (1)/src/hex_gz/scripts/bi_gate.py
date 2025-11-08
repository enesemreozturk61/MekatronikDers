#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════
                    GERİ YÜRÜYÜŞ HAREKET ALGORITMASI
                          (BIPOD GAIT PATTERN)
═══════════════════════════════════════════════════════════════════

PROJE ADI    : Hexapod Robot Kontrol Sistemi
DOSYA ADI    : bi_gate.py (Geri Yürüyüş)
GELİŞTİRİCİ  : OZTURK
EMAIL        : enozturtka@gmail.com
TARİH        : Kasım 2025
VERSİYON     : 2.0 (Hızlandırılmış + Türkçe Açıklamalı)

AÇIKLAMA:
---------
Bu script, hexapod robotun geriye doğru yürümesini sağlar.
Bipod Gait algoritması kullanılır - sol ve sağ taraf sırayla hareket eder.

BIPOD GAIT PRENSİBİ:
-------------------
Robot bacakları iki gruba ayrılır (sol ve sağ):

    SOL GRUP: Bacak 1 (Sol-Ön)    ●
              Bacak 3 (Sol-Orta)  ●
              Bacak 5 (Sol-Arka)  ●

    SAĞ GRUP: Bacak 2 (Sağ-Ön)       ●
              Bacak 4 (Sağ-Orta)      ●
              Bacak 6 (Sağ-Arka)      ●

HAREKET DÖNGüSü (2 FAZ):
-----------------------
FAZ 0:
    - Sol grup (1,3,5): Havada geriye hareket
    - Sağ grup (2,4,6): Yerde vücudu geriye iter
    → Robot geri gider

FAZ 1:
    - Sağ grup (2,4,6): Havada geriye hareket
    - Sol grup (1,3,5): Yerde vücudu geriye iter
    → Robot geri gider

TRİPOD'DAN FARKI:
----------------
- Tripod'da 3 bacak aynı anda hareket eder (daha dengeli)
- Bipod'da tek taraf hareket eder (geri giderken daha kontrollü)

HAREKET PARAMETRELERİ:
---------------------
    - Adım uzunluğu: 0.25 radyan  
    - Kaldırma yüksekliği: 0.35 radyan
    - Hareket süresi: 0.35 saniye

KULLANIM:
--------
Terminal'den:
    ros2 run hex_gz bi_gate.py

Klavye kontrolden:
    S tuşu ile çağırılır

═══════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class GeriYuruyusKontrolcu(Node):
    """
    Geri yürüyüş hareketini kontrol eden ROS2 Node sınıfı.
    
    Bipod gait algoritması ile sol ve sağ tarafı sırayla
    koordine ederek geriye hareket sağlar.
    """
    
    def __init__(self):
        """Node başlatma ve ayarlar."""
        super().__init__('bi_gate')
        
        # ═══════════════════════════════════════════════════════════
        # PUBLISHER'LARI OLUŞTUR
        # ═══════════════════════════════════════════════════════════
        self.bacak_yayinlayicilar = {}
        
        for bacak_numarasi in range(1, 7):
            topic_adi = f'/leg{bacak_numarasi}_controller/joint_trajectory'
            self.bacak_yayinlayicilar[bacak_numarasi] = self.create_publisher(
                JointTrajectory, topic_adi, 10
            )
        
        self.get_logger().info('═' * 60)
        self.get_logger().info('✅ GERİ YÜRÜYÜŞ SİSTEMİ BAŞLATILDI')
        self.get_logger().info('═' * 60)
        
        # ═══════════════════════════════════════════════════════════
        # HAREKET PARAMETRELERİ
        # ═══════════════════════════════════════════════════════════
        self.adim_uzunlugu = 0.25         # Daha küçük adımlar (geri giderken güvenli)
        self.kaldir_yuksekligi = 0.35     # Kaldırma yüksekliği
        self.hareket_suresi = 0.35        # Hareket süresi (hızlandırılmış)
        self.faz = 0                      # Başlangıç fazı (0: Sol, 1: Sağ)
        
        self.get_logger().info(f'📊 Adım uzunluğu: {self.adim_uzunlugu} rad')
        self.get_logger().info(f'📊 Hareket süresi: {self.hareket_suresi} saniye')
        self.get_logger().info('═' * 60)
        
        # ═══════════════════════════════════════════════════════════
        # ZAMANLAYICI
        # ═══════════════════════════════════════════════════════════
        self.zamanlayici = self.create_timer(
            self.hareket_suresi,
            self.bipod_adim_at
        )
    
    def trajectory_mesaji_olustur(self, bacak_numarasi, eklem_pozisyonlari, sure):
        """Trajectory mesajı oluştur."""
        trajectory_mesaji = JointTrajectory()
        trajectory_mesaji.joint_names = [
            f'joint1_{bacak_numarasi}',
            f'joint2_{bacak_numarasi}',
            f'joint3_{bacak_numarasi}'
        ]
        
        hedef_nokta = JointTrajectoryPoint()
        hedef_nokta.positions = eklem_pozisyonlari
        hedef_nokta.time_from_start = Duration(
            sec=int(sure),
            nanosec=int((sure % 1) * 1e9)
        )
        
        trajectory_mesaji.points.append(hedef_nokta)
        return trajectory_mesaji
    
    def bipod_adim_at(self):
        """
        Bipod gait algoritması - Bir adım atar.
        
        FAZ 0: Sol bacaklar geriye
        FAZ 1: Sağ bacaklar geriye
        """
        
        if self.faz == 0:
            # ═══════════════════════════════════════════════════════
            # FAZ 0: SOL BACAKLAR GERİYE
            # ═══════════════════════════════════════════════════════
            
            # Sol bacaklar havada geriye
            for bacak_no in [1, 3, 5]:
                pozisyon = [
                    -self.adim_uzunlugu,       # Geriye dönüş
                    -self.kaldir_yuksekligi,   # Yukarı kaldır
                    1.2                         # Hafif bükük
                ]
                mesaj = self.trajectory_mesaji_olustur(bacak_no, pozisyon, self.hareket_suresi)
                self.bacak_yayinlayicilar[bacak_no].publish(mesaj)
            
            # Sağ bacaklar yerde (vücudu iter)
            for bacak_no in [2, 4, 6]:
                pozisyon = [
                    self.adim_uzunlugu,        # Öne (itme)
                    0.15,                       # Yerde
                    1.4                         # Düz
                ]
                mesaj = self.trajectory_mesaji_olustur(bacak_no, pozisyon, self.hareket_suresi)
                self.bacak_yayinlayicilar[bacak_no].publish(mesaj)
            
            self.get_logger().debug('🔄 FAZ 0: Sol bacaklar geriye hareket ediyor')
            self.faz = 1
        
        else:
            # ═══════════════════════════════════════════════════════
            # FAZ 1: SAĞ BACAKLAR GERİYE
            # ═══════════════════════════════════════════════════════
            
            # Sağ bacaklar havada geriye
            for bacak_no in [2, 4, 6]:
                pozisyon = [
                    -self.adim_uzunlugu,
                    -self.kaldir_yuksekligi,
                    1.2
                ]
                mesaj = self.trajectory_mesaji_olustur(bacak_no, pozisyon, self.hareket_suresi)
                self.bacak_yayinlayicilar[bacak_no].publish(mesaj)
            
            # Sol bacaklar yerde (vücudu iter)
            for bacak_no in [1, 3, 5]:
                pozisyon = [
                    self.adim_uzunlugu,
                    0.15,
                    1.4
                ]
                mesaj = self.trajectory_mesaji_olustur(bacak_no, pozisyon, self.hareket_suresi)
                self.bacak_yayinlayicilar[bacak_no].publish(mesaj)
            
            self.get_logger().debug('🔄 FAZ 1: Sağ bacaklar geriye hareket ediyor')
            self.faz = 0


def main(args=None):
    """Ana fonksiyon."""
    rclpy.init(args=args)
    node = GeriYuruyusKontrolcu()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('⚠️  Kullanıcı tarafından durduruldu')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('✅ Geri yürüyüş sistemi kapatıldı\n')


if __name__ == '__main__':
    main()
