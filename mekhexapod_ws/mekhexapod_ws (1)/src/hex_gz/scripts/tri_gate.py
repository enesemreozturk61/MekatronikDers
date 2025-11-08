#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════
                    İLERİ YÜRÜYÜŞ HAREKET ALGORITMASI
                          (TRIPOD GAIT PATTERN)
═══════════════════════════════════════════════════════════════════

PROJE ADI    : Hexapod Robot Kontrol Sistemi
DOSYA ADI    : tri_gate.py (İleri Yürüyüş)
GELİŞTİRİCİ  : OZTURK
EMAIL        : enozturtka@gmail.com
TARİH        : Kasım 2025
VERSİYON     : 2.0 (Hızlandırılmış + Türkçe Açıklamalı)

AÇIKLAMA:
---------
Bu script, hexapod robotun ileri doğru yürümesini sağlar.
Tripod Gait algoritması kullanılır - her zaman 3 bacak yerde kalır
böylece robot maksimum dengede olur.

TRIPOD GAIT PRENSİBİ:
--------------------
Robot 6 bacaklıdır ve bacaklar 2 gruba ayrılır:

    GRUP 1: Bacak 1 (Sol-Ön)    ●
            Bacak 4 (Sağ-Orta)       ●
            Bacak 5 (Sol-Arka)  ●

    GRUP 2: Bacak 2 (Sağ-Ön)       ●
            Bacak 3 (Sol-Orta)  ●
            Bacak 6 (Sağ-Arka)      ●

HAREKET DÖNGüSü (2 FAZ):
-----------------------
FAZ 0: 
    - Grup 1 (1,4,5): Havada öne hareket eder
    - Grup 2 (2,3,6): Yerde kalıp vücudu iter
    → Robot ileri gider, Grup 1'in bacakları hazır pozisyona gelir

FAZ 1:
    - Grup 2 (2,3,6): Havada öne hareket eder  
    - Grup 1 (1,4,5): Yerde kalıp vücudu iter
    → Robot ileri gider, Grup 2'nin bacakları hazır pozisyona gelir

Bu iki faz sürekli tekrarlanır ve robot düzgün ilerleme sağlar.

EKLEM AÇIKLARI:
--------------
Her bacakta 3 eklem vardır:
    - joint1 (COXA):  Yatay dönüş ekseni (sağa-sola)
    - joint2 (FEMUR): Yukarı-aşağı ekseni (bacağı kaldırma)
    - joint3 (TIBIA): Diz eklemi (bacağı bükme)

HAREKET PARAMETRELERİ:
---------------------
    - Adım uzunluğu (stride_length): 0.3 radyan
    - Kaldırma yüksekliği (lift_height): 0.4 radyan
    - Hareket süresi (step_duration): 0.3 saniye
    - Faz değişim hızı: Otomatik (zamanlayıcı ile)

ROS2 TOPIC'LER:
--------------
Her bacak için ayrı trajectory gönderilir:
    /leg1_controller/joint_trajectory
    /leg2_controller/joint_trajectory
    ...
    /leg6_controller/joint_trajectory

KULLANIM:
--------
Terminal'den çalıştırma:
    ros2 run hex_gz tri_gate.py

Veya klavye kontrolden:
    W tuşu ile çağırılır

═══════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class IleriYuruyusKontrolcu(Node):
    """
    İleri yürüyüş hareketini kontrol eden ROS2 Node sınıfı.
    
    Bu sınıf:
        1. Her bacak için ROS2 publisher oluşturur
        2. Zamanlayıcı ile periyodik hareket döngüsü başlatır
        3. İki grup bacağı sırayla koordine eder
        4. Sürekli ileri hareket sağlar
    """
    
    def __init__(self):
        """
        Node başlatma ve başlangıç ayarlarını yapma.
        
        Yapılan işlemler:
            - ROS2 node'u 'tri_gate' adıyla başlatılır
            - 6 bacak için ayrı publisher'lar oluşturulur
            - Hareket parametreleri tanımlanır
            - Zamanlayıcı başlatılır
        """
        super().__init__('tri_gate')
        
        # ═══════════════════════════════════════════════════════════
        # ADIM 1: HER BACAK İÇİN PUBLISHER OLUŞTUR
        # ═══════════════════════════════════════════════════════════
        self.bacak_yayinlayicilar = {}
        
        for bacak_numarasi in range(1, 7):  # Bacak 1'den 6'ya kadar
            topic_adi = f'/leg{bacak_numarasi}_controller/joint_trajectory'
            
            self.bacak_yayinlayicilar[bacak_numarasi] = self.create_publisher(
                JointTrajectory,  # Mesaj tipi
                topic_adi,        # Topic adı
                10                # Queue boyutu
            )
        
        self.get_logger().info('═' * 60)
        self.get_logger().info('✅ İLERİ YÜRÜYÜŞ SİSTEMİ BAŞLATILDI')
        self.get_logger().info('═' * 60)
        self.get_logger().info('📊 Hareket Parametreleri:')
        
        # ═══════════════════════════════════════════════════════════
        # ADIM 2: HAREKET PARAMETRELERİNİ TANIMLA
        # ═══════════════════════════════════════════════════════════
        self.adim_uzunlugu = 0.3      # Coxa ekleminin dönüş miktarı (radyan)
        self.kaldir_yuksekligi = 0.4  # Bacağın yerden kalkma yüksekliği
        self.hareket_suresi = 0.3     # Her fazın tamamlanma süresi (saniye)
        self.faz = 0                  # Başlangıç fazı (0: Grup 1, 1: Grup 2)
        
        self.get_logger().info(f'   → Adım uzunluğu: {self.adim_uzunlugu} rad')
        self.get_logger().info(f'   → Kaldırma yüksekliği: {self.kaldir_yuksekligi} rad')
        self.get_logger().info(f'   → Hareket süresi: {self.hareket_suresi} saniye')
        self.get_logger().info('═' * 60)
        
        # ═══════════════════════════════════════════════════════════
        # ADIM 3: PERİYODİK ZAMANLAYICI BAŞLAT
        # ═══════════════════════════════════════════════════════════
        # Her 0.3 saniyede bir tripod_adim_at() fonksiyonu çağrılır
        self.zamanlayici = self.create_timer(
            self.hareket_suresi,     # Periyot (saniye)
            self.tripod_adim_at      # Çağrılacak fonksiyon
        )
    
    def trajectory_mesaji_olustur(self, bacak_numarasi, eklem_pozisyonlari, sure):
        """
        Bir bacak için trajectory komutu mesajı oluşturur.
        
        Args:
            bacak_numarasi (int): Hedef bacak numarası (1-6)
            eklem_pozisyonlari (list): [coxa, femur, tibia] açıları (radyan)
            sure (float): Hedefe ulaşma süresi (saniye)
        
        Returns:
            JointTrajectory: ROS2 trajectory mesajı
        
        Açıklama:
            Bu fonksiyon, belirlenen bacağın eklemlerini istenen 
            pozisyonlara götürecek trajectory mesajını hazırlar.
        """
        # Boş trajectory mesajı oluştur
        trajectory_mesaji = JointTrajectory()
        
        # Hedef eklemleri belirle
        trajectory_mesaji.joint_names = [
            f'joint1_{bacak_numarasi}',  # Coxa: Yatay dönüş
            f'joint2_{bacak_numarasi}',  # Femur: Yukarı-aşağı
            f'joint3_{bacak_numarasi}'   # Tibia: Diz
        ]
        
        # Hedef noktayı oluştur
        hedef_nokta = JointTrajectoryPoint()
        hedef_nokta.positions = eklem_pozisyonlari
        
        # Süreyi ROS2 formatına çevir
        hedef_nokta.time_from_start = Duration(
            sec=int(sure),                      # Tam saniye kısmı
            nanosec=int((sure % 1) * 1e9)      # Nanosaniye kısmı
        )
        
        # Hedef noktayı trajectory'e ekle
        trajectory_mesaji.points.append(hedef_nokta)
        
        return trajectory_mesaji
    
    def tripod_adim_at(self):
        """
        Tripod gait algoritmasının bir adımını gerçekleştirir.
        
        Bu fonksiyon zamanlayıcı tarafından sürekli çağrılır.
        Her çağrıda bir faz ilerler (Faz 0 → Faz 1 → Faz 0 ...)
        
        FAZ 0: Grup 1 bacaklar havada ilerler, Grup 2 yerde iter
        FAZ 1: Grup 2 bacaklar havada ilerler, Grup 1 yerde iter
        
        Bu sayede robot sürekli dengeli bir şekilde ilerler.
        """
        
        if self.faz == 0:
            # ═══════════════════════════════════════════════════════
            # FAZ 0: GRUP 1 HAVADA, GRUP 2 YERDE
            # ═══════════════════════════════════════════════════════
            
            # --- GRUP 1: Havada öne hareket et (1, 4, 5) ---
            for bacak_no in [1, 4, 5]:
                pozisyon = [
                    self.adim_uzunlugu,        # Coxa: Öne dönüş
                    -self.kaldir_yuksekligi,   # Femur: Yukarı kaldır (- değeri yukarı demek)
                    1.2                         # Tibia: Hafif bükük pozisyon
                ]
                
                mesaj = self.trajectory_mesaji_olustur(
                    bacak_no, 
                    pozisyon, 
                    self.hareket_suresi
                )
                
                self.bacak_yayinlayicilar[bacak_no].publish(mesaj)
            
            # --- GRUP 2: Yerde vücudu öne it (2, 3, 6) ---
            for bacak_no in [2, 3, 6]:
                pozisyon = [
                    -self.adim_uzunlugu,       # Coxa: Geriye dönüş (vücut öne gider)
                    0.15,                       # Femur: Yerde, hafif yukarıda
                    1.4                         # Tibia: Düz pozisyon (itme gücü için)
                ]
                
                mesaj = self.trajectory_mesaji_olustur(
                    bacak_no, 
                    pozisyon, 
                    self.hareket_suresi
                )
                
                self.bacak_yayinlayicilar[bacak_no].publish(mesaj)
            
            self.get_logger().debug('🔄 FAZ 0: Grup 1 (1,4,5) havada → Grup 2 (2,3,6) yerde')
            self.faz = 1  # Sonraki çağrıda Faz 1'e geç
        
        else:
            # ═══════════════════════════════════════════════════════
            # FAZ 1: GRUP 2 HAVADA, GRUP 1 YERDE
            # ═══════════════════════════════════════════════════════
            
            # --- GRUP 2: Havada öne hareket et (2, 3, 6) ---
            for bacak_no in [2, 3, 6]:
                pozisyon = [
                    self.adim_uzunlugu,
                    -self.kaldir_yuksekligi,
                    1.2
                ]
                
                mesaj = self.trajectory_mesaji_olustur(
                    bacak_no, 
                    pozisyon, 
                    self.hareket_suresi
                )
                
                self.bacak_yayinlayicilar[bacak_no].publish(mesaj)
            
            # --- GRUP 1: Yerde vücudu öne it (1, 4, 5) ---
            for bacak_no in [1, 4, 5]:
                pozisyon = [
                    -self.adim_uzunlugu,
                    0.15,
                    1.4
                ]
                
                mesaj = self.trajectory_mesaji_olustur(
                    bacak_no, 
                    pozisyon, 
                    self.hareket_suresi
                )
                
                self.bacak_yayinlayicilar[bacak_no].publish(mesaj)
            
            self.get_logger().debug('🔄 FAZ 1: Grup 2 (2,3,6) havada → Grup 1 (1,4,5) yerde')
            self.faz = 0  # Sonraki çağrıda Faz 0'a dön


def main(args=None):
    """
    Ana fonksiyon - Program başlangıç noktası.
    
    Bu fonksiyon:
        1. ROS2 sistemini başlatır
        2. İleri yürüyüş node'unu oluşturur
        3. Node'u çalıştırır (spin)
        4. Temiz kapanış sağlar
    """
    # ROS2'yi başlat
    rclpy.init(args=args)
    
    # İleri yürüyüş kontrolcüsü oluştur
    node = IleriYuruyusKontrolcu()
    
    try:
        # Node'u çalıştır (sonsuz döngü - Ctrl+C ile durur)
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        # Kullanıcı Ctrl+C bastı
        node.get_logger().info('⚠️  Kullanıcı tarafından durduruldu')
    
    finally:
        # Temiz kapanış
        node.destroy_node()
        rclpy.shutdown()
        print('✅ İleri yürüyüş sistemi kapatıldı\n')


if __name__ == '__main__':
    main()
