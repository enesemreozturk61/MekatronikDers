#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════
                    DÖNÜŞ HAREKET ALGORITMASI
                      (IN-PLACE ROTATION)
═══════════════════════════════════════════════════════════════════

PROJE ADI    : Hexapod Robot Kontrol Sistemi
DOSYA ADI    : rotation.py (Yerinde Dönüş)
GELİŞTİRİCİ  : OZTURK
EMAIL        : enozturtka@gmail.com
TARİH        : Kasım 2025
VERSİYON     : 2.0 (Hızlandırılmış + Türkçe Açıklamalı)

AÇIKLAMA:
---------
Bu script, hexapod robotun kendi etrafında dönmesini sağlar.
Robot ileri veya geri gitmeden sadece yerinde döner (in-place rotation).

DÖNÜŞ PRENSİBİ:
--------------
Robot 6 bacaklıdır ve bacaklar simetrik şekilde hareket eder:

    SOL TARAF:  Bacak 1 (Sol-Ön)    ●  ┐
                Bacak 3 (Sol-Orta)  ●  ├─ Bir yöne döner
                Bacak 5 (Sol-Arka)  ●  ┘

    SAĞ TARAF:  Bacak 2 (Sağ-Ön)       ●  ┐
                Bacak 4 (Sağ-Orta)      ●  ├─ Ters yöne döner
                Bacak 6 (Sağ-Arka)      ●  ┘

HAREKET MEKANİĞİ:
----------------
1. Sol bacaklar saat yönünde döner (pozitif açı)
2. Sağ bacaklar saat yönünün tersine döner (negatif açı)
3. Bu karşıt kuvvetler robotun merkezinde dönmesini sağlar
4. Robot yerinden kalkmaz, sadece döner

MATEMATİK:
---------
Dönüş açısı = ±0.35 radyan (±20 derece)

    Sol bacaklar:  Coxa = +0.35 rad  (sağa dön)
    Sağ bacaklar:  Coxa = -0.35 rad  (sola dön)
    
    Sonuç: Robot sağa döner (clockwise)

YÖN DEĞİŞTİRME:
--------------
Eğer sola dönmek istersen:
    - Sol bacaklar: negatif açı
    - Sağ bacaklar: pozitif açı

HAREKET PARAMETRELERİ:
---------------------
    - Dönüş açısı: 0.35 radyan (~20 derece)
    - Hareket süresi: 0.5 saniye
    - Dönüş yönü: Sağa (clockwise)

EKLEM POZİSYONLARI:
------------------
    - joint1 (COXA):  Dönüş açısı (sol/sağ değişir)
    - joint2 (FEMUR): 0.15 rad (hafif yukarı - dengeli duruş)
    - joint3 (TIBIA): 1.4 rad (düz - güçlü duruş)

KULLANIM:
--------
Terminal'den:
    ros2 run hex_gz rotation.py

Klavye kontrolden:
    A tuşu: Sola dön
    D tuşu: Sağa dön

NOT:
----
Bu script sürekli döner. Durdurmak için SPACE tuşuna basın
veya Ctrl+C ile programı sonlandırın.

═══════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class DonusKontrolcu(Node):
    """
    Dönüş hareketini kontrol eden ROS2 Node sınıfı.
    
    Bu sınıf robotun yerinde dönmesini sağlar.
    Sol ve sağ bacaklar ters yönde hareket ederek
    robot merkezinde döner.
    """
    
    def __init__(self):
        """
        Node başlatma ve ayarlar.
        
        Yapılan işlemler:
            - 6 bacak için publisher oluştur
            - Dönüş parametrelerini ayarla
            - Periyodik zamanlayıcı başlat
        """
        super().__init__('rotation')
        
        # ═══════════════════════════════════════════════════════════
        # ADIM 1: PUBLISHER'LARI OLUŞTUR
        # ═══════════════════════════════════════════════════════════
        self.bacak_yayinlayicilar = {}
        
        for bacak_numarasi in range(1, 7):
            topic_adi = f'/leg{bacak_numarasi}_controller/joint_trajectory'
            self.bacak_yayinlayicilar[bacak_numarasi] = self.create_publisher(
                JointTrajectory,
                topic_adi,
                10
            )
        
        self.get_logger().info('═' * 60)
        self.get_logger().info('✅ DÖNÜŞ SİSTEMİ BAŞLATILDI')
        self.get_logger().info('═' * 60)
        
        # ═══════════════════════════════════════════════════════════
        # ADIM 2: DÖNÜŞ PARAMETRELERİ
        # ═══════════════════════════════════════════════════════════
        self.donus_acisi = 0.35          # Her adımda dönüş açısı (radyan)
        self.hareket_suresi = 0.5        # Hareket süresi (saniye) - HIZLANDIRILDI
        self.donus_yonu = 1              # 1: sağa (CW), -1: sola (CCW)
        
        self.get_logger().info(f'📊 Dönüş açısı: {self.donus_acisi} rad (~20°)')
        self.get_logger().info(f'📊 Hareket süresi: {self.hareket_suresi} saniye')
        self.get_logger().info(f'📊 Dönüş yönü: {"SAĞA ➡️" if self.donus_yonu > 0 else "SOLA ⬅️"}')
        self.get_logger().info('═' * 60)
        
        # ═══════════════════════════════════════════════════════════
        # ADIM 3: ZAMANLAYICI BAŞLAT
        # ═══════════════════════════════════════════════════════════
        # Her 0.5 saniyede bir don() fonksiyonu çağrılır
        self.zamanlayici = self.create_timer(
            self.hareket_suresi,
            self.don
        )
    
    def trajectory_mesaji_olustur(self, bacak_numarasi, eklem_pozisyonlari, sure):
        """
        Trajectory mesajı oluştur.
        
        Args:
            bacak_numarasi (int): Hedef bacak (1-6)
            eklem_pozisyonlari (list): [coxa, femur, tibia] açıları
            sure (float): Hareket süresi (saniye)
        
        Returns:
            JointTrajectory: Hazır trajectory mesajı
        """
        trajectory_mesaji = JointTrajectory()
        
        # Hedef eklemleri belirle
        trajectory_mesaji.joint_names = [
            f'joint1_{bacak_numarasi}',  # Coxa
            f'joint2_{bacak_numarasi}',  # Femur
            f'joint3_{bacak_numarasi}'   # Tibia
        ]
        
        # Hedef pozisyonu oluştur
        hedef_nokta = JointTrajectoryPoint()
        hedef_nokta.positions = eklem_pozisyonlari
        hedef_nokta.time_from_start = Duration(
            sec=int(sure),
            nanosec=int((sure % 1) * 1e9)
        )
        
        trajectory_mesaji.points.append(hedef_nokta)
        return trajectory_mesaji
    
    def don(self):
        """
        Dönüş hareketini gerçekleştirir.
        
        Hareket mantığı:
            1. Sol bacaklar (1,3,5): Pozitif açı ile döner
            2. Sağ bacaklar (2,4,6): Negatif açı ile döner
            3. Bu karşıt kuvvetler robotun yerinde dönmesini sağlar
        
        Matematik:
            Sol bacak coxa açısı  = +0.35 * donus_yonu
            Sağ bacak coxa açısı  = -0.35 * donus_yonu
            
            donus_yonu = +1 → Robot sağa döner
            donus_yonu = -1 → Robot sola döner
        
        Örnek (Sağa dönüş):
            Bacak 1 (Sol-Ön):  coxa = +0.35  (sağa iter)
            Bacak 2 (Sağ-Ön):  coxa = -0.35  (sola iter)
            Sonuç: Robot sağa döner
        """
        
        # ═══════════════════════════════════════════════════════════
        # TÜM BACAKLARI DÖNDÜR
        # ═══════════════════════════════════════════════════════════
        
        for bacak_numarasi in range(1, 7):
            
            # --- Dönüş açısını belirle ---
            if bacak_numarasi in [1, 3, 5]:  # Sol bacaklar
                aci = self.donus_acisi * self.donus_yonu
                taraf = "Sol"
            else:  # Sağ bacaklar (2, 4, 6)
                aci = -self.donus_acisi * self.donus_yonu
                taraf = "Sağ"
            
            # --- Pozisyonu hazırla ---
            pozisyon = [
                aci,      # joint1 (Coxa): Dönüş açısı
                0.15,     # joint2 (Femur): Hafif yukarı (dengeli duruş)
                1.4       # joint3 (Tibia): Düz pozisyon (güçlü duruş)
            ]
            
            # --- Mesajı oluştur ve gönder ---
            mesaj = self.trajectory_mesaji_olustur(
                bacak_numarasi,
                pozisyon,
                self.hareket_suresi
            )
            
            self.bacak_yayinlayicilar[bacak_numarasi].publish(mesaj)
            
            # Debug bilgisi (ilk bacak için)
            if bacak_numarasi == 1:
                yon_metni = "sağa ➡️" if self.donus_yonu > 0 else "sola ⬅️"
                self.get_logger().debug(
                    f'🔄 Dönüyor: {yon_metni} | '
                    f'Sol açı: {aci:.2f} rad | '
                    f'Sağ açı: {-aci:.2f} rad'
                )


def main(args=None):
    """
    Ana fonksiyon - Program başlangıcı.
    
    İşlemler:
        1. ROS2'yi başlat
        2. Dönüş node'unu oluştur
        3. Node'u çalıştır
        4. Temiz kapanış sağla
    """
    # ROS2 sistemini başlat
    rclpy.init(args=args)
    
    # Dönüş kontrolcüsünü oluştur
    node = DonusKontrolcu()
    
    try:
        # Node'u çalıştır (sonsuz döngü)
        # Ctrl+C ile durur
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        # Kullanıcı programı durdurdu
        node.get_logger().info('⚠️  Kullanıcı tarafından durduruldu')
    
    finally:
        # Temizlik işlemleri
        node.destroy_node()
        rclpy.shutdown()
        print('✅ Dönüş sistemi kapatıldı\n')


if __name__ == '__main__':
    main()
