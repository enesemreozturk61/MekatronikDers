#!/usr/bin/env python3
"""
HEXAPOD KLAVYE KONTROL SİSTEMİ
==============================
Optimize edilmiş klavye kontrol programı.

Geliştirici: [Öğrenci Adı]
Tarih: Ekim 2025
"""

import subprocess
import sys
import termios
import tty
import time
import os
import signal


class HexapodKlavyeKontrol:
    """Hexapod klavye kontrolü ana sınıf."""
    
    def __init__(self):
        """Sistemi başlat."""
        self.terminal_ayarlari = termios.tcgetattr(sys.stdin)
        self.calisan_hareket_pid = None
        self.kullanim_kilavuzunu_goster()
    
    def kullanim_kilavuzunu_goster(self):
        """Kontrol tuşlarını göster."""
        print('=' * 60)
        print('     HEXAPOD ROBOT KLAVYE KONTROL SİSTEMİ')
        print('=' * 60)
        print('\n🎮 HAREKET KONTROLLERI:')
        print('  W → İleri yürü')
        print('  S → Geri yürü')
        print('  A → Sola dön')
        print('  D → Sağa dön')
        print('\n🎭 ÖZEL HAREKETLER:')
        print('  H → Selam ver')
        print('  1 → Dans 1')
        print('  2 → Dans 2')
        print('\n⚙️  SİSTEM:')
        print('  SPACE → Durdur')
        print('  Q → Çıkış')
        print('=' * 60 + '\n')
    
    def tus_oku(self):
        """
        Klavyeden tek bir tuş oku (blocking).
        
        Returns:
            str: Basılan tuş
        """
        # Raw moda geç
        tty.setraw(sys.stdin.fileno())
        
        # Tek karakter oku (blocking - tuş basılana kadar bekler)
        tus = sys.stdin.read(1)
        
        # Normal moda dön
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_ayarlari)
        
        return tus
    
    def hareketleri_durdur(self):
        """Tüm hareketleri agresif şekilde durdur."""
        print('🛑 Durduruluyor...', end=' ', flush=True)
        
        # PID varsa önce onu durdur
        if self.calisan_hareket_pid:
            try:
                os.kill(self.calisan_hareket_pid, signal.SIGTERM)
                time.sleep(0.2)
                try:
                    os.kill(self.calisan_hareket_pid, signal.SIGKILL)
                except:
                    pass
            except:
                pass
            self.calisan_hareket_pid = None
        
        # Tüm hex_gz süreçlerini bul ve öldür
        try:
            # pgrep ile süreçleri bul
            result = subprocess.run(
                ['pgrep', '-f', 'hex_gz'],
                capture_output=True,
                text=True,
                timeout=1
            )
            
            if result.stdout:
                pids = result.stdout.strip().split('\n')
                for pid in pids:
                    try:
                        os.kill(int(pid), signal.SIGKILL)
                    except:
                        pass
        except:
            pass
        
        # Alternatif: pkill kullan
        try:
            subprocess.run(
                ['pkill', '-9', '-f', 'hex_gz'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1
            )
        except:
            pass
        
        time.sleep(0.2)
        print('✅')
    
    def hareket_calistir(self, script_adi):
        """
        Hareket scriptini çalıştır.
        
        Args:
            script_adi: Çalıştırılacak scriptin adı
        """
        # Önce durdur
        self.hareketleri_durdur()
        
        print(f'▶️  {script_adi}', end=' ', flush=True)
        
        try:
            # Yeni subprocess başlat
            proc = subprocess.Popen(
                ['ros2', 'run', 'hex_gz', script_adi],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL,
                start_new_session=True  # Yeni session'da başlat
            )
            
            # PID'yi kaydet
            self.calisan_hareket_pid = proc.pid
            
            # Kısa bekleme
            time.sleep(0.15)
            
            # Hala çalışıyor mu?
            if proc.poll() is None:
                print('✅')
            else:
                print('❌')
                self.calisan_hareket_pid = None
                
        except Exception as e:
            print(f'❌ Hata: {e}')
            self.calisan_hareket_pid = None
    
    def kontrol_dongusu(self):
        """Ana kontrol döngüsü."""
        try:
            while True:
                # Tuş oku (blocking - tuşa basılana kadar bekler)
                tus = self.tus_oku()
                
                # Tuşa göre işlem yap
                if tus.lower() == 'w':
                    self.hareket_calistir('tri_gate.py')
                
                elif tus.lower() == 's':
                    self.hareket_calistir('bi_gate.py')
                
                elif tus.lower() == 'a':
                    self.hareket_calistir('rotation.py')
                
                elif tus.lower() == 'd':
                    self.hareket_calistir('rotation.py')
                
                elif tus.lower() == 'h':
                    self.hareket_calistir('hi.py')
                
                elif tus == '1':
                    self.hareket_calistir('dance.py')
                
                elif tus == '2':
                    self.hareket_calistir('dance1.py')
                
                elif tus == ' ':
                    # SPACE - Durdur
                    self.hareketleri_durdur()
                
                elif tus.lower() == 'q' or tus == '\x03':
                    # Q veya Ctrl+C - Çıkış
                    print('\n👋 Çıkış yapılıyor...')
                    break
        
        except KeyboardInterrupt:
            print('\n⚠️  Program sonlandırıldı')
        
        finally:
            # Terminal'i düzelt
            try:
                termios.tcsetattr(
                    sys.stdin,
                    termios.TCSADRAIN,
                    self.terminal_ayarlari
                )
            except:
                pass
            
            # Hareketleri durdur
            self.hareketleri_durdur()
            
            print('✅ Program kapatıldı\n')


def main(args=None):
    """Program başlangıcı."""
    print('\n🚀 Klavye kontrol sistemi başlatılıyor...\n')
    kontrol = HexapodKlavyeKontrol()
    kontrol.kontrol_dongusu()


if __name__ == '__main__':
    main()
