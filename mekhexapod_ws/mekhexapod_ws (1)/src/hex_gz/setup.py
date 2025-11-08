from setuptools import setup
import os
from glob import glob

package_name = 'hex_gz'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch dosyaları
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # World dosyaları (varsa)
        (os.path.join('share', package_name, 'worlds'), 
            glob('worlds/*.world') if os.path.exists('worlds') else []),
        # Config dosyaları (varsa)
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml') if os.path.exists('config') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Öğrenci Adı',
    maintainer_email='ogrenci@universite.edu.tr',
    description='Hexapod hareket scriptleri',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Türkçe script isimleri
            'ileri_yuruyu = scripts.ileri_yuruyu:main',
            'geri_yuruyu = scripts.geri_yuruyu:main',
            'donus = scripts.donus:main',
            'selam_ver = scripts.selam_ver:main',
            'dans_bir = scripts.dans_bir:main',
            'dans_iki = scripts.dans_iki:main',
            
            # Orijinal isimler (geriye uyumluluk)
            'tri_gate.py = scripts.tri_gate:main',
            'bi_gate.py = scripts.bi_gate:main',
            'rotation.py = scripts.rotation:main',
            'hi.py = scripts.hi:main',
            'dance.py = scripts.dance:main',
            'dance1.py = scripts.dance1:main',
        ],
    },
)
