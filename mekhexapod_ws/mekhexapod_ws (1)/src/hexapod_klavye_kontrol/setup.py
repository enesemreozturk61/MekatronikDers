from setuptools import setup

package_name = 'hexapod_klavye_kontrol'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Öğrenci Adı',
    maintainer_email='ogrenci@universite.edu.tr',
    description='Türkçe klavye kontrol sistemi',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'klavye_kontrol = hexapod_klavye_kontrol.klavye_kontrol:main',
        ],
    },
)
