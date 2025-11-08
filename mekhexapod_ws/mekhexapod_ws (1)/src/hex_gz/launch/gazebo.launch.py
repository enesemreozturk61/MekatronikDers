#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════
                    HEXAPOD GAZEBO SİMÜLASYON BAŞLATICI
═══════════════════════════════════════════════════════════════════════════

PROJE ADI    : Hexapod Robot Kontrol Sistemi
DOSYA ADI    : gazebo.launch.py
GELİŞTİRİCİ  : OZTURK
EMAIL        : enozturtka@gmail.com
TARİH        : Kasım 2025
VERSİYON     : 2.0 (Türkçe Açıklamalı)

AÇIKLAMA:
---------
Bu launch dosyası, hexapod robotun Gazebo simülasyonunu başlatır.

YAPILAN İŞLEMLER:
----------------
1. Gazebo simülasyon ortamını başlat
2. Robot modelini (URDF) yükle
3. ROS2 Control sistemini başlat
4. Eklem kontrolcülerini (joint controllers) başlat
5. Robot'u Gazebo dünyasına spawn et

GAZEBO NEDİR?
------------
Gazebo, robotik simülasyonlar için kullanılan açık kaynaklı bir
simülatördür. 3D görselleştirme, fizik motoru ve sensör simülasyonu sağlar.

ROS2 LAUNCH SİSTEMİ:
-------------------
Launch dosyaları, birden fazla ROS2 node'unu ve parametreyi
tek bir komutla başlatmak için kullanılır.

KULLANIM:
--------
Terminal'den:
    ros2 launch hex_gz gazebo.launch.py

Klavye kontrolden:
    Gazebo başlatıldıktan sonra başka bir terminalde:
    ros2 run hexapod_klavye_kontrol klavye_kontrol

PARAMETRELER:
------------
    - world: Gazebo dünya dosyası (varsayılan: boş dünya)
    - x, y, z: Robot'un başlangıç pozisyonu
    - use_sim_time: Simülasyon zamanı kullan (True)

═══════════════════════════════════════════════════════════════════════════
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch dosyası oluşturma fonksiyonu.
    
    Bu fonksiyon ROS2 tarafından otomatik çağrılır ve
    başlatılacak tüm node'ları ve parametreleri döner.
    
    Returns:
        LaunchDescription: Başlatılacak işlemler listesi
    """
    
    # ═══════════════════════════════════════════════════════════════════
    # PAKET YOLLARINI BUL
    # ═══════════════════════════════════════════════════════════════════
    hex_gz_paketi = get_package_share_directory('hex_gz')
    hexapod_description_paketi = get_package_share_directory('hexapod_description')
    gazebo_ros_paketi = get_package_share_directory('gazebo_ros')
    
    # ═══════════════════════════════════════════════════════════════════
    # DOSYA YOLLARINI TANIMLA
    # ═══════════════════════════════════════════════════════════════════
    # Robot model dosyası (URDF/XACRO)
    robot_model_dosyasi = os.path.join(
        hexapod_description_paketi,
        'urdf',
        'hexapod.urdf.xacro'
    )
    
    # Gazebo dünya dosyası
    dunya_dosyasi = os.path.join(
        hex_gz_paketi,
        'worlds',
        'empty.world'  # Boş dünya (değiştirilebilir)
    )
    
    # Controller ayar dosyası
    controller_config = os.path.join(
        hexapod_description_paketi,
        'config',
        'controller_config.yaml'
    )
    
    # ═══════════════════════════════════════════════════════════════════
    # LAUNCH ARGÜMANLARI (Komut satırından değiştirilebilir)
    # ═══════════════════════════════════════════════════════════════════
    
    # Simülasyon zamanı kullan (her zaman True olmalı)
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Gazebo simülasyon zamanını kullan'
    )
    
    # Gazebo GUI göster mi?
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Gazebo GUI arayüzünü göster'
    )
    
    # Robot başlangıç pozisyonu X
    x_pozisyon_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Robot X pozisyonu (metre)'
    )
    
    # Robot başlangıç pozisyonu Y
    y_pozisyon_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Robot Y pozisyonu (metre)'
    )
    
    # Robot başlangıç pozisyonu Z
    z_pozisyon_arg = DeclareLaunchArgument(
        'z',
        default_value='0.5',
        description='Robot Z pozisyonu (yükseklik - metre)'
    )
    
    # ═══════════════════════════════════════════════════════════════════
    # ROBOT TANIMI (URDF/XACRO İŞLEME)
    # ═══════════════════════════════════════════════════════════════════
    # XACRO dosyasını URDF'ye çevir
    robot_tanimi = Command([
        'xacro ',
        robot_model_dosyasi,
        ' use_sim_time:=true'
    ])
    
    # ═══════════════════════════════════════════════════════════════════
    # GAZEBO SUNUCUSUNU BAŞLAT
    # ═══════════════════════════════════════════════════════════════════
    gazebo_sunucu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_ros_paketi, 'launch', 'gzserver.launch.py')
        ]),
        launch_arguments={
            'world': dunya_dosyasi,
            'verbose': 'true',
            'pause': 'false'
        }.items()
    )
    
    # ═══════════════════════════════════════════════════════════════════
    # GAZEBO İSTEMCİSİNİ BAŞLAT (GUI)
    # ═══════════════════════════════════════════════════════════════════
    gazebo_istemci = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_ros_paketi, 'launch', 'gzclient.launch.py')
        ]),
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    # ═══════════════════════════════════════════════════════════════════
    # ROBOT STATE PUBLISHER (Robot durumunu yayınla)
    # ═══════════════════════════════════════════════════════════════════
    # Bu node, robot eklemlerinin durumunu TF (transform) olarak yayınlar
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_tanimi,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # ═══════════════════════════════════════════════════════════════════
    # ROBOT'U GAZEBO'YA SPAWN ET (Dünyaya yerleştir)
    # ═══════════════════════════════════════════════════════════════════
    robot_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='hexapod_spawner',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'hexapod',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ]
    )
    
    # ═══════════════════════════════════════════════════════════════════
    # CONTROLLER MANAGER (Kontrolcüleri yönet)
    # ═══════════════════════════════════════════════════════════════════
    # Bu node, eklem kontrolcülerini başlatır ve yönetir
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_tanimi},
            controller_config
        ],
        output='screen'
    )
    
    # ═══════════════════════════════════════════════════════════════════
    # JOINT STATE BROADCASTER (Eklem durumlarını yayınla)
    # ═══════════════════════════════════════════════════════════════════
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ]
    )
    
    # ═══════════════════════════════════════════════════════════════════
    # BACAK KONTROLCÜLERİNİ BAŞLAT (Her bacak için ayrı)
    # ═══════════════════════════════════════════════════════════════════
    # Her bacak (leg1-leg6) için trajectory controller başlatılır
    
    leg1_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg1_controller', '--controller-manager', '/controller_manager']
    )
    
    leg2_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg2_controller', '--controller-manager', '/controller_manager']
    )
    
    leg3_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg3_controller', '--controller-manager', '/controller_manager']
    )
    
    leg4_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg4_controller', '--controller-manager', '/controller_manager']
    )
    
    leg5_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg5_controller', '--controller-manager', '/controller_manager']
    )
    
    leg6_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg6_controller', '--controller-manager', '/controller_manager']
    )
    
    # ═══════════════════════════════════════════════════════════════════
    # LAUNCH DESCRIPTION OLUŞTUR
    # ═══════════════════════════════════════════════════════════════════
    # Tüm node'ları ve argümanları birleştir
    
    return LaunchDescription([
        # Argümanlar
        sim_time_arg,
        gui_arg,
        x_pozisyon_arg,
        y_pozisyon_arg,
        z_pozisyon_arg,
        
        # Gazebo
        gazebo_sunucu,
        gazebo_istemci,
        
        # Robot
        robot_state_publisher,
        robot_spawner,
        
        # Kontrolcüler
        controller_manager,
        joint_state_broadcaster_spawner,
        
        # Bacak kontrolcüleri
        leg1_controller,
        leg2_controller,
        leg3_controller,
        leg4_controller,
        leg5_controller,
        leg6_controller,
    ])


"""
═══════════════════════════════════════════════════════════════════════════
KULLANIM ÖRNEKLERİ:
═══════════════════════════════════════════════════════════════════════════

1. Standart başlatma:
   ros2 launch hex_gz gazebo.launch.py

2. GUI olmadan (sadece simülasyon):
   ros2 launch hex_gz gazebo.launch.py gui:=false

3. Farklı pozisyonda başlat:
   ros2 launch hex_gz gazebo.launch.py x:=1.0 y:=2.0 z:=1.0

4. Farklı dünya dosyası ile:
   (gazebo.launch.py içinde world parametresini değiştir)

═══════════════════════════════════════════════════════════════════════════
NOTLAR:
═══════════════════════════════════════════════════════════════════════════

- Gazebo açıldıktan sonra robot otomatik yüklenir
- Eklem kontrolcüleri otomatik başlatılır
- Hareket scriptleri (tri_gate.py vs.) ayrı çalıştırılmalı
- Klavye kontrol başka bir terminalde çalıştırılır

SORUN GİDERME:
-------------

Eğer robot görünmüyorsa:
  - URDF dosyasını kontrol et
  - Spawn komutu hata verdi mi bak

Eğer eklemler hareket etmiyorsa:
  - Controller'lar başladı mı kontrol et
  - ros2 control list_controllers

═══════════════════════════════════════════════════════════════════════════
"""
