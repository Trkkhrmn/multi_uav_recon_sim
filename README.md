<!-- ============================================================
     Multi-UAV Reconnaissance & Task Allocation System
     ============================================================ -->

<!-- 📸 GÖRSEL ÖNERİSİ [BANNER]:
     Projenin genel görünümünü gösteren geniş bir kapak fotoğrafı.
     Gazebo'da 10 drone'un havadan görünümü (4 leader + 6 worker) veya
     RViz'de hedef noktaları ve zone sınırlarının gösterildiği bir ekran görüntüsü.
     Boyut: ~1280x400 px, docs/media/banner.png olarak kaydet.
-->
<!-- <p align="center">
  <img src="docs/media/banner.png" alt="Multi-UAV Recon Banner" width="100%"/>
</p> -->

<h1 align="center">Multi-UAV Reconnaissance & Task Allocation System</h1>

<p align="center">
  <strong>4 Keşif Dronu + 6 İşçi Dron | Otonom Hedef Tespiti, Sensör Füzyonu ve Görev Dağıtımı</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros&logoColor=white" alt="ROS 2 Humble"/>
  <img src="https://img.shields.io/badge/PX4-Autopilot-orange?logo=drone&logoColor=white" alt="PX4"/>
  <img src="https://img.shields.io/badge/Gazebo-Classic_11-green?logo=gazebo&logoColor=white" alt="Gazebo"/>
  <img src="https://img.shields.io/badge/Python-3.10-yellow?logo=python&logoColor=white" alt="Python"/>
  <img src="https://img.shields.io/badge/OpenCV-4.x-red?logo=opencv&logoColor=white" alt="OpenCV"/>
  <img src="https://img.shields.io/badge/License-MIT-brightgreen" alt="License"/>
</p>

<p align="center">
  <a href="#-demo">Demo</a> •
  <a href="#-özellikler">Özellikler</a> •
  <a href="#%EF%B8%8F-sistem-mimarisi">Mimari</a> •
  <a href="#-kurulum">Kurulum</a> •
  <a href="#-çalıştırma">Çalıştırma</a> •
  <a href="#-sonuçlar-ve-metrikler">Sonuçlar</a> •
  <a href="#-proje-yapısı">Yapı</a>
</p>

---

## Proje Hakkında

Simülasyon ortamında **10 otonom İHA** kullanarak uçtan uca bir keşif ve görev atama pipeline'ı sunan ROS 2 tabanlı bir projedir.

**4 lider (keşif) dronu** belirlenen bölgeler üzerinde uçarak aşağı bakan kamerasıyla mavi hedefleri tespit eder. Kamera görüntülerinden **piksel → 3B ışın → yer düzlemi kesişimi** ile hedef koordinatları hesaplanır. Merkezi bir **füzyon düğümü** 4 drone'dan gelen tespitleri birleştirip tekrarlı gözlemleri **centroid** ile ortalamalayarak tek bir hedef haritası üretir. Ardından **6 işçi dronu**, **MRTA (Multi-Robot Task Allocation)** algoritmasıyla bu hedeflere atanır: en yakın hedefe uçar, alçalır, yük bırakır ve geri döner.

Tüm iletişim **ROS 2 Humble** topic'leri üzerinden; uçuş kontrolü **PX4 SITL**, fizik ve dünya **Gazebo Classic 11** ile sağlanır. Gerçek donanım gerekmez.

<!-- 📸 GÖRSEL ÖNERİSİ [OVERVIEW GIF]:
     ~15-30 saniyelik bir GIF:
     1. Gazebo'da drone'lar kalkıyor
     2. Keşif bölgelerine gidiyor
     3. Hedefler tespit ediliyor (RViz'de yeşil noktalar beliriyor)
     4. İşçiler hedeflere uçuyor
     Kayıt: OBS veya Peek ile ekran kaydı alıp GIF'e çevir.
     docs/media/overview_demo.gif olarak kaydet.
-->
<!-- <p align="center">
  <img src="docs/media/overview_demo.gif" alt="Sistem Genel Demo" width="80%"/>
</p> -->

---

## Demo

<!-- 📸 GÖRSEL ÖNERİSİ [DEMO]:
     Aşağıdaki 4 görsel yan yana veya 2x2 grid şeklinde yerleştirilebilir.
     Her birini docs/media/ klasörüne kaydet.

     1. docs/media/gazebo_world.png
        → Gazebo'da haritanın genel görünümü (10 drone spawn hali)

     2. docs/media/rviz_fused_targets.png
        → RViz'de fused_targets (yeşil kutular) + zone sınırları (çizgiler)
        → Frame: world, /scout/fused_targets_markers + /scout/zone_boundaries gösterilmeli

     3. docs/media/rviz_mrta_markers.png
        → RViz'de MRTA marker'ları: yeşil hedef, sarı atama çizgileri, beyaz drone etiketleri
        → /mrta/markers topic'i

     4. docs/media/camera_detection.png
        → rqt_image_view'dan bir leader kamerası görüntüsü
        → Mavi hedefler tespit edilmiş (bounding box veya contour çizili)
-->

| Gazebo Simülasyonu | RViz — Fused Targets |
|:---:|:---:|
| ![Gazebo](docs/media/gazebo_world.png) | ![RViz Targets](docs/media/rviz_fused_targets.png) |

| MRTA Görev Atama | Kamera Tespiti |
|:---:|:---:|
| ![MRTA](docs/media/rviz_mrta_markers.png) | ![Kamera](docs/media/camera_detection.png) |

<!-- 📸 GÖRSEL ÖNERİSİ [FULL DEMO GIF]:
     Tüm pipeline'ın çalıştığını gösteren 30-60 saniyelik ekran kaydı GIF.
     Gazebo + RViz yan yana split screen kaydı ideal.
     docs/media/full_pipeline_demo.gif
-->
<!-- <p align="center">
  <img src="docs/media/full_pipeline_demo.gif" alt="Full Pipeline Demo" width="90%"/>
</p> -->

---

## Özellikler

| Özellik | Açıklama |
|---------|----------|
| **Çoklu Robot Kapsama (MRCPP)** | Harita 4 bölgeye ayrılır; her lider drone kendi bölgesini kapsar (uydu modu, lawnmower, daire tarama) |
| **Kamera Tabanlı Hedef Tespiti** | HSV renk segmentasyonu + kontur analizi + `cv2.moments()` ile alt-piksel merkez hesaplama |
| **Piksel → Dünya Koordinatı** | Pinhole kamera modeli (`image_geometry`) ile 3B ışın, yer düzlemi kesişimi, NED → dünya dönüşümü |
| **Merkezi Sensör Füzyonu** | 4 drone'dan gelen tespitler mesafe bazlı birleştirilir (merge radius + centroid), min. gözlem filtresi |
| **MRTA Görev Dağıtımı** | Greedy nearest-first atama; 6 işçi drone durum makinesi (ARM → CLIMB → GO → DESCEND → DROP → RTH) |
| **Yük Bırakma** | Gazebo `/spawn_entity` servisi ile hedef noktaya model spawn edilir |
| **Canlı Görselleştirme** | RViz MarkerArray (hedefler, atama çizgileri, drone durumları), 2D fused map image, MRTA panel |
| **Kapasite ve İkmal** | İşçilere farklı paket kapasitesi ve hız atanır; paket bitince üsse dönüp ikmal yapılır |
| **Senaryo Analizi** | `scenario_analysis.py` ile ground truth vs tespit karşılaştırması, RMSE/hata grafikleri |

---

## Sistem Mimarisi

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          GAZEBO CLASSIC + PX4 SITL                         │
│                                                                             │
│   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐      │
│   │  Leader 1    │  │  Leader 2    │  │  Leader 3    │  │  Leader 4    │      │
│   │  (iris_leader)│  │  (iris_leader)│  │  (iris_leader)│  │  (iris_leader)│      │
│   │  ↓ camera    │  │  ↓ camera    │  │  ↓ camera    │  │  ↓ camera    │      │
│   └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘      │
│          │                 │                 │                 │              │
│   ┌──────┴─────┐  ┌──────┴─────┐  ┌──────┴─────┐  ┌──────┴─────┐          │
│   │  Worker 5  │  │  Worker 6  │  │  Worker 7  │  │ Worker 8-10│          │
│   │  (iris)    │  │  (iris)    │  │  (iris)    │  │  (iris)    │          │
│   └────────────┘  └────────────┘  └────────────┘  └────────────┘          │
└───────────────────────────────┬─────────────────────────────────────────────┘
                                │ UDP 8888
                    ┌───────────▼───────────┐
                    │  Micro XRCE-DDS Agent  │
                    └───────────┬───────────┘
                                │ DDS ↔ ROS 2
┌───────────────────────────────▼─────────────────────────────────────────────┐
│                              ROS 2 HUMBLE                                   │
│                                                                             │
│  ┌─────────────────┐     ┌──────────────────────┐                          │
│  │  scout_mission   │     │  map_object_detector  │                          │
│  │  (4× satellite   │────▶│  (4× blue_target      │                          │
│  │   scout_node)    │     │   mapper)              │                          │
│  │                  │     │  HSV → contour → ray   │                          │
│  │  Offboard kontrol│     │  → world (x,y)         │                          │
│  └─────────────────┘     └──────────┬───────────┘                          │
│                                      │ /scout/detections_{1..4}             │
│                           ┌──────────▼───────────┐                          │
│                           │     multi_scout       │                          │
│                           │  ┌─────────────────┐  │                          │
│                           │  │  fusion_node     │  │  merge radius + centroid │
│                           │  │  zone_boundaries │  │  RViz zone çizgileri     │
│                           │  │  fused_map_image │  │  2D harita görüntüsü     │
│                           │  │  recorder_node   │  │  YAML kayıt             │
│                           │  └─────────────────┘  │                          │
│                           └──────────┬───────────┘                          │
│                                      │ /scout/fused_targets (PoseArray)     │
│                           ┌──────────▼───────────┐                          │
│                           │   task_allocation     │                          │
│                           │   (MRTA node)         │                          │
│                           │   Greedy nearest-first│                          │
│                           │   State machine:      │                          │
│                           │   IDLE→ARM→CLIMB→GO   │                          │
│                           │   →DESCEND→DROP→RTH   │                          │
│                           └──────────────────────┘                          │
└─────────────────────────────────────────────────────────────────────────────┘
```

<!-- 📸 GÖRSEL ÖNERİSİ [MİMARİ DİYAGRAM]:
     Yukarıdaki ASCII diyagramı daha profesyonel göstermek istersen,
     draw.io veya Figma'da blok diyagram çizip
     docs/media/architecture_diagram.png olarak kaydet.
-->

### Veri Akışı (ROS 2 Topic'leri)

```
Lider Drone'lar (PX4 → ROS 2)
├── px4_{1..4}/fmu/out/vehicle_local_position
├── px4_{1..4}/fmu/out/vehicle_status
├── px4_{1..4}/fmu/out/vehicle_attitude
└── leader_{1..4}/camera/image_raw + camera_info

Tespit (map_object_detector)
└── /scout/detections_{1..4}  (PointStamped, world frame)

Füzyon (multi_scout)
├── /scout/fused_targets          (PoseArray)
├── /scout/fused_targets_markers  (MarkerArray — RViz)
├── /scout/zone_boundaries        (MarkerArray — RViz)
└── /scout/fused_map_image        (Image — 2D harita)

İşçi Drone'lar (MRTA → PX4)
├── px4_{5..10}/fmu/in/offboard_control_mode
├── px4_{5..10}/fmu/in/trajectory_setpoint
├── px4_{5..10}/fmu/in/vehicle_command
├── /mrta/markers                 (MarkerArray — RViz)
└── /mrta/status                  (String — terminal)
```

---

## Algoritmalar ve Yöntemler

### 1. Piksel → 3B Işın (Unprojection)
`CameraInfo` mesajından gelen **K matrisi** (intrinsic) ve `image_geometry.PinholeCameraModel` kullanılarak, tespit edilen piksele ait birim yön vektörü (3B ışın) kamera optik çerçevesinde hesaplanır.

### 2. Kamera Optik → Body NED Dönüşümü
Aşağı bakan kamera için sabit bir rotasyon matrisi tanımlanmıştır:
```
Optik Z (ileri) → Body Z (aşağı)
Optik X (sağ)   → Body Y (doğu)
-Optik Y (aşağı) → Body X (kuzey)
```

### 3. Yer Düzlemi Kesişimi
- Drone pozisyonu (PX4 local NED) ve tutumu (quaternion) ile ışın NED'e dönüştürülür
- **z = 0** yer düzlemi ile kesişim: `t = -z_drone / ray_z`
- Lokal NED → Dünya: spawn offset eklenerek ortak (X, Y) çerçevesine geçilir

### 4. Alt-Piksel Tespit Merkezi
`cv2.moments(contour)` ile **centroid** hesaplanır: `u = M10/M00`, `v = M01/M00` — bounding box merkezinden daha hassas.

### 5. Renk Tabanlı Hedef Tespiti
HSV renk uzayında mavi hedefler (H: 100–140) segmente edilir. Morfolojik açma ile gürültü temizlenir; kontur alanı ve en-boy oranı filtreleri uygulanır.

### 6. Merkezi Harita Füzyonu
4 drone'dan gelen tespitler `merge_radius` (ör. 4 m) içinde ise **aynı hedef** kabul edilir ve pozisyon tüm gözlemlerin **centroid**'i olarak güncellenir. Yalnızca `min_observations` kadar görülen hedefler son haritaya dahil edilir.

<!-- 📸 GÖRSEL ÖNERİSİ [ALGORİTMA AÇIKLAMA]:
     Piksel → Dünya dönüşümünü gösteren bir diyagram çok etkileyici olur:
     Kamera görüntüsü → 3D ışın → yer düzlemi kesişimi → world (x,y)
     docs/media/pixel_to_world_diagram.png
-->

---

## Kurulum

### Gereksinimler

| Bileşen | Sürüm / Not |
|---------|-------------|
| **İşletim Sistemi** | Ubuntu 22.04 LTS |
| **ROS 2** | Humble Hawksbill |
| **PX4-Autopilot** | SITL + Gazebo Classic |
| **Gazebo** | Classic 11 |
| **Python** | 3.10+ |
| **OpenCV** | 4.x (`pip install opencv-python`) |
| **NumPy** | `pip install numpy` |
| **Matplotlib** | `pip install matplotlib` (senaryo analizi için) |

### Adım Adım Kurulum

**1. Workspace'i klonla:**
```bash
git clone https://github.com/<KULLANICI_ADI>/multi_uav_recon_ws.git
cd multi_uav_recon_ws
```

**2. PX4-Autopilot kur ve Gazebo Classic submodule'ünü başlat:**
```bash
# PX4 klonla (eğer yoksa)
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Gazebo Classic submodule
git submodule update --init Tools/simulation/gazebo-classic/sitl_gazebo-classic

# SITL build
make px4_sitl_default sitl_gazebo-classic
```

**3. px4_msgs paketini workspace'e ekle:**
```bash
cd ~/multi_uav_recon_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ..
```

**4. Gazebo model asset'lerini bağla:**
```bash
./scripts/setup_assets.sh
```
> `assets/` klasörü oluşturulur (symlink). Gazebo harita modelleri (`common_models`, `mili_tech` vb.) `~/gazebo_maps` altında olmalıdır. Farklı bir yol kullanıyorsan `GAZEBO_MAPS_DIR` değişkenini düzenle.

**5. Workspace'i derle:**
```bash
# ROS 2 ortamını yükle
source /opt/ros/humble/setup.bash

# Paketleri derle
colcon build --packages-up-to multi_scout
colcon build --packages-select task_allocation

# Ortamı kaynak et
source install/setup.bash
```

---

## Çalıştırma

Sistemi çalıştırmak için **4 ayrı terminal** gerekir. Sıralama önemlidir.

### Hızlı Başlangıç

Tüm komutları görmek için:
```bash
./scripts/run_demo.sh
```

### Adım Adım Çalıştırma

<!-- 📸 GÖRSEL ÖNERİSİ [TERMINAL]:
     4 terminali aynı anda gösteren bir screenshot:
     tmux veya terminator ile 4 panel açıp komutları çalıştır.
     docs/media/terminals_running.png
-->

**Terminal 1 — Micro XRCE-DDS Agent** (PX4 ↔ ROS 2 köprüsü):
```bash
./scripts/run_microxrce_agent.sh
# veya: MicroXRCEAgent udp4 -p 8888
```

**Terminal 2 — Simülasyon** (Gazebo + 10 PX4 instance):
```bash
./scripts/run_sim_four_drones.sh
# GUI olmadan: ./scripts/run_sim_four_drones.sh headless
```

**Terminal 3 — Keşif + Füzyon** (4 scout + 4 mapper + fusion + viz):
```bash
source install/setup.bash
ros2 launch multi_scout multi_scout.launch.py
```

**Terminal 4 — MRTA** (görev dağıtımı — füzyon hedef yayınlamaya başladıktan sonra):
```bash
source install/setup.bash
ros2 launch task_allocation mrta.launch.py
```

### Opsiyonel

```bash
# MRTA canlı panel
ros2 run task_allocation mrta_panel

# Kamera/harita görüntüsü
ros2 run rqt_image_view rqt_image_view

# RViz ile MRTA marker'larını görüntüle
rviz2 -d src/task_allocation/config/mrta.rviz

# MRTA durumunu terminalde izle
ros2 topic echo /mrta/status
```

### RViz Marker Renk Kodları

| Renk | Anlam |
|------|-------|
| **Yeşil kutu** | Henüz ziyaret edilmemiş hedef |
| **Gri kutu** | Yük bırakılmış (tamamlanmış) hedef |
| **Sarı çizgi** | Drone → atanmış hedef bağlantısı |
| **Beyaz etiket** | Drone durumu (ör. `W5: GO`, `W7: DESCEND`) |

---

## Sonuçlar ve Metrikler

Proje, **4 farklı senaryo** üzerinde test edilmiştir. Ground truth koordinatları ile tespit edilen koordinatlar karşılaştırılarak doğruluk ölçülmüştür.

<!-- 📸 GÖRSEL ÖNERİSİ [GRAFİKLER]:
     scenario_analysis.py scriptini çalıştırarak grafikleri oluştur:
       cd multi_uav_recon_ws
       python3 scripts/scenario_analysis.py
     Çıktılar output/figures/ klasöründe oluşur.
     Bu grafikleri docs/media/ altına kopyala:

     1. docs/media/figures/01_gt_vs_detected_xy.png
        → Ground Truth vs Detected (X ve Y koordinatları)
     2. docs/media/figures/02_error_vectors_2d.png
        → 2D hata vektörleri (GT → Detected okları)
     3. docs/media/figures/03_error_histogram.png
        → Öklidyen hata dağılımı histogramı
     4. docs/media/figures/04_error_by_scenario_boxplot.png
        → Senaryoya göre hata boxplot
     5. docs/media/figures/05_rmse_mean_per_scenario.png
        → Senaryo başına RMSE ve ortalama hata
     6. docs/media/figures/06_xy_error_by_scenario.png
        → X ve Y bileşen hataları (senaryoya göre)
-->

### Genel Performans

| Metrik | Değer |
|--------|-------|
| **Toplam test noktası** | 55 (4 senaryo) |
| **Ortalama Öklidyen hata** | 0.808 m |
| **RMSE** | 0.930 m |
| **Maksimum hata** | 2.812 m |

| Senaryo | Nokta Sayısı | Ortalama Hata | RMSE |
|---------|:---:|:---:|:---:|
| Senaryo 1 | 13 | 0.982 m | 1.189 m |
| Senaryo 2 | 14 | 0.765 m | 0.840 m |
| Senaryo 3 | 14 | 0.775 m | 0.861 m |
| Senaryo 4 | 14 | 0.722 m | 0.797 m |

> Not: Değerler `scripts/scenario_analysis.py` çalıştırılarak doğrulanabilir.

### Grafikler

Grafikleri oluşturmak için:
```bash
python3 scripts/scenario_analysis.py
# Çıktılar: output/figures/
```

| Ground Truth vs Detected | Hata Vektörleri (2D) |
|:---:|:---:|
| ![GT vs Det](docs/media/figures/01_gt_vs_detected_xy.png) | ![Error Vectors](docs/media/figures/02_error_vectors_2d.png) |

| Hata Dağılımı | Senaryoya Göre Hata |
|:---:|:---:|
| ![Histogram](docs/media/figures/03_error_histogram.png) | ![Boxplot](docs/media/figures/04_error_by_scenario_boxplot.png) |

| RMSE / Ortalama Hata | X-Y Bileşen Hataları |
|:---:|:---:|
| ![RMSE](docs/media/figures/05_rmse_mean_per_scenario.png) | ![XY Error](docs/media/figures/06_xy_error_by_scenario.png) |

---

## Proje Yapısı

```
multi_uav_recon_ws/
├── README.md                          # Bu dosya
├── .gitignore
│
├── scripts/                           # Giriş noktaları ve yardımcı scriptler
│   ├── run_demo.sh                    # Tüm komutları gösteren yardımcı
│   ├── run_microxrce_agent.sh         # DDS agent başlatma
│   ├── run_sim_four_drones.sh         # Gazebo + 10 PX4 (4 leader + 6 worker)
│   ├── setup_assets.sh                # Gazebo model symlink'leri
│   ├── setup.sh                       # px4_msgs kurulumu
│   └── scenario_analysis.py           # GT vs tespit analiz & grafik üretimi
│
├── docs/                              # Dokümantasyon
│   ├── overview/                      # Proje açıklamaları, CV/LinkedIn metni
│   ├── architecture/                  # MRTA görselleştirme, çoklu drone kontrol
│   ├── missions/                      # Scout/satellite scout parametreleri
│   ├── sensors/                       # Kamera kurulumu
│   ├── legacy/                        # Eski baseline yedekleri
│   └── media/                         # README görselleri (GIF, PNG)
│       └── figures/                   # Analiz grafikleri
│
├── output/                            # Üretilen dosyalar (gitignore)
│   └── .gitkeep
│
└── src/                               # ROS 2 paketleri
    ├── scout_mission/                 # Lider drone uçuş kontrolü
    │   └── satellite_scout_node.py    # Zone'a git, hover/orbit/lawnmower tarama
    │
    ├── map_object_detector/           # Kamera tabanlı hedef tespiti
    │   └── blue_target_mapper.py      # HSV → contour → piksel→dünya dönüşümü
    │
    ├── multi_scout/                   # Keşif katmanı (launch + füzyon + viz)
    │   ├── launch/
    │   │   └── multi_scout.launch.py  # 4 scout + 4 mapper + fusion + zone + recorder
    │   ├── config/
    │   │   ├── coverage_partitions.yaml   # 4 drone: spawn, zone, merkez
    │   │   └── fusion_params.yaml         # merge_radius, min_observations
    │   ├── fusion_node.py             # Merkezi sensör füzyonu
    │   ├── zone_boundaries_node.py    # RViz zone sınır çizgileri
    │   ├── fused_map_image_node.py    # 2D top-down harita görüntüsü
    │   └── detected_targets_recorder_node.py  # Tespit kayıt (YAML)
    │
    └── task_allocation/               # MRTA görev dağıtımı
        ├── launch/
        │   └── mrta.launch.py         # MRTA node başlatma
        ├── config/
        │   ├── worker_drones.yaml     # 6 worker: spawn, kapasite, hız
        │   └── mrta.rviz              # RViz konfigürasyonu
        ├── mrta_node.py               # Greedy nearest-first atama + durum makinesi
        └── mrta_panel.py              # Opsiyonel canlı MRTA paneli
```

> **Not:** `px4_msgs` bu repoya dahil değildir. [Kurulum](#-kurulum) bölümündeki adımları takip ederek `src/` altına klonlayın. `assets/` klasörü `setup_assets.sh` tarafından oluşturulur ve gitignore edilmiştir.

---

## Koordinat Sistemi

Bu projede 3 farklı koordinat çerçevesi kullanılır:

| Çerçeve | Eksenler | Orijin | Kullanım |
|---------|----------|--------|----------|
| **Gazebo World** | X = Doğu, Y = Kuzey, Z = Yukarı | Harita orijini | Hedef konumları, zone sınırları |
| **PX4 Local NED** | X = Kuzey, Y = Doğu, Z = Aşağı | Her drone'un spawn noktası | Uçuş setpoint'leri |
| **Kamera Optik** | Z = İleri, X = Sağ, Y = Aşağı | Kamera merceği | Piksel → 3B ışın |

**Dönüşüm formülleri:**
```
World → Local NED:
  local_north = world_y - spawn_y
  local_east  = world_x - spawn_x

Local NED → World:
  world_x = spawn_x + local_east
  world_y = spawn_y + local_north
```

---

## Yapılandırma

### Keşif Drone'ları (`coverage_partitions.yaml`)

| Drone | Zone | Merkez (X, Y) | Spawn (X, Y) |
|-------|------|---------------|---------------|
| Leader 1 | Sağ Üst | (38.59, 32.34) | (185.41, 56.52) |
| Leader 2 | Sol Üst | (-35.22, 32.34) | (189.41, 56.52) |
| Leader 3 | Sol Alt | (-35.22, -32.29) | (189.41, 60.52) |
| Leader 4 | Sağ Alt | (38.59, -32.29) | (185.41, 60.52) |

### İşçi Drone'ları (`worker_drones.yaml`)

| Drone | ID | Paket Kapasitesi | Hız Çarpanı | Tipi |
|-------|----|-----------------|-------------|------|
| Worker 5 | 5 | 3 | 0.5x (yavaş) | Ağır yük |
| Worker 6 | 6 | 3 | 0.5x (yavaş) | Ağır yük |
| Worker 7 | 7 | 2 | 0.7x (orta) | Orta yük |
| Worker 8 | 8 | 2 | 0.7x (orta) | Orta yük |
| Worker 9 | 9 | 1 | 1.0x (hızlı) | Hafif yük |
| Worker 10 | 10 | 1 | 1.0x (hızlı) | Hafif yük |

---

## İşçi Drone Durum Makinesi

```
                    ┌──────────────────────────────┐
                    │                              ▼
┌──────┐    ┌──────────┐    ┌───────┐    ┌─────┐    ┌──────────┐
│ IDLE │───▶│ ARMING   │───▶│ CLIMB │───▶│ GO  │───▶│ DESCEND  │
└──────┘    └──────────┘    └───────┘    └─────┘    └────┬─────┘
   ▲                                                      │
   │                                                      ▼
   │        ┌──────────────┐    ┌───────────────┐    ┌────────┐
   │        │ CLIMB_AFTER  │◀───│  AFTER_DROP   │◀───│  DROP  │
   │        │    _DROP     │    └───────────────┘    └────────┘
   │        └──────┬───────┘
   │               │
   │    ┌──────────▼─────────┐    ┌──────────────┐
   │    │ Yeni hedef var mı? │───▶│ RETURN_TO    │
   │    │   Evet → ARMING    │    │    _BASE     │
   │    └────────────────────┘    └──────┬───────┘
   │                                      │
   │              ┌────────────┐          │
   └──────────────│ RTH_LANDED │◀─────────┘
                  └────────────┘
```

---

## Teknoloji Yığını

<p align="center">
  <img src="https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white"/>
  <img src="https://img.shields.io/badge/PX4-Autopilot-F05032?style=for-the-badge&logo=drone&logoColor=white"/>
  <img src="https://img.shields.io/badge/Gazebo-Classic_11-E88C1F?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white"/>
  <img src="https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white"/>
  <img src="https://img.shields.io/badge/NumPy-013243?style=for-the-badge&logo=numpy&logoColor=white"/>
  <img src="https://img.shields.io/badge/Matplotlib-11557c?style=for-the-badge"/>
</p>

---

## Bilinen Kısıtlamalar

- Tek dünya haritası (askeri kale stili); sabit spawn pozisyonları ve zone düzeni
- MRTA greedy (en yakın hedef) — optimal rotalama yok
- Öncelik bazlı atama veya arıza durumunda yeniden atama yok
- Hedef tespiti renk bazlı; karmaşık ortamlarda false positive olabilir

## Gelecek Çalışmalar

- [ ] Öncelik bazlı MRTA (hedef tiplerine göre farklı öncelik)
- [ ] Arıza durumunda yeniden atama (worker failure → reassignment)
- [ ] Dinamik hedef ekleme (simülasyon sırasında yeni hedef spawn)
- [ ] Farklı dünya haritaları ve yapılandırılabilir zone düzenleri
- [ ] SLAM tabanlı hedef tespiti (renk yerine öznitelik bazlı)
- [ ] Optimal rotalama algoritmaları (TSP / VRP tabanlı)

---

## Dokümantasyon

| Klasör | İçerik |
|--------|--------|
| `docs/overview/` | Proje açıklaması, CV/LinkedIn özeti |
| `docs/architecture/` | MRTA görselleştirme, çoklu drone kontrol |
| `docs/missions/` | Scout ve satellite scout parametreleri |
| `docs/sensors/` | Kamera kurulumu ve topic'ler |
| `docs/legacy/` | Eski baseline yedekleri (referans) |

Paket bazlı README dosyaları: `src/<paket_adı>/README.md`

---

## Referanslar

<details>
<summary><strong>Akademik referanslar ve kaynaklar</strong> (genişletmek için tıkla)</summary>

### Kamera Modeli ve Projeksiyon
1. **R. Hartley & A. Zisserman**, *Multiple View Geometry in Computer Vision*, 2nd ed., Cambridge University Press, 2003.
2. **ROS image_geometry**, [PinholeCameraModel](https://docs.ros.org/en/api/image_geometry/html/python/)

### Dönüşümler (Quaternion, Koordinat Çerçeveleri)
3. **J. Diebel**, "Representing attitude: Euler angles, unit quaternions, and rotation vectors," Stanford University, 2006.
4. **PX4 Development Guide**, [Coordinate Frames](https://docs.px4.io/main/en/coordinate_frames/README.html)

### Çoklu Robot Kapsama
5. **H. Choset**, "Coverage of known spaces: The boustrophedon cellular decomposition," *Autonomous Robots*, 9(3), 2000.
6. **E. Galceran & M. Carreras**, "A survey on coverage path planning for robotics," *Robotics and Autonomous Systems*, 61(12), 2013.

### Veri Füzyonu
7. **S. Thrun, W. Burgard, D. Fox**, *Probabilistic Robotics*, MIT Press, 2005.

### Görüntü İşleme
8. **R. Szeliski**, *Computer Vision: Algorithms and Applications*, 2nd ed., Springer, 2022.
9. **OpenCV Documentation**, [Structural Analysis and Shape Descriptors](https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html)

</details>

---

## Lisans

Bu proje [MIT Lisansı](LICENSE) ile lisanslanmıştır.

---

<p align="center">
  <sub>Geliştirici: <strong><a href="https://github.com/<KULLANICI_ADI>">Tarık</a></strong></sub><br/>
  <sub>Herhangi bir soru veya öneriniz varsa <a href="https://github.com/<KULLANICI_ADI>/multi_uav_recon_ws/issues">Issue</a> açabilirsiniz.</sub>
</p>