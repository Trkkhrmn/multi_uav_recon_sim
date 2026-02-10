# Satellite Scout — Lider uydu hover + orbit tarama

Lider drone, hedef nokta üstüne çıkar; ardından **80 m irtifada** hedef etrafında **daire**, **kare** veya **lawnmower (zigzag)** ile tarama yapar. **Lawnmower** merkez etrafında dikdörtgen alanı şerit şerit tarar (jet + tank bölgesini kaplamak için uygun).

## Dünya → PX4 yerel dönüşümü

- **Gazebo dünya:** Genelde X = Doğu (East), Y = Kuzey (North).
- **PX4 NED (yerel):** x = Kuzey, y = Doğu; orijin = drone’un home’u (spawn).
- Dönüşüm:
  - `local_x (North) = target_y_world - spawn_y_world`
  - `local_y (East)  = target_x_world - spawn_x_world`
  - `local_z = satellite_altitude` (NED: yukarı negatif, örn. -50 = 50 m)

Böylece “dünya koordinatı (X,Y)” doğru yere gider.

## Çalıştırma

```bash
source ~/multi_uav_recon_ws/install/setup.bash
ros2 run scout_mission satellite_scout_node
```

## Parametreler

| Parametre | Varsayılan | Açıklama |
|-----------|------------|----------|
| `vehicle_id` | 1 | Lider = 1 (px4_1) |
| `target_x_world` | -0.16 | Hedef Gazebo X (Doğu) |
| `target_y_world` | 0.33 | Hedef Gazebo Y (Kuzey) |
| `satellite_altitude` | -80.0 | Hover yüksekliği NED (m); 100 m FPS/grileşme yapıyorsa 80 m + geniş FOV kullan |
| `spawn_x_world` | 185.41 | Lider spawn X |
| `spawn_y_world` | 56.52 | Lider spawn Y |
| **Tarama zonu (düşman karesi)** | | |
| `use_scan_zone` | true | true = aşağıdaki kare kullanılır (tam alan, dışına çıkılmaz) |
| `scan_zone_world_min_x` | -70.44 | Köşe (Gazebo X = Doğu) |
| `scan_zone_world_max_x` | 77.17 | Köşe |
| `scan_zone_world_min_y` | -64.58 | Köşe (Gazebo Y = Kuzey) |
| `scan_zone_world_max_y` | 64.68 | Köşe |
| `orbit_scan_enable` | true | Hedefe ulaşınca tarama açık |
| `orbit_radius` | 60.0 | Daire/kare yarıçapı (m) |
| `orbit_shape` | **'lawnmower'** | **'lawnmower'** (zigzag, geniş alan), 'circle', 'square' |
| `orbit_direction` | 1 | Daire yönü: 1=CCW, -1=CW |
| `orbit_waypoints` | 48 | Daire/kare waypoint sayısı |
| `orbit_acceptance_radius` | 2.0 | Waypoint/segment uca bu kadar yaklaşınca sonrakine geç (m) |
| **Lawnmower** | | |
| `lawnmower_half_width_north` | 80.0 | Merkezden North yönünde yarı genişlik (m); tanklara yetecek kadar büyük tut |
| `lawnmower_half_height_east` | 80.0 | Merkezden East yönünde yarı yükseklik (m) |
| `lawnmower_lane_spacing` | 30.0 | Şerit arası (m) |
| `lawnmower_speed` | 1.1 | Zigzag hızı (m/s) |
| `orbit_control_mode` | 'velocity' | velocity = yumuşak hareket; circle ve lawnmower velocity ile çalışır |
| `orbit_speed` | 0.55 | Daire teğet hızı (m/s) |
| `orbit_altitude_kp` | 0.05 | İrtifa P (velocity modu) |

**Lawnmower:** Merkez (target) etrafında ±half_width_north (North) ve ±half_height_east (East) dikdörtgeni zigzag şeritlerle tarar. Jet ve tank bölgesini kaplamak için `lawnmower_half_width_north` / `lawnmower_half_height_east` değerlerini artır (örn. 80–100 m). **Daire** garip çiziyorsa `orbit_shape:=lawnmower` kullan veya `orbit_direction:=-1` ile yönü değiştir.

Örnek — farklı nokta, 80 m, küçük daire (yavaş tarama):

```bash
ros2 run scout_mission satellite_scout_node --ros-args \
  -p target_x_world:=10.0 -p target_y_world:=20.0 -p satellite_altitude:=-80.0 \
  -p orbit_radius:=20.0 -p orbit_waypoints:=64
```

Sadece sabit hover (orbit yok): `-p orbit_scan_enable:=false`

- **80 m + geniş FOV:** 100 m’de FPS düşüp grileşme oluyorsa, 80 m’de kalıp lider kameranın FOV’u artırıldı (≈120°) — aynı yükseklikte daha geniş zemin görünür.
- Daha da geniş alan istersen: `satellite_altitude:=-120` dene (FPS’e dikkat) veya kamera çözünürlüğünü düşür (modelde width/height).

## Eksenler ters gelirse

Eğer drone hedefin tam tersi yöne gidiyorsa, Gazebo haritanda X/Y farklı atanmış olabilir. O zaman `satellite_scout_node.py` içinde dönüşümü şöyle dene:

- `target_local_x = target_x_world - spawn_x_world` (X → North)
- `target_local_y = target_y_world - spawn_y_world` (Y → East)

veya bir parametre ile (ör. `world_xy_means_north_east`) seçilebilir hale getirilebilir.
