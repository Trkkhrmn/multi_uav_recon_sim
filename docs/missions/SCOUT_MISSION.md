# Scout Mission — Lider Drone (satellite_scout_node)

Bu paket **satellite_scout_node** içerir: lider drone belirli bir bölge ortasına intikal eder, isteğe bağlı lawnmower/orbit tarama yapar. Normal akışta **multi_scout.launch.py** ile 4 drone için otomatik başlatılır.

## Gereksinimler

- ROS 2 (Humble veya Jellyfish)
- **px4_msgs**: PX4 ile ROS 2 iletişimi için gerekli. [px4_msgs](https://github.com/PX4/px4_msgs) paketini aynı workspace'e klonlayıp önce onu, sonra `scout_mission`'ı derleyin:
  ```bash
  cd ~/multi_uav_recon_ws/src && git clone https://github.com/PX4/px4_msgs.git
  cd ~/multi_uav_recon_ws && colcon build --packages-up-to scout_mission
  source install/setup.bash
  ```
- Simülasyon: PX4 SITL + Gazebo (Military Fortress).
- **Micro XRCE-DDS agent (zorunlu):** PX4, ROS 2 topic'lerini doğrudan dinlemez. Agent (UDP port 8888) PX4 ile ROS 2 DDS arasında köprü kurar. **Agent çalışmazsa scout_mission komutları PX4'e ulaşmaz ve drone kalkmaz.** Kurulum: `sudo snap install micro-xrce-dds-agent` veya [Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent) kaynaktan.

## Derleme

```bash
cd ~/multi_uav_recon_ws
colcon build --packages-select scout_mission
source install/setup.bash
```

## Çalıştırma (sıra önemli)

**Neden kalkmıyor?** Node sadece ROS 2 topic'lerine yazar. PX4 bu topic'leri **yalnızca Micro XRCE-DDS agent çalışıyorsa** alır. Önce agent'ı başlatın.

**Doğru sıra — 3 terminal:**

1. **Terminal 1 — Micro XRCE-DDS agent** (önce bunu başlat, açık kalsın):
   ```bash
   cd ~/multi_uav_recon_ws/scripts && ./run_microxrce_agent.sh
   ```
   Veya doğrudan: `MicroXRCEAgent udp4 -p 8888`

2. **Terminal 2 — Simülasyon** (Gazebo + 4× PX4 SITL):
   ```bash
   cd ~/multi_uav_recon_ws/scripts && ./run_sim_four_drones.sh
   ```

3. **Terminal 3 — MRCPP (multi_scout)** veya tek lider test için scout_mission:
   ```bash
   source ~/multi_uav_recon_ws/install/setup.bash
   ros2 launch multi_scout multi_scout.launch.py
   ```
   Tek lider test için: `ros2 run scout_mission satellite_scout_node --ros-args -p vehicle_id:=1`

## Parametreler

`satellite_scout_node` parametreleri (zone, irtifa, tarama modu vb.) için **docs/SATELLITE_SCOUT.md** dosyasına bakın. Ana parametreler: `vehicle_id`, `spawn_x_world`, `spawn_y_world`, `satellite_altitude`, `scan_zone_world_*`, `zone_center_world_*`, `scan_enable`, `patrol_mode`, `orbit_shape`.

## Topic'ler (vehicle_id=1 örneği)

- Yayın: `px4_1/fmu/in/offboard_control_mode`, `px4_1/fmu/in/trajectory_setpoint`, `px4_1/fmu/in/vehicle_command`
- Dinleme: `px4_1/fmu/out/vehicle_local_position`, `px4_1/fmu/out/vehicle_status`

Parametreler ve görev akışı detayı için **SATELLITE_SCOUT.md**'e bakın.
