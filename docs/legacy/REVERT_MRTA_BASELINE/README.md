# REVERT: MRTA revize öncesi baseline

**Bu klasör, MRTA detaylı revizyonundan (3 hedef tipi, paket kapasiteleri, tazeleme, COMPLETED) ÖNCEki halin yedeğidir.**

Aksilik çıkarsa yapılan tüm revizyon değişikliklerini silip **bu haline** geri dönebilirsin.

## Yedeklenen dosyalar

| Dosya | Geri yükle hedefi |
|-------|--------------------|
| `mrta_node.py` | `src/task_allocation/task_allocation/mrta_node.py` |
| `worker_drones.yaml` | `src/task_allocation/config/worker_drones.yaml` |
| `military_fortress.world.bak` | `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/military_fortress.world` (isteğe bağlı; alev/ağaç geri gelsin istersen) |

## Geri dönüş adımları

1. **MRTA kodu ve config:**
   ```bash
   cp multi_uav_recon_ws/docs/REVERT_MRTA_BASELINE/mrta_node.py \
      multi_uav_recon_ws/src/task_allocation/task_allocation/mrta_node.py
   cp multi_uav_recon_ws/docs/REVERT_MRTA_BASELINE/worker_drones.yaml \
      multi_uav_recon_ws/src/task_allocation/config/worker_drones.yaml
   ```

2. **World (alev/ağaç geri gelsin istersen):**
   ```bash
   cp multi_uav_recon_ws/docs/REVERT_MRTA_BASELINE/military_fortress.world.bak \
      PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/military_fortress.world
   ```

3. **Fusion / multi_scout** revizyonda değiştirildiyse, onların da eski sürümlerini git veya ayrı yedekten geri yükle.

4. `colcon build` ile tekrar derle.

---
*Yedek tarihi: Revizyon öncesi baseline (hedef tipleri, paket kapasiteleri, tazeleme, COMPLETED öncesi)*
