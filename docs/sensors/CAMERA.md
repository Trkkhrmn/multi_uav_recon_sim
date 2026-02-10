# Quadcopter kameraları

Simülasyon: `./scripts/run_sim_four_drones.sh` (4 lider iris_leader + 6 işçi iris).

- **Lider (iris_leader):** Kamera topic’leri:
  - `/leader_1/camera/image_raw`, `/leader_1/camera/camera_info` … `/leader_4/camera/...`
- **İşçi (iris):** `/px4_5/camera/...` … `/px4_10/camera/...`

## Özet

| Araç      | Kamera topic'leri |
|-----------|-------------------|
| Lider 1–4 | `/leader_1/camera/image_raw`, … `/leader_4/camera/...` |
| İşçi 5–10 | `/px4_5/camera/...`, … `/px4_10/camera/...` |

Görüntü izlemek: `ros2 run rqt_image_view rqt_image_view` → topic: `/leader_1/camera/image_raw` (veya diğer lider/işçi).
