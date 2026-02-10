# Çoklu drone kontrolü ve kamera erişimi

## px4_5 neden var? Kaç drone var?

- **Script 4 drone spawn ediyor:** 1 lider + 3 iris → toplam 4. O zaman PX4 namespace’ler **px4_1, px4_2, px4_3, px4_4** olmalı (lider = px4_1, kamera ayrıca `/leader/...`).
- **Eğer px4_2, px4_3, px4_4, px4_5 görüyorsan:** 4 tane “numaralı” namespace var; **px4_1** listende yoksa büyük ihtimalle ilk araç (lider) **px4_1** veya namespace’siz `/fmu/...` kullanıyordur. px4_5’in görünmesi **5. bir PX4 instance** olduğu anlamına gelir → yani **5 drone** (1 lider + 4 işçi).
- **Neden 5 olabilir?** Spawn script’te fazladan bir blok (virgülle ayrılmış 5. parça) veya bir yerde script’in iki kez çalışması. Simülasyonu tam kapatıp **tek seferde** `./run_sim_four_drones.sh` çalıştır; çıktıda “4 spawn bloğu” gör. px4_5’i istemiyorsan sadece 4 araç spawn ettiğinden emin ol.

## Evet — hepsini istediğin gibi hareket ettirebilir, kamera verisine erişebilirsin

### Kontrol (her araç için)

Her PX4 instance'ının kendi **fmu** topic'leri var. Bir dronu hareket ettirmek için o aracın namespace'ine yayın yaparsın:

| Araç   | Offboard / setpoint / komut topic'leri      |
|--------|---------------------------------------------|
| 1 (lider) | `px4_1/fmu/in/offboard_control_mode`, `px4_1/fmu/in/trajectory_setpoint`, `px4_1/fmu/in/vehicle_command` |
| 2      | `px4_2/fmu/in/...`                         |
| 3      | `px4_3/fmu/in/...`                         |
| 4      | `px4_4/fmu/in/...`                         |

- **Offboard:** Sürekli `offboard_control_mode` + `trajectory_setpoint` yayınla.
- **Arm:** `vehicle_command` ile `VEHICLE_CMD_COMPONENT_ARM_DISARM`, `target_system` = o aracın MAV_SYS_ID’si (genelde px4_1 → 1, px4_2 → 2, …).
- **Pozisyon:** `trajectory_setpoint` ile NED (x, y, z) gönder.

Listede bazen ilk araç **px4_1** yerine namespace’siz `/fmu/in/...` olarak da görünebilir; 4 drone varsa px4_1, px4_2, px4_3, px4_4 (veya 2,3,4,5) kullan.

### Kamera (hepsine erişim)

| Araç   | Görüntü / bilgi topic'leri        |
|--------|-----------------------------------|
| Lider  | `/leader/camera/image_raw`, `/leader/camera/camera_info` |
| 2      | `/px4_2/camera/image_raw`, `/px4_2/camera/camera_info`   |
| 3      | `/px4_3/camera/...`               |
| 4      | `/px4_4/camera/...`               |

- Lider sabit: **`/leader/camera/image_raw`** (aşağı bakan keşif kamerası).
- İşçiler: **`/px4_N/camera/image_raw`**.

Daha önce **px4_2** kamera bazen görünmüyordu; iris modelinde plugin adı instance’a göre benzersiz yapıldı (`camera_controller_drone{{ mavlink_id }}`). Simülasyonu yeniden başlattığında px4_2 kamera da gelmeli.

### Özet

- **Hareket:** Evet — her araç için ilgili `px4_N/fmu/in/` topic’lerine yazarak hepsini ayrı ayrı kontrol edebilirsin.
- **Kamera:** Evet — lider için `/leader/camera/...`, diğerleri için `/px4_N/camera/...`; simü yeniden başlattıktan sonra hepsinde kamera topic’i olmalı.
