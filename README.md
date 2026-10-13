# Autonomous Drone Development: Raspberry Pi 4 & Pixhawk PIX6

Repositori ini berisi panduan, konfigurasi, dan kumpulan skrip Python berbasis **PyMAVLink** untuk membangun sistem drone otonom menggunakan **Raspberry Pi 4** sebagai *companion computer* dan **Pixhawk PIX6** (ArduCopter) sebagai *flight controller*.

Dokumentasi ini mencakup langkah-langkah yang telah divalidasi pada pengujian lingkungan **Indoor / Laboratorium** tanpa sinyal GPS (menggunakan sensor internal Barometer & IMU).

---

> [!WARNING]
> ### Protokol Keselamatan (Safety First)
> **SELALU LEPAS SEMUA BALING-BALING (PROPELLER)** saat melakukan pengujian skrip di atas meja kerja (*bench test*). Jangan memasang propeller sampai seluruh fungsi otomatisasi dan prosedur darurat terverifikasi 100%.

---

## 🛠️ Spesifikasi Sistem

| Komponen | Spesifikasi / Konfigurasi |
|:---|:---|
| **Companion Computer** | Raspberry Pi 4 |
| **Flight Controller** | Pixhawk PIX6 (Firmware: ArduCopter Quad) |
| **Jalur Komunikasi** | Kabel Data USB (`/dev/ttyACM0` - Baudrate default MAVLink) |
| **Lingkungan Uji** | Indoor / Laboratorium (Tanpa GPS, Barometer + IMU aktif) |
| **Bahasa Pemrograman** | Python 3 (Virtual Environment: `~/venvs/uav`) |
| **Protokol** | MAVLink melalui pustaka `pymavlink` |

---

## 📦 1. Persiapan Lingkungan (Setup Environment)

Jalankan perintah berikut di terminal Raspberry Pi:

### A. Izin Akses Serial Port
Berikan izin akses port serial kepada user sistem:
```bash
sudo usermod -a -G dialout $USER
```
*(Lakukan restart atau relogin jika baru pertama kali menambahkan grup).*

### B. Aktivasi Virtual Environment & Instalasi Library
```bash
# Aktivasi virtual environment
source ~/venvs/uav/bin/activate

# Instalasi PyMAVLink
pip install pymavlink
```

---

## 🚀 2. Tahapan Pengujian yang Telah Tervalidasi

### Modul 1: Diagnostik Koneksi & Verifikasi Identitas
**File:** `01_pixhawk_diagnostic.py`  
Memastikan Raspberry Pi berhasil membaca *heartbeat*, mengonfirmasi `System ID: 1`, `Component ID: 1`, serta memastikan tipe wahana dikenali sebagai ArduCopter Quad.

```python
import time
from pymavlink import mavutil

PORT = "/dev/ttyACM0"
master = mavutil.mavlink_connection(PORT)

print("Menghubungkan ke PIX6...")
heartbeat = master.wait_heartbeat(timeout=15)
if heartbeat is None:
    raise TimeoutError("Tidak ada heartbeat yang diterima dari Flight Controller!")

print("\n==============================")
print("   PIX6 CONNECTION SUCCESS")
print("==============================")
print("System ID      :", heartbeat.get_srcSystem())
print("Component ID   :", heartbeat.get_srcComponent())
print("Autopilot Type :", heartbeat.autopilot)  # 3 = ArduPilot
print("Vehicle Type   :", heartbeat.type)       # 2 = Quadcopter
print("Flight Mode    :", master.flightmode)
```

**Eksekusi:**
```bash
python3 01_pixhawk_diagnostic.py
```

---

### Modul 2: Pembacaan Telemetri Orientasi (Attitude)
**File:** `02_pixhawk_attitude.py`  
Membaca data sensor IMU secara real-time (*Roll, Pitch, Yaw*) melalui pesan MAVLink `ATTITUDE` dengan interval frekuensi 5 Hz.

```python
import math
import time
from pymavlink import mavutil

PORT = "/dev/ttyACM0"
master = mavutil.mavlink_connection(PORT)

print("Menghubungkan ke PIX6...")
heartbeat = master.wait_heartbeat(timeout=15)
print(f"PIX6 Terhubung! System: {heartbeat.get_srcSystem()}, Component: {heartbeat.get_srcComponent()}")

# Meminta data ATTITUDE dikirim pada frekuensi 5 Hz (200.000 mikrodetik)
master.mav.command_long_send(
    master.target_system,
    heartbeat.get_srcComponent(),
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    200000,
    0, 0, 0, 0, 0
)

print("\nMembaca orientasi real-time (Ctrl+C untuk berhenti)...\n")

try:
    while True:
        msg = master.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
        if msg:
            roll_deg = math.degrees(msg.roll)
            pitch_deg = math.degrees(msg.pitch)
            yaw_deg = math.degrees(msg.yaw)
            print(f"Roll: {roll_deg:6.2f}° | Pitch: {pitch_deg:6.2f}° | Yaw: {yaw_deg:6.2f}°", end="\r")
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nPembacaan dihentikan.")
```

**Eksekusi:**
```bash
python3 02_pixhawk_attitude.py
```

---

### Modul 3: Mode Flight `ALT_HOLD` & Pengujian Arm/Disarm (No RC)
**File:** `03_arm_disarm_althold.py`  
Mengalihkan mode penerbangan dari `STABILIZE` ke `ALT_HOLD` (Altitude Hold), lalu mengeksekusi perintah ARM motor selama 20 detik, dan diakhiri dengan DISARM otomatis tanpa menggunakan pemancar Remote Control fisik.

```python
import time
from pymavlink import mavutil

PORT = "/dev/ttyACM0"
ARM_TIME = 20

print("======================================")
print("   PIX6 ARM TEST - ALT_HOLD (NO RC)")
print("======================================")
print("PASTIKAN PROPELLER SUDAH DILEPAS!\n")

master = mavutil.mavlink_connection(PORT)

print("Menghubungkan ke PIX6...")
master.wait_heartbeat()
print("PIX6 Terhubung.")
print(f"Flight Mode saat ini: {master.flightmode}")

# Mengubah mode ke ALT_HOLD jika belum aktif
if master.flightmode != "ALT_HOLD":
    print("Mengubah flight mode ke ALT_HOLD...")
    mode_id = master.mode_mapping()["ALT_HOLD"]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    while True:
        master.recv_match(type='HEARTBEAT', blocking=True)
        if master.flightmode == "ALT_HOLD":
            print("Flight Mode: ALT_HOLD AKTIF")
            break

try:
    print("\nMengirim perintah ARM...")
    master.arducopter_arm()

    print("Menunggu konfirmasi status ARMED...")
    master.motors_armed_wait()

    print("\n==============================")
    print(">>> DRONE ARMED <<<")
    print("==============================")

    for remaining in range(ARM_TIME, 0, -1):
        print(f"ARMED - DISARM otomatis dalam {remaining:02d} detik", end="\r")
        time.sleep(1)

except KeyboardInterrupt:
    print("\nInterupsi darurat (Ctrl+C).")

finally:
    print("\n\nMengirim perintah DISARM...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()

    print("==============================")
    print(">>> DRONE DISARMED <<<")
    print("==============================")
```

**Eksekusi:**
```bash
python3 03_arm_disarm_althold.py
```

---

## 📈 Status Progres Proyek

- [x] Komunikasi Serial USB Raspberry Pi $\leftrightarrow$ Pixhawk PIX6
- [x] Verifikasi Heartbeat & Identitas Flight Controller (ArduCopter Quad)
- [x] Streaming Telemetri IMU Real-time (*Roll, Pitch, Yaw*)
- [x] Perubahan Flight Mode ke `ALT_HOLD` via Python
- [x] Arming & Disarming otomatis tanpa RC di mode `ALT_HOLD`
- [ ] **Sedang Berjalan:** Takeoff 1 Meter & Landing Vertikal Mandiri (`takeoff_land_althold.py`)
- [ ] Integrasi Modul Sensor Posisi Indoor (UWB DW3000 / Optical Flow)
- [ ] Algoritma Navigasi Otonom SAR & Swarm Coordination
