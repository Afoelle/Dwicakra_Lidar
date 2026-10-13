#!/usr/bin/env python3
"""
Slamtec RPLIDAR C1 Real-Time Radar Visualizer
============================================
Visualisasi data 360° LiDAR Slamtec RPLIDAR C1 berbasis OpenCV & PySerial.

Fitur:
- Komunikasi serial berkecepatan tinggi (Baudrate C1: 460800 bps)
- Pembacaan data asinkron via background thread (bebas lag/stutter)
- GUI Radar Interaktif dengan radar rings, crosshair sudut, dan pelacak halangan terdekat
- Deteksi port serial otomatis (/dev/ttyUSB*, /dev/ttyACM*, COM*)
- Mode Simulasi (--demo) untuk mencoba visualisasi tanpa sensor fisik
- Kontrol keyboard: Zoom (+/-), Pause (Spasi), Simpan Screenshot (S), Reset (R), Keluar (Q)
"""

import os
import sys
import time
import math
import glob
import random
import argparse
import threading
from datetime import datetime

import cv2
import numpy as np
import serial

# Default konfigurasi untuk RPLIDAR C1
DEFAULT_BAUD = 460800
DEFAULT_RANGE_M = 6.0  # Jarak maksimum tampilan radar (meter)
RADAR_SIZE = 800       # Ukuran window tampilan (800x800 pixel)


class RPLidarC1Driver:
    """Driver komunikasi serial mandiri untuk Slamtec RPLIDAR C1."""

    def __init__(self, port, baudrate=DEFAULT_BAUD, timeout=1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.is_scanning = False

    def connect(self):
        """Membuka koneksi serial ke LiDAR dan mengaktifkan daya motor."""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            # DTR = False dan RTS = False mengaktifkan jalur daya motor pada adapter CP2102
            self.ser.dtr = False
            self.ser.rts = False
            # Bersihkan buffer serial
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(0.1)
            return True
        except serial.SerialException as e:
            print(f"[ERROR] Gagal membuka port serial '{self.port}': {e}")
            if "Permission denied" in str(e):
                print("\n[TIPS IZIN AKSES LINUX]:")
                print("Jalankan perintah ini di terminal agar akun Anda punya izin membaca serial:")
                print("   sudo usermod -a -G dialout $USER")
                print("   sudo chmod 666 " + str(self.port))
                print("Lalu logout dan login kembali.\n")
            return False

    def stop_motor(self):
        """Mengirim perintah STOP ke LiDAR."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"\xa5\x25")  # STOP request
                time.sleep(0.05)
                self.ser.reset_input_buffer()
            except Exception:
                pass
        self.is_scanning = False

    def reset_device(self):
        """Mengirim perintah RESET ke LiDAR."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"\xa5\x40")  # RESET request
                time.sleep(0.5)
                self.ser.reset_input_buffer()
            except Exception:
                pass

    def start_scan(self):
        """Memulai pemindaian dan memutar motor (Scan Mode)."""
        if not self.ser or not self.ser.is_open:
            return False

        # Pastikan sinyal DTR aktif low untuk menyalakan motor
        self.ser.dtr = False
        self.ser.rts = False

        # Hentikan operasi sebelumnya dan bersihkan buffer
        self.stop_motor()
        self.ser.reset_input_buffer()
        time.sleep(0.05)

        # Kirim perintah SCAN (0xA5 0x20)
        self.ser.write(b"\xa5\x20")

        # Baca 7 byte respon descriptor: A5 5A 05 00 00 40 81
        descriptor = self.ser.read(7)
        if len(descriptor) < 7:
            print("[WARN] Respon descriptor tidak lengkap, mencoba membaca stream data langsung...")
        elif descriptor[0:2] != b"\xa5\x5a":
            print(f"[WARN] Descriptor sinkronisasi tidak cocok ({descriptor.hex()}), mencoba melanjutkan...")

        self.is_scanning = True
        return True

    def read_packet(self):
        """
        Membaca dan mem-parsing 1 paket data standar (5 byte):
        Byte 0: [Quality: 6 bit | ~S: 1 bit | S: 1 bit]
        Byte 1: [Angle_q6 low: 7 bit | Checkbit: 1 bit (harus 1)]
        Byte 2: [Angle_q6 high: 8 bit]
        Byte 3: [Distance_q2 low: 8 bit]
        Byte 4: [Distance_q2 high: 8 bit]
        """
        if not self.ser or not self.ser.is_open:
            return None

        # Baca byte awal untuk mencari header sinkronisasi
        b0 = self.ser.read(1)
        if not b0:
            return None
        byte0 = b0[0]

        # Validasi bit S (start flag) dan ~S (inversed start flag)
        s_bit = byte0 & 0x01
        s_inv_bit = (byte0 >> 1) & 0x01
        if s_bit == s_inv_bit:
            # Tidak sinkron, lewati byte ini
            return None

        # Baca 4 byte sisanya
        remaining = self.ser.read(4)
        if len(remaining) < 4:
            return None

        byte1, byte2, byte3, byte4 = remaining

        # Validasi checkbit (bit 0 pada byte1 harus bernilai 1)
        if (byte1 & 0x01) != 1:
            return None

        quality = byte0 >> 2
        angle_q6 = (byte1 >> 1) | (byte2 << 7)
        angle_deg = (angle_q6 / 64.0) % 360.0

        dist_q2 = byte3 | (byte4 << 8)
        dist_mm = dist_q2 / 4.0

        start_flag = bool(s_bit)
        return (start_flag, quality, angle_deg, dist_mm)

    def disconnect(self):
        """Menutup port serial dengan aman."""
        self.stop_motor()
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None


def auto_detect_serial_port():
    """Mendeteksi port serial yang terhubung (Linux / macOS / Windows)."""
    candidates = []
    if sys.platform.startswith("linux"):
        candidates.extend(sorted(glob.glob("/dev/ttyUSB*")))
        candidates.extend(sorted(glob.glob("/dev/ttyACM*")))
    elif sys.platform.startswith("darwin"):
        candidates.extend(sorted(glob.glob("/dev/tty.usbserial*")))
        candidates.extend(sorted(glob.glob("/dev/tty.usbmodem*")))
    elif sys.platform.startswith("win"):
        candidates.extend([f"COM{i}" for i in range(1, 32)])

    for port in candidates:
        try:
            s = serial.Serial(port)
            s.close()
            return port
        except Exception:
            # Port mungkin sedang digunakan atau butuh izin
            if os.path.exists(port):
                return port

    return "/dev/ttyUSB0" if not sys.platform.startswith("win") else "COM3"


class LidarScanBuffer:
    """Buffer thread-safe untuk menyimpan data 360 derajat terakhir."""

    def __init__(self):
        self.lock = threading.Lock()
        # Menyimpan data jarak (mm) dan kualitas untuk setiap sudut integer 0..359
        self.scan_points = {}  # {angle_deg_rounded: (distance_mm, quality, timestamp)}
        self.scan_count = 0
        self.point_rate = 0.0
        self._last_rate_time = time.time()
        self._point_counter = 0

    def update_point(self, angle_deg, dist_mm, quality):
        with self.lock:
            # Bulatkan ke resolusi 0.5 derajat agar rapat dan rapi
            key = round(angle_deg * 2) / 2.0
            now = time.time()
            if dist_mm > 0:
                self.scan_points[key] = (dist_mm, quality, now)
            elif key in self.scan_points:
                # Titik tidak terdeteksi
                pass

            self._point_counter += 1
            if now - self._last_rate_time >= 1.0:
                self.point_rate = self._point_counter / (now - self._last_rate_time)
                self._point_counter = 0
                self._last_rate_time = now

    def get_snapshot(self, max_age_seconds=0.3):
        """Mengambil data titik yang masih segar (tidak kedaluwarsa)."""
        now = time.time()
        with self.lock:
            fresh_points = []
            keys_to_remove = []
            for angle, (dist, qual, t_stamp) in self.scan_points.items():
                if now - t_stamp <= max_age_seconds:
                    fresh_points.append((angle, dist, qual))
                elif now - t_stamp > 1.5:
                    keys_to_remove.append(angle)

            for k in keys_to_remove:
                del self.scan_points[k]

            return fresh_points, self.point_rate


def lidar_worker(driver, buffer_obj, stop_event):
    """Background worker untuk membaca data serial secara kontinu."""
    print("[INFO] Thread pembaca data LiDAR dimulai...")
    while not stop_event.is_set():
        if not driver.is_scanning:
            time.sleep(0.05)
            continue

        try:
            packet = driver.read_packet()
            if packet is not None:
                start_flag, quality, angle_deg, dist_mm = packet
                buffer_obj.update_point(angle_deg, dist_mm, quality)
        except Exception as e:
            if not stop_event.is_set():
                time.sleep(0.01)


def demo_worker(buffer_obj, stop_event):
    """Generator data virtual simulasi ruangan jika LiDAR fisik tidak terhubung."""
    print("[INFO] Mode Simulasi Aktif: Membangun ruangan virtual...")
    angle = 0.0
    t0 = time.time()
    while not stop_event.is_set():
        elapsed = time.time() - t0
        # Simulasikan ruangan persegi 4x4 meter dengan 1 rintangan bergerak
        rad = math.radians(angle)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)

        # Jarak ke dinding kotak
        d_x = 2000 / (abs(cos_a) + 1e-5)
        d_y = 2000 / (abs(sin_a) + 1e-5)
        dist = min(d_x, d_y)

        # Objek silinder bergerak
        obj_x = 1000 * math.sin(elapsed * 1.5)
        obj_y = 800 * math.cos(elapsed * 1.0)
        obj_dist = math.hypot(obj_x, obj_y)
        obj_angle = (math.degrees(math.atan2(obj_y, obj_x)) + 360) % 360

        if abs(angle - obj_angle) < 6.0:
            dist = min(dist, obj_dist)

        # Beri sedikit noise acak
        dist += random.uniform(-15, 15)

        buffer_obj.update_point(angle, dist, quality=45)
        angle = (angle + 0.5) % 360.0
        time.sleep(0.0002)


class RadarVisualizer:
    """Renderer GUI Radar interaktif dengan OpenCV."""

    def __init__(self, size=RADAR_SIZE, max_range_m=DEFAULT_RANGE_M):
        self.size = size
        self.center = (size // 2, size // 2)
        self.radius = (size // 2) - 40
        self.max_range_m = max_range_m
        self.is_paused = False
        self.last_points = []
        self.window_name = "Slamtec RPLIDAR C1 - Radar Visualizer"

    def adjust_range(self, delta_m):
        """Mengatur batas jarak zoom (1m s/d 16m)."""
        self.max_range_m = max(1.0, min(16.0, self.max_range_m + delta_m))

    def render(self, points, point_rate, fps, port_name, is_demo=False):
        """Menggambar frame radar lengkap."""
        if not self.is_paused:
            self.last_points = points
        else:
            points = self.last_points

        # Kanvas latar belakang hitam elegan
        frame = np.zeros((self.size, self.size, 3), dtype=np.uint8)
        cx, cy = self.center

        # 1. Gambar grid lingkaran (Distance Rings)
        ring_step = 1.0 if self.max_range_m <= 6.0 else 2.0
        r_current = ring_step
        while r_current <= self.max_range_m:
            pix_r = int((r_current / self.max_range_m) * self.radius)
            if pix_r <= self.radius:
                # Lingkaran tipis hijau tua
                cv2.circle(frame, (cx, cy), pix_r, (30, 60, 30), 1, cv2.LINE_AA)
                # Label jarak
                label = f"{r_current:.0f}m"
                cv2.putText(frame, label, (cx + 5, cy - pix_r + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (80, 160, 80), 1, cv2.LINE_AA)
            r_current += ring_step

        # Lingkaran terluar radar
        cv2.circle(frame, (cx, cy), self.radius, (0, 140, 0), 2, cv2.LINE_AA)

        # 2. Gambar garis sudut (0°, 90°, 180°, 270°) dan diagonal (45°, 135°, ...)
        for deg in range(0, 360, 45):
            rad = math.radians(deg - 90)  # 0° di atas (Depan / Front)
            x_end = int(cx + self.radius * math.cos(rad))
            y_end = int(cy + self.radius * math.sin(rad))
            color = (0, 90, 0) if deg % 90 == 0 else (20, 50, 20)
            thickness = 2 if deg % 90 == 0 else 1
            cv2.line(frame, (cx, cy), (x_end, y_end), color, thickness, cv2.LINE_AA)

            # Label sudut di lingkaran luar
            label_x = int(cx + (self.radius + 18) * math.cos(rad))
            label_y = int(cy + (self.radius + 18) * math.sin(rad))
            text = f"{deg}°"
            if deg == 0:
                text = "0° (F)"
            elif deg == 90:
                text = "90° (R)"
            elif deg == 180:
                text = "180° (B)"
            elif deg == 270:
                text = "270° (L)"

            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)
            cv2.putText(frame, text, (label_x - tw // 2, label_y + th // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 200, 100), 1, cv2.LINE_AA)

        # 3. Gambar titik-titik LiDAR (Point Cloud)
        min_dist_mm = float("inf")
        closest_point = None

        for angle_deg, dist_mm, quality in points:
            if dist_mm <= 0:
                continue

            dist_m = dist_mm / 1000.0
            if dist_m > self.max_range_m:
                continue

            # Cari titik halangan terdekat
            if dist_mm < min_dist_mm and dist_mm > 50:  # Abaikan blind-zone < 5cm
                min_dist_mm = dist_mm
                closest_point = (angle_deg, dist_mm)

            # Konversi sudut ke koordinat layar (0° di sumbu Y negatif / ke atas)
            rad = math.radians(angle_deg - 90)
            pix_dist = (dist_m / self.max_range_m) * self.radius
            px = int(cx + pix_dist * math.cos(rad))
            py = int(cy + pix_dist * math.sin(rad))

            # Pewarnaan berdasarkan jarak (Dekat = Merah/Kuning, Jauh = Cyan/Biru)
            ratio = min(1.0, dist_m / self.max_range_m)
            # BGR: Merah terang (dekat) ke Biru/Cyan terang (jauh)
            b = int(255 * ratio)
            g = int(255 * (1.0 - abs(ratio - 0.5) * 2))
            r = int(255 * (1.0 - ratio))

            cv2.circle(frame, (px, py), 2, (b, g, r), -1, cv2.LINE_AA)

        # 4. Highlight titik terdekat jika ada
        if closest_point is not None:
            c_ang, c_dist = closest_point
            rad = math.radians(c_ang - 90)
            pix_dist = ((c_dist / 1000.0) / self.max_range_m) * self.radius
            c_px = int(cx + pix_dist * math.cos(rad))
            c_py = int(cy + pix_dist * math.sin(rad))

            # Cincin target merah berkedip
            cv2.circle(frame, (c_px, c_py), 8, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.line(frame, (cx, cy), (c_px, c_py), (0, 0, 255), 1, cv2.LINE_AA)

        # 5. Gambar simbol Drone / Sensor di tengah
        cv2.circle(frame, (cx, cy), 6, (0, 255, 255), -1, cv2.LINE_AA)
        # Panah penunjuk arah depan (0°)
        cv2.arrowedLine(frame, (cx, cy), (cx, cy - 25), (0, 255, 255), 2, tipLength=0.4)

        # 6. Informasi HUD (Heads-Up Display) di Pojok Kiri Atas
        hud_color = (200, 255, 200)
        mode_str = "SIMULASI (DEMO)" if is_demo else "ONLINE"
        status_color = (0, 200, 255) if is_demo else (0, 255, 0)
        if self.is_paused:
            mode_str += " [PAUSED]"
            status_color = (0, 140, 255)

        cv2.putText(frame, "SLAMTEC RPLIDAR C1", (20, 30),
                    cv2.FONT_HERSHEY_DUPLEX, 0.65, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f"Status: {mode_str}", (20, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, status_color, 1, cv2.LINE_AA)
        cv2.putText(frame, f"Port: {port_name} | {DEFAULT_BAUD} bps", (20, 72),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, hud_color, 1, cv2.LINE_AA)
        cv2.putText(frame, f"Jangkauan Tampilan: {self.max_range_m:.1f} m", (20, 92),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, hud_color, 1, cv2.LINE_AA)
        cv2.putText(frame, f"Titik Aktif: {len(points)}  |  Laju: {point_rate:.0f} pts/s", (20, 112),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, hud_color, 1, cv2.LINE_AA)
        cv2.putText(frame, f"Tampilan FPS: {fps:.1f}", (20, 132),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, hud_color, 1, cv2.LINE_AA)

        # Tampilkan Halangan Terdekat di Pojok Kanan Atas
        if closest_point is not None:
            c_ang, c_dist = closest_point
            warn_box_color = (0, 0, 255) if c_dist < 600 else (0, 200, 255)
            cv2.putText(frame, "OBSTACLE TERDEKAT:", (self.size - 260, 30),
                        cv2.FONT_HERSHEY_DUPLEX, 0.5, warn_box_color, 1, cv2.LINE_AA)
            cv2.putText(frame, f"Jarak: {c_dist/10.0:.1f} cm ({c_dist/1000.0:.2f} m)",
                        (self.size - 260, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, f"Sudut: {c_ang:.1f} deg",
                        (self.size - 260, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        # 7. Menu Petunjuk Tombol di Pojok Bawah
        menu_text = "[+] Zoom In  [-] Zoom Out  [Spasi] Pause  [S] Simpan Gambar  [Q] Keluar"
        cv2.putText(frame, menu_text, (20, self.size - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (140, 140, 140), 1, cv2.LINE_AA)

        return frame


def main():
    parser = argparse.ArgumentParser(description="Visualisasi Real-Time Slamtec RPLIDAR C1")
    parser.add_argument("--port", type=str, default=None,
                        help="Port serial (contoh: /dev/ttyUSB0 atau COM3). Jika kosong, dideteksi otomatis.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default C1: {DEFAULT_BAUD})")
    parser.add_argument("--range", type=float, default=DEFAULT_RANGE_M,
                        help=f"Jangkauan maksimum tampilan awal dalam meter (default: {DEFAULT_RANGE_M})")
    parser.add_argument("--demo", action="store_true",
                        help="Jalankan dalam mode simulasi virtual tanpa memerlukan perangkat keras fisik.")
    args = parser.parse_args()

    stop_event = threading.Event()
    buffer_obj = LidarScanBuffer()
    driver = None
    is_demo = args.demo
    actual_port = args.port

    if not is_demo:
        if actual_port is None:
            actual_port = auto_detect_serial_port()
            print(f"[INFO] Mendeteksi port serial otomatis: {actual_port}")

        driver = RPLidarC1Driver(actual_port, baudrate=args.baud)
        print(f"[INFO] Menghubungkan ke RPLIDAR C1 di {actual_port} ({args.baud} bps)...")

        if not driver.connect():
            print("\n[PERINGATAN] Gagal terhubung ke RPLIDAR C1.")
            print("Apakah Anda ingin beralih ke Mode Simulasi (DEMO)? (y/n): ", end="", flush=True)
            try:
                # Tunggu input singkat atau beralih otomatis jika non-interaktif
                ans = input().strip().lower()
            except Exception:
                ans = "y"

            if ans in ("y", "yes", ""):
                is_demo = True
                actual_port = "SIMULASI"
            else:
                print("[INFO] Program dibatalkan.")
                sys.exit(1)
        else:
            print("[INFO] Koneksi serial berhasil! Memulai laser scanner...")
            if not driver.start_scan():
                print("[ERROR] Gagal memulai perintah scan pada RPLIDAR C1.")
                driver.disconnect()
                sys.exit(1)

    # Jalankan background thread
    if is_demo:
        thread = threading.Thread(target=demo_worker, args=(buffer_obj, stop_event), daemon=True)
    else:
        thread = threading.Thread(target=lidar_worker, args=(driver, buffer_obj, stop_event), daemon=True)
    thread.start()

    # Inisialisasi Visualizer
    viz = RadarVisualizer(size=RADAR_SIZE, max_range_m=args.range)
    print("\n" + "=" * 50)
    print("  VISUALISASI SLAMTEC RPLIDAR C1 TELAH BERJALAN")
    print("=" * 50)
    print("Kontrol Keyboard:")
    print("  + / =    : Zoom In (persempit jarak pandang)")
    print("  - / _    : Zoom Out (perluas jarak pandang s/d 16m)")
    print("  SPASI    : Pause / Lanjutkan pemindaian")
    print("  S        : Ambil screenshot (simpan gambar radar)")
    print("  Q / ESC  : Keluar dari program")
    print("=" * 50 + "\n")

    last_time = time.time()
    fps = 0.0

    try:
        while True:
            t_now = time.time()
            dt = t_now - last_time
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)
            last_time = t_now

            # Dapatkan titik-titik pindaian terbaru
            fresh_pts, pt_rate = buffer_obj.get_snapshot(max_age_seconds=0.35)

            # Render kanvas
            frame = viz.render(
                points=fresh_pts,
                point_rate=pt_rate,
                fps=fps,
                port_name=str(actual_port),
                is_demo=is_demo
            )

            cv2.imshow(viz.window_name, frame)

            # Baca tombol keyboard
            key = cv2.waitKey(16) & 0xFF  # ~60 FPS
            if key in (ord('q'), ord('Q'), 27):  # 27 = ESC
                print("[INFO] Menghentikan program...")
                break
            elif key in (ord('+'), ord('=')):
                viz.adjust_range(-1.0)
            elif key in (ord('-'), ord('_')):
                viz.adjust_range(1.0)
            elif key == ord(' '):
                viz.is_paused = not viz.is_paused
                status = "DIHENTIKAN SEMENTARA (PAUSED)" if viz.is_paused else "DILANJUTKAN"
                print(f"[INFO] Pemindaian {status}")
            elif key in (ord('s'), ord('S')):
                filename = f"lidar_scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                cv2.imwrite(filename, frame)
                print(f"[INFO] Screenshot disimpan sebagai: {filename}")

    except KeyboardInterrupt:
        print("\n[INFO] Dihentikan oleh pengguna (Ctrl+C).")

    finally:
        # Hentikan thread & matikan motor LiDAR dengan aman
        stop_event.set()
        if driver is not None:
            print("[INFO] Mengirim perintah STOP & RESET ke RPLIDAR C1...")
            driver.disconnect()
        cv2.destroyAllWindows()
        print("[INFO] Program selesai dengan aman.")


if __name__ == "__main__":
    main()

