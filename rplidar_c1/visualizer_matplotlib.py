#!/usr/bin/env python3
"""
Slamtec RPLIDAR C1 - Matplotlib Polar Plot Visualizer
===================================================
Visualisasi polar 360° menggunakan Matplotlib (cocok untuk analisa ilmiah/laporan).
"""

import sys
import time
import argparse
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Import driver dari modul visualizer utama
from visualizer import RPLidarC1Driver, LidarScanBuffer, lidar_worker, demo_worker, auto_detect_serial_port, DEFAULT_BAUD


def main():
    parser = argparse.ArgumentParser(description="Matplotlib Polar Plot untuk RPLIDAR C1")
    parser.add_argument("--port", type=str, default=None, help="Port serial (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument("--range", type=float, default=6.0, help="Jarak max dalam meter (default: 6.0)")
    parser.add_argument("--demo", action="store_true", help="Jalankan mode simulasi tanpa sensor fisik")
    args = parser.parse_args()

    stop_event = threading.Event()
    buffer_obj = LidarScanBuffer()
    driver = None
    is_demo = args.demo
    actual_port = args.port

    if not is_demo:
        if actual_port is None:
            actual_port = auto_detect_serial_port()
        driver = RPLidarC1Driver(actual_port, baudrate=args.baud)
        print(f"[INFO] Menghubungkan ke RPLIDAR C1 di {actual_port} ({args.baud} bps)...")
        if not driver.connect() or not driver.start_scan():
            print("[WARN] Gagal terhubung ke perangkat. Beralih ke mode --demo...")
            is_demo = True
            actual_port = "SIMULASI"

    if is_demo:
        thread = threading.Thread(target=demo_worker, args=(buffer_obj, stop_event), daemon=True)
    else:
        thread = threading.Thread(target=lidar_worker, args=(driver, buffer_obj, stop_event), daemon=True)
    thread.start()

    # Siapkan figure Matplotlib dalam koordinat polar
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='polar')
    ax.set_theta_zero_location('N')   # 0 derajat di atas (Depan / Front)
    ax.set_theta_direction(-1)        # Searah jarum jam (Clockwise)
    ax.set_rmax(args.range)
    ax.grid(True, color='#005500', linestyle='--', linewidth=0.7)

    scatter = ax.scatter([], [], s=12, c=[], cmap='plasma', alpha=0.9)
    title = ax.set_title(f"Slamtec RPLIDAR C1 - Polar Scan ({actual_port})", va='bottom', color='#00FFCC')

    def update(_):
        pts, rate = buffer_obj.get_snapshot(max_age_seconds=0.4)
        if not pts:
            return scatter,

        angles = [np.radians(p[0]) for p in pts]
        dists = [p[1] / 1000.0 for p in pts]  # Konversi mm ke meter

        offsets = np.column_stack((angles, dists))
        scatter.set_offsets(offsets)
        scatter.set_array(np.array(dists))
        title.set_text(f"Slamtec RPLIDAR C1 ({actual_port}) | {len(pts)} Titik | Laju: {rate:.0f} pts/s")
        return scatter,

    ani = FuncAnimation(fig, update, interval=40, blit=False)

    try:
        print("[INFO] Menampilkan grafik polar... Tekan Ctrl+C atau tutup jendela untuk keluar.")
        plt.show()
    finally:
        stop_event.set()
        if driver:
            driver.disconnect()
        print("[INFO] Selesai.")


if __name__ == "__main__":
    main()

