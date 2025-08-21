# -*- coding: utf-8 -*-
"""
Read ArduPilot DataFlash .BIN via pymavlink (DFReader), export GPS/ATT/IMU to CSV,
and plot a 3D trajectory using Plotly.

- Opens a file dialog to choose the .BIN file (tkinter).
- Works with DFReader (no timeout/blocking args on recv_match).
- Uses TimeUS (µs) if available; falls back to msg._timestamp (s).
- Normalizes GPS Lat/Lng/Alt units with simple heuristics.
- Saves CSVs next to the .BIN file.
"""

import os
import tkinter as tk
from tkinter import filedialog

from pymavlink import mavutil
import pandas as pd
import plotly.graph_objects as go
import plotly.io as pio


def choose_bin_file() -> str:
    """Open a file dialog for selecting a DataFlash .BIN log."""
    root = tk.Tk()
    root.withdraw()  # hide the root window
    filepath = filedialog.askopenfilename(
        title="Select an ArduPilot DataFlash .BIN log",
        filetypes=[("BIN files", "*.BIN;*.bin"), ("All files", "*.*")]
    )
    return filepath


def normalize_lat_lon_alt(gps_df: pd.DataFrame) -> pd.DataFrame:
    """
    Normalize GPS columns to:
      - Lat/Lng in degrees
      - Alt in meters

    Heuristics:
      - If |Lat| > 90 or |Lng| > 180, assume degrees*1e7 → divide by 1e7
      - For Alt, if median is very large, assume cm → divide by 100
        else if moderately large (>500), assume 0.1 m → divide by 10
    """
    df = gps_df.copy()

    # Latitude / Longitude
    if "Lat" in df.columns and df["Lat"].abs().max() > 90:
        df["Lat"] = df["Lat"] / 1e7
    if "Lng" in df.columns and df["Lng"].abs().max() > 180:
        df["Lng"] = df["Lng"] / 1e7

    # Altitude
    if "Alt" in df.columns:
        alt_series = df["Alt"].dropna()
        if len(alt_series):
            med = alt_series.median()
            if med > 2000:          # e.g., ~10000 cm → ~100 m
                df["Alt"] = df["Alt"] / 100.0
            elif med > 500:         # e.g., decimeters (0.1 m)
                df["Alt"] = df["Alt"] / 10.0
            # else: treat as meters
    return df


def main():
    # 1) Select .BIN
    bin_file = choose_bin_file()
    if not bin_file:
        print("No file selected. Exit.")
        return
    print(f"[INFO] Selected file: {bin_file}")

    out_dir = os.path.dirname(bin_file)
    base = os.path.splitext(os.path.basename(bin_file))[0]

    # 2) Open DataFlash log (DFReader)
    print("[INFO] Opening DataFlash log...")
    mlog = mavutil.mavlink_connection(bin_file)

    # 3) Read messages (no timeout arg for DFReader)
    gps_rows, att_rows, imu_rows = [], [], []
    wanted_types = ["GPS", "GPS2", "ATT", "IMU", "IMU2"]

    print("[INFO] Reading messages (GPS/ATT/IMU)...")
    while True:
        msg = mlog.recv_match(type=wanted_types)  # DFReader: returns next or None at EOF
        if msg is None:
            break

        d = msg.to_dict()

        # Unified timestamp in seconds
        if "TimeUS" in d and d["TimeUS"] is not None:
            try:
                d["time_s"] = float(d["TimeUS"]) / 1e6
            except Exception:
                d["time_s"] = float(getattr(msg, "_timestamp", 0.0) or 0.0)
        else:
            d["time_s"] = float(getattr(msg, "_timestamp", 0.0) or 0.0)

        mtype = msg.get_type()
        if mtype.startswith("GPS"):        # GPS or GPS2
            gps_rows.append(d)
        elif mtype == "ATT":
            att_rows.append(d)
        elif mtype.startswith("IMU"):      # IMU or IMU2
            imu_rows.append(d)

    # 4) DataFrames + CSV export
    gps_df = pd.DataFrame(gps_rows)
    att_df = pd.DataFrame(att_rows)
    imu_df = pd.DataFrame(imu_rows)

    gps_csv = os.path.join(out_dir, f"{base}_gps.csv")
    att_csv = os.path.join(out_dir, f"{base}_att.csv")
    imu_csv = os.path.join(out_dir, f"{base}_imu.csv")

    gps_df.to_csv(gps_csv, index=False)
    att_df.to_csv(att_csv, index=False)
    imu_df.to_csv(imu_csv, index=False)

    print("[INFO] Saved CSVs:")
    print(f"       GPS: {gps_csv} ({len(gps_df)} rows)")
    print(f"       ATT: {att_csv} ({len(att_df)} rows)")
    print(f"       IMU: {imu_csv} ({len(imu_df)} rows)")

    # 5) Plot 3D trajectory if GPS present
    if gps_df.empty or not {"Lat", "Lng", "Alt"}.issubset(gps_df.columns):
        print("[WARN] GPS data missing or columns not found (Lat/Lng/Alt). Skip plotting.")
        return

    gps_norm = normalize_lat_lon_alt(gps_df)
    lat = gps_norm["Lat"]
    lng = gps_norm["Lng"]
    alt = gps_norm["Alt"]

    # 6) Plot with Plotly
    print("[INFO] Plotting 3D trajectory...")
    pio.renderers.default = "browser"  # open in default web browser

    fig = go.Figure(data=[
        go.Scatter3d(
            x=lng, y=lat, z=alt,
            mode="lines+markers",
            marker=dict(size=3),
            name="GPS Track"
        )
    ])
    fig.update_layout(
        scene=dict(
            xaxis_title="Longitude (deg)",
            yaxis_title="Latitude (deg)",
            zaxis_title="Altitude (m)"
        ),
        title="3D Trajectory from DataFlash GPS"
    )
    fig.show()
    print("[INFO] Done.")


if __name__ == "__main__":
    main()
