#!/usr/bin/env python3

import serial, struct, math, time
import matplotlib.pyplot as plt
print(plt.get_backend())
import numpy as np

# Serial Communication Constants
PORT         = "/dev/ttyUSB0"    
BAUD         = 230_400
FRAME_LEN    = 47
HEADER       = 0x54
VER_LEN      = 0x2C
PTS_PER_PKT  = 12

def build_crc8_table(poly=0x1D):
    tbl = []
    for byte in range(256):
        crc = byte
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
        tbl.append(crc)
    return tbl

CRC_TABLE = build_crc8_table()

def calc_crc8(data: bytes) -> int:
    """
    Compute the CRC‑8 checksum for a bytes‑like object using the same
    lookup‑table algorithm as the original C code.

    Parameters
    ----------
    data : bytes | bytearray | memoryview
        Payload to checksum.

    Returns
    -------
    int
        Computed CRC‑8 value (0‑255).
    """
    crc = 0
    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc

def crc_check(data: bytes) -> bool:
    """
    Verifies the CRC8 of a 47-byte LiDAR frame.
    """
    if len(data) != 47:
        raise ValueError("Expected exactly 47 bytes for a LiDAR frame")

    computed_crc = calc_crc8(data[:46])  # Exclude the last byte
    received_crc = data[46]              # Last byte is the CRC8 from the sensor

    return computed_crc == received_crc

def parse_data(frame_bytes):
    if len(frame_bytes) != FRAME_LEN:
        raise ValueError("Invalid frame size")

    header, ver_len, speed, start_angle = struct.unpack_from("<BBHH", frame_bytes, 0)

    # Read the 12 raw (distance, strength) points
    points = []
    offset = 6
    for _ in range(PTS_PER_PKT):
        dist, strength = struct.unpack_from("<HB", frame_bytes, offset)
        points.append((dist, strength))
        offset += 3

    end_angle, timestamp = struct.unpack_from("<HH", frame_bytes, offset)

    # Compute angular span with wrap‑around
    delta = (end_angle - start_angle + 36000) % 36000
    step  = delta / (PTS_PER_PKT - 1)

    # Append per‑point angle to each tuple
    for i in range(len(points)):
        angle = start_angle + step * i
        dist, strength = points[i]
        points[i] = (dist, angle, strength)

    return {
        "header":      header,
        "ver_len":     ver_len,
        "speed":       speed,
        "start_angle": start_angle,
        "end_angle":   end_angle,
        "timestamp":   timestamp,
        "points":      points,  # now (distance, strength, angle)
    }

def plotdata(distances, thetas, intensities, axes):
    for a, d, i in zip(thetas, distances, intensities):
        rad = math.radians(a)
        #pts_x.append(d * math.cos(rad))
        #pts_y.append(d * math.sin(rad))
        c = axes.scatter(a, d, s=i, alpha=0.75)
    return c

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Serial open on {PORT} @ {BAUD} baud")

    plt.ion()  # Turn on interactive mode
    fig = plt.figure()
    ax = fig.add_subplot(projection='polar')
    plt.show()

    try:
        while True:
            byte = ser.read(1)
            if not byte or byte[0] != HEADER:
                continue

            frame = bytes([HEADER]) + ser.read(FRAME_LEN - 1)
            if len(frame) != FRAME_LEN or not crc_check(frame):
                continue

            parsed = parse_data(frame)
            angles = [p[1]/100.0 for p in parsed["points"]]        # degrees
            dists  = [p[0]/1000.0 for p in parsed["points"]]       # meters
            strengths = [p[2] for p in parsed["points"]]

            ax.clear()
            ax.set_theta_zero_location("N")
            ax.set_theta_direction(-1)
            ax.set_rlim(0, max(dists) + 1)
            plotdata(dists, angles, strengths, ax)

            plt.draw()
            plt.pause(0.001)  # <- THIS is what makes the plot window responsive

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
