import serial
import threading
import time
import sys
import math
import csv

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ---------- USER SETTINGS ----------
SERIAL_PORT = "COM8"      # <-- change to your COM port (e.g. COM3 on Windows or /dev/ttyUSB0 on Linux)
BAUDRATE = 115200
LOG_CSV = True            # save received data to CSV
CSV_FILENAME = "pdr_log.csv"

SMOOTHING_ALPHA = 0.4     # 0..1 for exponential smoothing (1=no smoothing, lower = smoother)
# -----------------------------------

# thread-safe storage of latest value
import threading
latest_lock = threading.Lock()
latest = {"x": None, "y": None, "h": None, "t": None}

# path list (keeps everything, no vanishing)
full_path = []

# CSV header writer
csv_file = None
csv_writer = None
if LOG_CSV:
    csv_file = open(CSV_FILENAME, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["timestamp_ms", "x_m", "y_m", "heading_deg", "raw_line"])


def parse_pos_line(line):
    line = line.strip()
    if not line:
        return None
    if not line.startswith("POS,"):
        return None
    try:
        parts = line.split(",")
        if len(parts) < 4:
            return None
        x = float(parts[1])
        y = float(parts[2])
        h = float(parts[3])
        return x, y, h
    except Exception:
        return None


def serial_reader_thread(port, baud):
    global latest, csv_writer
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"[serial] Opened {port} @ {baud}")
    except Exception as e:
        print(f"[serial] Failed to open {port}: {e}")
        sys.exit(1)

    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            parsed = parse_pos_line(line)
            ts = int(time.time() * 1000)
            if LOG_CSV and csv_writer:
                csv_writer.writerow([ts, *(([""] * 3) if parsed is None else parsed), line])
            if parsed:
                x, y, h = parsed
                with latest_lock:
                    latest["x"] = x
                    latest["y"] = y
                    latest["h"] = h
                    latest["t"] = ts
        except Exception as e:
            print(f"[serial] Read error: {e}")
            time.sleep(0.5)


def setup_plot():
    fig, ax = plt.subplots()
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Live PDR Track")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    trail_line, = ax.plot([], [], linewidth=1, color="blue")   # full path
    current_dot, = ax.plot([], [], marker='o', color="red")    # current position

    # text for debug
    info_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")

    return fig, ax, trail_line, current_dot, info_text


def update_plot(frame, ax, trail_line, current_dot, info_text, smoothing_state):
    global latest, full_path

    # read latest safely
    with latest_lock:
        lx, ly, lh, lt = latest["x"], latest["y"], latest["h"], latest["t"]

    if lx is None:
        info_text.set_text("Waiting for data...")
        return trail_line, current_dot, info_text

    # smoothing: exponential moving average
    if smoothing_state["sx"] is None:
        smoothing_state["sx"] = lx
        smoothing_state["sy"] = ly
        smoothing_state["sh"] = lh
    else:
        a = SMOOTHING_ALPHA
        smoothing_state["sx"] = a * lx + (1 - a) * smoothing_state["sx"]
        smoothing_state["sy"] = a * ly + (1 - a) * smoothing_state["sy"]
        # heading smoothing (wrap-around)
        prev_h = smoothing_state["sh"]
        v_prev = complex(math.cos(math.radians(prev_h)), math.sin(math.radians(prev_h)))
        v_new = complex(math.cos(math.radians(lh)), math.sin(math.radians(lh)))
        v_avg = a * v_new + (1 - a) * v_prev
        avg_angle = math.degrees(math.atan2(v_avg.imag, v_avg.real)) % 360
        smoothing_state["sh"] = avg_angle

    sx = smoothing_state["sx"]
    sy = smoothing_state["sy"]
    sh = smoothing_state["sh"]

    # append to full path (keeps growing, no vanish)
    full_path.append((sx, sy))

    # update axis limits to follow path
    xs = [p[0] for p in full_path]
    ys = [p[1] for p in full_path]
    if xs and ys:
        margin = 1.0
        xmin, xmax = min(xs) - margin, max(xs) + margin
        ymin, ymax = min(ys) - margin, max(ys) + margin
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)

    # update path and current dot
    trail_line.set_data(xs, ys)
    current_dot.set_data([sx], [sy])

    # draw heading arrow
    arrow_len = 0.6
    theta = math.radians((90.0 - sh))
    dx = arrow_len * math.cos(theta)
    dy = arrow_len * math.sin(theta)

    if not hasattr(update_plot, "heading_lines"):
        update_plot.heading_lines = []
    ln, = ax.plot([sx, sx + dx], [sy, sy + dy], linewidth=2, color="red")
    update_plot.heading_lines.append(ln)
    if len(update_plot.heading_lines) > 4:
        old = update_plot.heading_lines.pop(0)
        try:
            old.remove()
        except Exception:
            pass

    # info text update
    info_text.set_text(f"X={sx:.3f} m  Y={sy:.3f} m  H={sh:.1f}°\nraw_ts={lt} ms")

    return trail_line, current_dot, info_text


def main():
    # start serial reader
    t = threading.Thread(target=serial_reader_thread, args=(SERIAL_PORT, BAUDRATE), daemon=True)
    t.start()

    # setup plot
    fig, ax, trail_line, current_dot, info_text = setup_plot()

    # smoothing_state holds EMA values
    smoothing_state = {"sx": None, "sy": None, "sh": None}

    ani = FuncAnimation(fig, update_plot,
                        fargs=(ax, trail_line, current_dot, info_text, smoothing_state),
                        interval=100, blit=False)

    plt.show()

    # cleanup
    if LOG_CSV and csv_file:
        csv_file.close()


if __name__ == "__main__":
    main()
