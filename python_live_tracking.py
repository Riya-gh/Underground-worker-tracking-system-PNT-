# live_plot_pdr.py
import serial
import time
import math
import matplotlib.pyplot as plt

SERIAL_PORT = "COM8"   # << change this to your port
BAUDRATE = 115200
TIMEOUT = 1.0

ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
time.sleep(1.0)  # allow the serial port to warm up

xs = []
ys = []
yaws = []

plt.ion()
fig, ax = plt.subplots(figsize=(6,6))
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("Live PDR path")
line, = ax.plot([], [], '-o')

while True:
    try:
        line_raw = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line_raw:
            continue
        # Expect: POS,x,y,yaw
        if line_raw.startswith("POS"):
            parts = line_raw.split(',')
            if len(parts) >= 4:
                try:
                    px = float(parts[1])
                    py = float(parts[2])
                    yaw = float(parts[3])
                except:
                    continue
                xs.append(px)
                ys.append(py)
                yaws.append(yaw)
                # plot
                line.set_xdata(xs)
                line.set_ydata(ys)
                ax.relim()
                ax.autoscale_view()
                # a small marker for heading at last point
                ax.plot(xs[-1], ys[-1], 'ro')
                plt.pause(0.01)
        else:
            # debug prints (optional)
            print("DEBUG:", line_raw)
    except KeyboardInterrupt:
        print("Stopped by user")
        break
    except Exception as e:
        print("Err:", e)
        time.sleep(0.1)

ser.close()
