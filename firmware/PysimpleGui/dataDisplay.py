import FreeSimpleGUI as sg
from usb_reader import USBReader
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
import queue

# ---- USBReader ----
# reader = USBReader(port='COM5') # uncomment when having gps and imu connected

# ---- DummyReader to simulate USBReader ---- (Erease dummyt usb reader when having real usb connected)
class DummyReader:
    def __init__(self):
        self.data_queue = queue.Queue()

reader = DummyReader()

# ---- Function to feed fake data ----
def feed_dummy_data():
    lat, lon, alt = 36.12, -96.04, 250.0
    ax, ay, az = 0.01, 0.02, 0.98
    sats = 5
    while True:
        dummy = {
            "LAT": lat,
            "LON": lon,
            "ALT": alt,
            "AX": ax,
            "AY": ay,
            "AZ": az,
            "SATS": sats
        }
        reader.data_queue.put(dummy)
        # simulate data change
        lat += 0.0001
        lon += 0.0001
        alt += 0.1
        ax += 0.01
        ay += 0.01
        az -= 0.001
        sats = (sats % 12) + 1
        time.sleep(0.5)

# Start the dummy data thread
threading.Thread(target=feed_dummy_data, daemon=True).start()

# ---- Matplotlib Figure ----
fig, ax = plt.subplots()
line, = ax.plot([], [], 'ro-')
ax.set_xlabel("Longitude")
ax.set_ylabel("Latitude")
ax.set_title("GPS Track")
lat_data = []
lon_data = []

# ---- Helper to draw the plot in FSG ----
def draw_figure(canvas, figure):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg

# ---- Home screen layout ----
home = [
    [sg.Button("Start Display", button_color="green"), sg.Button("End Session", button_color="red")]
]

# ---- Data display layout ----
dataDisplay = [
    [sg.Button("End Display", button_color="red"), sg.Button("Clear Plot", button_color="orange")],
    [sg.Text("GPS Data", justification="center", font=('Arial Bold', 20))],
    [sg.Text("Latitude: --", key='lat')],
    [sg.Text("Longitude: --", key='lon')],
    [sg.Text("Altitude: -- m", key='alt')],
    [sg.Text("Satellites: 0", key='sats')],
    [sg.Text("IMU Data", justification="center", font=('Arial Bold', 20))],
    [sg.Text("Ax: -- m/s²", key='ax')],
    [sg.Text("Ay: -- m/s²", key='ay')],
    [sg.Text("Az: -- m/s²", key='az')],
    [sg.Canvas(size=(400, 300), key='-CANVAS-')]
]

# ---- Layout arrangement ----
layout = [
    [sg.Column(home, key='-COL1-'), sg.Column(dataDisplay, visible=False, key='-COL2-')]
]

window = sg.Window("Localization Device Dashboard", layout, finalize=True, margins=(50,50))

# Draw Matplotlib figure on the Canvas
canvas_elem = window['-CANVAS-']
canvas = canvas_elem.Widget
fig_agg = draw_figure(canvas, fig)

layout_visible = 1
satellites = 0

# ---- Main event loop ----
while True:
    event, values = window.read(timeout=100)  # 0.1s polling

    if event in (None, 'End Session'):
        break

    if event == 'Start Display':
        window[f'-COL{layout_visible}-'].update(visible=False)
        layout_visible = 2
        window[f'-COL{layout_visible}-'].update(visible=True)

    if event == 'End Display':
        window[f'-COL{layout_visible}-'].update(visible=False)
        layout_visible = 1
        window[f'-COL{layout_visible}-'].update(visible=True)

    if event == 'Clear Plot':
        lat_data.clear()
        lon_data.clear()
        line.set_data(lat_data, lon_data)
        fig_agg.draw()

    # ---- Update data from USBReader ----
    if not reader.data_queue.empty():
        data = reader.data_queue.get()

        if "LAT" in data: 
            window['lat'].update(f"Latitude: {data['LAT']:.6f}")
            lat_data.append(data['LAT'])
        if "LON" in data: 
            window['lon'].update(f"Longitude: {data['LON']:.6f}")
            lon_data.append(data['LON'])
        if "ALT" in data: window['alt'].update(f"Altitude: {data['ALT']:.1f} m")
        if "AX" in data: window['ax'].update(f"Ax: {data['AX']:.2f} m/s²")
        if "AY" in data: window['ay'].update(f"Ay: {data['AY']:.2f} m/s²")
        if "AZ" in data: window['az'].update(f"Az: {data['AZ']:.2f} m/s²")

        # ---- Update satellites counter ----
        satellites += 1
        window['sats'].update(f"Satellites: {satellites}")

        # ---- Update plot ----
        line.set_data(lon_data, lat_data)
        ax.relim()
        ax.autoscale_view()
        fig_agg.draw()

# ---- Cleanup ----
reader.stop()
window.close()
