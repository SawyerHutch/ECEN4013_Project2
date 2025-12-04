# Updated GUI for new project.ino code

import FreeSimpleGUI as sg
from usb_reader import USBReader
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ---- USBReader ----
reader = USBReader(port='COM5')

# ---- Matplotlib Figure ----
fig, ax = plt.subplots()
line, = ax.plot([], [], 'ro-')
ax.set_xlabel("Longitude")
ax.set_ylabel("Latitude")
ax.set_title("GPS Track")
lat_data = []
lon_data = []

# ---- Helper to embed Matplotlib in GUI ----
def draw_figure(canvas, figure):
    fig_canvas = FigureCanvasTkAgg(figure, canvas)
    fig_canvas.draw()
    fig_canvas.get_tk_widget().pack(side='top', fill='both', expand=1)
    return fig_canvas

# ---- Home Screen ----
home = [
    [sg.Button("Start Display", button_color="green"),
     sg.Button("End Session", button_color="red")]
]

# ---- Data Display Layout ----
dataDisplay = [
    [sg.Button("End Display", button_color="red"),
     sg.Button("Clear Plot", button_color="orange")],

    [sg.Text("GPS Data", justification="center", font=('Arial Bold', 20))],
    [sg.Text("Latitude: --", key='lat')],
    [sg.Text("Longitude: --", key='lon')],
    [sg.Text("Altitude: -- m", key='alt')],
    [sg.Text("Satellites: 0", key='sats')],

    [sg.Text("IMU Data", justification="center", font=('Arial Bold', 20))],
    [sg.Text("Gyroscope (rad/s)", font=('Arial Bold', 14))],
    [sg.Text("GX: --", key='gx')],
    [sg.Text("GY: --", key='gy')],
    [sg.Text("GZ: --", key='gz')],

    [sg.Text("Accelerometer (m/s²)", font=('Arial Bold', 14))],
    [sg.Text("AX: --", key='ax')],
    [sg.Text("AY: --", key='ay')],
    [sg.Text("AZ: --", key='az')],

    [sg.Text("Magnetic Field (µT)", font=('Arial Bold', 14))],
    [sg.Text("MX: --", key='mx')],
    [sg.Text("MY: --", key='my')],
    [sg.Text("MZ: --", key='mz')],

    [sg.Canvas(size=(400, 300), key='-CANVAS-')]
]

# ---- Overall Layout ----
layout = [
    [sg.Column(home, key='-COL1-'),
     sg.Column(dataDisplay, visible=False, key='-COL2-')]
]

window = sg.Window("Localization Device Dashboard",
                   layout, finalize=True, margins=(50, 50))

canvas_elem = window['-CANVAS-']
canvas = canvas_elem.Widget
fig_agg = draw_figure(canvas, fig)

layout_visible = 1

# ==== MAIN GUI LOOP ====
while True:
    event, values = window.read(timeout=100)

    if event in (None, 'End Session'):
        break

    if event == 'Start Display':
        window['-COL1-'].update(visible=False)
        window['-COL2-'].update(visible=True)
        layout_visible = 2

    if event == 'End Display':
        window['-COL2-'].update(visible=False)
        window['-COL1-'].update(visible=True)
        layout_visible = 1

    if event == 'Clear Plot':
        lat_data.clear()
        lon_data.clear()
        line.set_data([], [])
        fig_agg.draw()

    # ===== UPDATE LIVE SENSOR DATA =====
    while not reader.data_queue.empty():
        data = reader.data_queue.get()

        # --- GPS ---
        if "LAT" in data:
            window['lat'].update(f"Latitude: {data['LAT']:.6f}")
            lat_data.append(data['LAT'])

        if "LON" in data:
            window['lon'].update(f"Longitude: {data['LON']:.6f}")
            lon_data.append(data['LON'])

        if "ALT" in data:
            window['alt'].update(f"Altitude: {data['ALT']:.1f} m")

        if "SATS" in data:
            window['sats'].update(f"Satellites: {int(data['SATS'])}")

        # --- Gyroscope ---
        if "GX" in data: window['gx'].update(f"GX: {data['GX']:.3f}")
        if "GY" in data: window['gy'].update(f"GY: {data['GY']:.3f}")
        if "GZ" in data: window['gz'].update(f"GZ: {data['GZ']:.3f}")

        # --- Accelerometer ---
        if "AX" in data: window['ax'].update(f"AX: {data['AX']:.3f}")
        if "AY" in data: window['ay'].update(f"AY: {data['AY']:.3f}")
        if "AZ" in data: window['az'].update(f"AZ: {data['AZ']:.3f}")

        # --- Magnetometer ---
        if "MX" in data: window['mx'].update(f"MX: {data['MX']:.3f}")
        if "MY" in data: window['my'].update(f"MY: {data['MY']:.3f}")
        if "MZ" in data: window['mz'].update(f"MZ: {data['MZ']:.3f}")

        # --- Update Plot ---
        line.set_data(lon_data, lat_data)
        ax.relim()
        ax.autoscale_view()
        fig_agg.draw()

window.close()
