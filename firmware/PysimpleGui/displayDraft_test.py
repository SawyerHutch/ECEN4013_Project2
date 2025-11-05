import FreeSimpleGUI as sg
import serial

sats = 0

# ---- Home screen layout ----
home = [
    [sg.Button("Start Display", button_color="green"),
     sg.Button("End Session", button_color="red")]
]

# ---- Data display layout ----
dataDisplay = [
    [sg.Button("End Display", button_color="red")],
    [sg.Text("GPS", justification="center", font=('Arial Bold', 20))],
    [sg.Text("Satellites: ", justification="left"),
     sg.Text(sats, key='sats')]
]

# ---- Layout arrangement ----
layout = [
    [sg.Column(home, key='-COL1-'),
     sg.Column(dataDisplay, visible=False, key='-COL2-')]
]

window = sg.Window("Serial Display GUI", layout, margins=(200, 100))
layout = 1

# ---- Main event loop ----
while True:
    event, values = window.read(timeout=500)

    if event in (None, 'End Session'):
        break

    if event == 'Start Display':
        window[f'-COL{layout}-'].update(visible=False)
        layout = 2
        window[f'-COL{layout}-'].update(visible=True)

    if event == 'End Display':
        window[f'-COL{layout}-'].update(visible=False)
        layout = 1
        window[f'-COL{layout}-'].update(visible=True)

    if event == sg.TIMEOUT_EVENT:
        sats += 1
        window['sats'].update(sats)

window.close()
