import tkinter as tk
from tkinter import ttk
from usb_reader import USBReader
import threading

class LocalizationGUI:
    def __init__(self, root):
        self.root = root
        root.title("Localization Device Dashboard")

        # GPS info labels
        self.lat_label = ttk.Label(root, text="Latitude: --")
        self.lon_label = ttk.Label(root, text="Longitude: --")
        self.alt_label = ttk.Label(root, text="Altitude: -- m")

        # IMU info labels
        self.ax_label = ttk.Label(root, text="Ax: -- m/s²")
        self.ay_label = ttk.Label(root, text="Ay: -- m/s²")
        self.az_label = ttk.Label(root, text="Az: -- m/s²")

        # Layout
        self.lat_label.pack(pady=3)
        self.lon_label.pack(pady=3)
        self.alt_label.pack(pady=3)
        ttk.Separator(root, orient="horizontal").pack(fill="x", pady=5)
        self.ax_label.pack(pady=3)
        self.ay_label.pack(pady=3)
        self.az_label.pack(pady=3)

        # Start USB thread
        self.reader = USBReader(port='COM5')
        self.update_gui()

    def update_gui(self):
        if not self.reader.data_queue.empty():
            data = self.reader.data_queue.get()

            if "LAT" in data: self.lat_label.config(text=f"Latitude: {data['LAT']:.6f}")
            if "LON" in data: self.lon_label.config(text=f"Longitude: {data['LON']:.6f}")
            if "ALT" in data: self.alt_label.config(text=f"Altitude: {data['ALT']:.1f} m")
            if "AX" in data: self.ax_label.config(text=f"Ax: {data['AX']:.2f} m/s²")
            if "AY" in data: self.ay_label.config(text=f"Ay: {data['AY']:.2f} m/s²")
            if "AZ" in data: self.az_label.config(text=f"Az: {data['AZ']:.2f} m/s²")

        self.root.after(100, self.update_gui)  # refresh every 0.1s

if __name__ == "__main__":
    root = tk.Tk()
    app = LocalizationGUI(root)
    root.mainloop()
