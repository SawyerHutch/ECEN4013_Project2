import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Plotter:
    def __init__(self):
        self.lat_data = []
        self.lon_data = []
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'ro-')

    def update(self, data):
        if "LAT" in data and "LON" in data:
            self.lat_data.append(data["LAT"])
            self.lon_data.append(data["LON"])
            self.line.set_data(self.lon_data, self.lat_data)
            self.ax.relim()
            self.ax.autoscale_view()

    def animate(self):
        ani = animation.FuncAnimation(self.fig, self.update, interval=500)
        plt.show()
