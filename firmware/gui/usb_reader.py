import serial
import threading
import queue

class USBReader:
    def __init__(self, port='COM5', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.data_queue = queue.Queue()
        self.running = True
        self.thread = threading.Thread(target=self.read_loop)
        self.thread.start()

    def read_loop(self):
        while self.running:
            try:
                line = self.ser.readline().decode().strip()
                if line:
                    parsed = self.parse_line(line)
                    if parsed:
                        self.data_queue.put(parsed)
            except Exception as e:
                print("Serial error:", e)

    def parse_line(self, line):
        try:
            items = line.split(',')
            data = {items[i]: float(items[i+1]) for i in range(0, len(items), 2)}
            return data
        except:
            return None

    def stop(self):
        self.running = False
        self.thread.join()
        self.ser.close()


