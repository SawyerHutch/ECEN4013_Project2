import serial
import threading
import queue

class USBReader:
    def __init__(self, port='COM4', baud=115200):
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            print(f"Opened serial port {port} at {baud} baud.")
        except Exception as e:
            print(f"Failed to open serial port {port}: {e}")
            raise e

        self.data_queue = queue.Queue()
        self.running = True
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def read_loop(self):
        while self.running:
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line:
                    print("Received line:", line)  # DEBUG
                    parsed = self.parse_line(line)
                    if parsed:
                        self.data_queue.put(parsed)
                    else:
                        print("Failed to parse line:", line)
            except Exception as e:
                print("Serial error:", e)

    def parse_line(self, line):
        """
        Parse CSV key,value,key,value,... lines into a dictionary.
        Returns None if parsing fails.
        """
        try:
            items = line.split(',')
            if len(items) % 2 != 0:
                return None
            data = {items[i]: float(items[i+1]) for i in range(0, len(items), 2)}
            return data
        except Exception as e:
            print("Parse exception:", e)
            return None

    def stop(self):
        self.running = False
        self.thread.join()
        self.ser.close()
        print("Serial connection closed.")
