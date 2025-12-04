# Updated parse code for new project.ino code

import serial
import threading
import queue

class USBReader:
    def __init__(self, port='COM5', baud=115200):
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
            except Exception as e:
                print("Serial error:", e)

    def parse_line(self, line):
        """
        Parse CSV key,value,key,value,... lines.
        Keep ONLY numeric values (GUI expects floats).
        Ignore anything else.
        """
        try:
            items = line.split(',')
            if len(items) < 2:
                return None

            data = {}
            for i in range(0, len(items)-1, 2):
                key = items[i].strip()
                val = items[i+1].strip()

                # Attempt numeric conversion
                try:
                    data[key] = float(val)
                except:
                    continue  # skip non-numeric values like TIME, DATE

            return data if data else None

        except Exception as e:
            print("Parse exception:", e)
            return None

    def stop(self):
        self.running = False
        self.thread.join()
        self.ser.close()
        print("Serial connection closed.")
