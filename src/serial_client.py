import sys
import threading
import queue
import serial
from PySide6.QtCore import QObject, Signal

class SerialClient(QObject):
    data_received = Signal(object)

    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.running = False
        self.read_thread = None

    def connect(self, port, baudrate=115200):
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop)
            self.read_thread.daemon = True
            self.read_thread.start()
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def send_data(self, data):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(data)

    def _read_loop(self):
        buffer = b''
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer += data
                    while True:
                        start = buffer.find(b'\xFA\xAF')
                        if start == -1:
                            buffer = b''
                            break
                        end = buffer.find(b'\xFB\xBF', start)
                        if end == -1:
                            buffer = buffer[start:]
                            break
                        packet = buffer[start:end+2]
                        self.data_received.emit(packet)
                        buffer = buffer[end+2:]
                else:
                    threading.Event().wait(0.01)
            except Exception as e:
                print(f"读错误: {e}")
                break