import serial, time
import threading

class Led:
    def __init__(self):
        self.blink = threading.Event()
        self.ser = None
        try:
            self.ser = serial.Serial('/dev/shm/bluecoin') #Open named port
            self.ser.baudrate = 115200
        except (OSError, serial.SerialException) as exc:
            print(f"[WARNING] BlueCoin LED disabled: {exc}")
            return

        self.led_thread = threading.Thread(target = self.waithng_for_led_blink, args = (self.blink,))
        self.led_thread.daemon = True
        self.led_thread.start()
        
    def waithng_for_led_blink(self, event):
        while True:
            if event.is_set():
                try:
                    self.ser.write(b'\x07')#White LED ON
                    time.sleep(0.01)
                    self.ser.write(b'\x00')
                except (OSError, serial.SerialException) as exc:
                    print(f"[WARNING] BlueCoin LED disabled: {exc}")
                    self.ser = None
                    return
                event.clear()
            time.sleep(0.5)

if __name__=="__main__":
    led = Led();
    for _ in range(20):
        a = input('click:')
        led.blink.set()
    pass
