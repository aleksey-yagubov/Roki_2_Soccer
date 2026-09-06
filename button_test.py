import os
import select
import time

from evdev import InputDevice, ecodes, list_devices


BUTTON_DEVICE_PATH = "/dev/input/roki-head-buttons"
BUTTON_DEVICE_NAME = "roki-head-buttons"
BUTTON_CODES = {
    1: ecodes.KEY_F13,
    2: ecodes.KEY_F14,
    3: ecodes.KEY_F15,
    4: ecodes.KEY_F16,
}


def open_button_device():
    if os.path.exists(BUTTON_DEVICE_PATH):
        return InputDevice(BUTTON_DEVICE_PATH)
    for path in list_devices():
        device = InputDevice(path)
        if device.name == BUTTON_DEVICE_NAME:
            return device
        device.close()
    raise FileNotFoundError(f"Input device '{BUTTON_DEVICE_NAME}' not found")


class HeadButtons:
    def __init__(self):
        self.device = open_button_device()

    def close(self):
        self.device.close()

    def read_events(self, timeout=None):
        ready, _, _ = select.select([self.device.fd], [], [], timeout)
        if not ready:
            return []
        return list(self.device.read())


class Button_Test:
    def __init__(self, list_of_labels):
        self.list_of_labels = list_of_labels
        self.buttons = HeadButtons()

    def _speak(self, loudness, message):
        os.system("espeak -ven-m1 -a" + loudness + " " + message)

    def _select_variant_while_held(self, button, loudness):
        variants = self.list_of_labels[button]
        if not variants:
            return None
        variant = 0
        current = variants[variant]
        self._speak(loudness, current)
        last_advance = time.perf_counter()
        target_code = BUTTON_CODES[button]
        while True:
            now = time.perf_counter()
            if now - last_advance >= 0.5 and variant < len(variants) - 1:
                variant += 1
                current = variants[variant]
                self._speak(loudness, current)
                last_advance = now
            for event in self.buttons.read_events(timeout=0.05):
                if event.type != ecodes.EV_KEY or event.code != target_code:
                    continue
                if event.value == 0:
                    return current

    def wait_for_button_pressing(self, loudness='200', message="'Waiting for button'"):
        self._speak(loudness, message)
        counter1 = 0
        period = 200
        while True:
            for event in self.buttons.read_events(timeout=0.05):
                if event.type != ecodes.EV_KEY or event.value != 1:
                    continue
                for button, code in BUTTON_CODES.items():
                    if event.code == code and button < len(self.list_of_labels):
                        result = self._select_variant_while_held(button, loudness)
                        if result is not None:
                            return result
            if counter1 > period:
                period *= 1.5
                self._speak(loudness, message)
                counter1 = 0
            else:
                counter1 += 1


if __name__ == "__main__":
    labels = [[], ['one', 'two', 'three'], ['four', 'five', 'six'], ['seven', 'eight', 'nine']]
    button = Button_Test(labels)
    print(button.wait_for_button_pressing())
