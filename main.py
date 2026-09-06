from multiprocessing import freeze_support
import datetime
import os
import signal
import subprocess
import sys
import time

from evdev import ecodes

from button_test import HeadButtons


def terminate_process_tree(process):
    process.terminate()
    if os.path.isfile('/dev/shm/process.txt'):
        with open('/dev/shm/process.txt', 'r') as process_file:
            pid = int(process_file.read())
            try:
                os.kill(pid, signal.SIGTERM)
            except Exception:
                pass
        os.remove('/dev/shm/process.txt')


if __name__ == '__main__':
    freeze_support()
    filename01 = "output.txt"
    buttons = HeadButtons()
    while True:
        time.sleep(1)
        print('New process')
        with open(filename01, "a") as f01:
            print(datetime.datetime.now(), file=f01)
            p01 = subprocess.Popen([sys.executable, 'main_killable_all.py'], stderr=f01)
        message_was_sounded = False
        counter = 0
        while True:
            p01.poll()
            if p01.returncode == 1:
                if not message_was_sounded or counter > 100:
                    os.system("espeak -ven-m1 -a200 'Termination with error'")
                    message_was_sounded = True
                    counter = 0
            if p01.returncode == 5:
                if not message_was_sounded or counter > 100:
                    os.system("espeak -ven-m1 -a200 'Process finished'")
                    message_was_sounded = True
                    counter = 0
            pressed_reload = False
            for event in buttons.read_events(timeout=0):
                if event.type != ecodes.EV_KEY or event.code != ecodes.KEY_F14:
                    continue
                if event.value == 1:
                    pressed_reload = True
                    break
            if pressed_reload:
                os.system("espeak -ven-m1 -a200 'Process re-load'")
                terminate_process_tree(p01)
                p01.poll()
                print('returncode =', p01.returncode)
                timer1 = time.perf_counter()
                while True:
                    released = False
                    for event in buttons.read_events(timeout=0.1):
                        if event.type != ecodes.EV_KEY or event.code != ecodes.KEY_F14:
                            continue
                        if event.value == 0:
                            released = True
                            break
                    if released:
                        break
                    if time.perf_counter() - timer1 > 2:
                        os.system("espeak -ven-m1 -a200 'Exit from program'")
                        sys.exit(0)
                break
            time.sleep(0.5)
            counter += 1
