#!/usr/bin/env python3

"""
PipeWire-compatible rotary encoder volume daemon for Raspberry Pi.

- Increases/decreases volume via a rotary encoder.
- Toggles mute via encoder press.
- Works with PipeWire sinks (automatically finds the first hardware sink).
"""

import os
import signal
import subprocess
import sys
import threading
import time

from RPi import GPIO
from queue import Queue

DEBUG = False

# SETTINGS
GPIO_A = 6
GPIO_B = 5
GPIO_BUTTON = 26
VOLUME_MIN = 20
VOLUME_MAX = 100
VOLUME_INCREMENT = 1

# Queue and event for thread-safe callbacks
QUEUE = Queue()
EVENT = threading.Event()


def debug(msg):
    if DEBUG:
        print(msg)


class RotaryEncoder:
    """Decode mechanical rotary encoder pulses."""

    def __init__(self, gpioA, gpioB, callback=None, buttonPin=None, buttonCallback=None):
        self.lastGpio = None
        self.gpioA = gpioA
        self.gpioB = gpioB
        self.callback = callback
        self.gpioButton = buttonPin
        self.buttonCallback = buttonCallback
        self.levA = 0
        self.levB = 0

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpioA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.gpioB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.gpioA, GPIO.BOTH, self._callback)
        GPIO.add_event_detect(self.gpioB, GPIO.BOTH, self._callback)

        if self.gpioButton:
            GPIO.setup(self.gpioButton, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(
                self.gpioButton, GPIO.FALLING, self._buttonCallback, bouncetime=500
            )

    def destroy(self):
        GPIO.remove_event_detect(self.gpioA)
        GPIO.remove_event_detect(self.gpioB)
        if self.gpioButton:
            GPIO.remove_event_detect(self.gpioButton)
        GPIO.cleanup()

    def _buttonCallback(self, channel):
        self.buttonCallback(GPIO.input(channel))

    def _callback(self, channel):
        self.levA = GPIO.input(self.gpioA)
        self.levB = GPIO.input(self.gpioB)
        if channel == self.lastGpio:
            return
        self.lastGpio = channel
        if self.levB == self.levA:
            if channel == self.gpioA:
                self.callback(1)
            else:
                self.callback(-1)


class VolumeError(Exception):
    pass


class Volume:
    """PipeWire volume control using pactl."""

    MIN = VOLUME_MIN
    MAX = VOLUME_MAX
    INCREMENT = VOLUME_INCREMENT

    def __init__(self):
        self.SINK = self.get_hardware_sink()
        self.last_volume = self.MIN
        self._sync()

    # --- helper to run shell commands ---
    def _run(self, cmd):
        #print(cmd)
        p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = p.communicate()
        if p.returncode != 0:
            raise VolumeError(err.decode())
        return out.decode()

    # --- find first hardware sink (ignores virtual sinks) ---
    def get_hardware_sink(self):
#        sinks = self._run("pactl list short sinks")
#        for line in sinks.splitlines():
#            parts = line.split()
#            idx, name, driver = parts[0], parts[1], parts[2]
#            if "virtual" not in driver.lower() and "filter" not in name.lower():
#                return name
        return "@DEFAULT_SINK@"

    # --- volume operations ---
    def up(self):
        return self.change(self.INCREMENT)

    def down(self):
        return self.change(-self.INCREMENT)

    def change(self, delta):
        v = self.volume + delta
        v = self._constrain(v)
        return self.set_volume(v)

    def set_volume(self, v):
        self.volume = self._constrain(v)
        self._run(f"pactl set-sink-volume {self.SINK} {self.volume}%")
        return self.volume

    def toggle(self):
        if self.is_muted:
            self._run(f"pactl set-sink-mute {self.SINK} 0")
            self.set_volume(self.last_volume)
        else:
            self.last_volume = self.volume
            self._run(f"pactl set-sink-mute {self.SINK} 1")
        self._sync()
        return self.is_muted

    def status(self):
        if self.is_muted:
            return f"{self.volume}% (muted)"
        return f"{self.volume}%"

    # --- sync volume/mute state from PipeWire ---
    def _sync(self):
        output = self._run(f"pactl get-sink-volume {self.SINK}")
        mute = self._run(f"pactl get-sink-mute {self.SINK}")
        try:
            pct = output.split("/")[1].replace("%", "").strip()
            self.volume = int(pct)
        except:
            self.volume = self.MIN
        self.is_muted = "yes" in mute.lower()

    def _constrain(self, v):
        return max(self.MIN, min(self.MAX, v))


# ---------------- Main ----------------
if __name__ == "__main__":
    v = Volume()

    # --- Volume watcher thread ---
    def volume_watcher():
        while True:
            try:
                v._sync()
            except Exception as e:
                debug(f"Volume watcher error: {e}")
            time.sleep(1)  # poll every 1000ms

    threading.Thread(target=volume_watcher, daemon=True).start()

    def on_press(value):
        v.toggle()
        print(f"Toggled mute to: {v.is_muted}")
        EVENT.set()

    def on_turn(delta):
        QUEUE.put(delta)
        EVENT.set()

    def consume_queue():
        while not QUEUE.empty():
            delta = QUEUE.get()
            handle_delta(delta)

    def handle_delta(delta):
        if v.is_muted:
            debug("Unmuting")
            v.toggle()
        if delta == 1:
            vol = v.up()
        else:
            vol = v.down()
        print(f"Set volume to: {vol}")

    def on_exit(a, b):
        print("Exiting...")
        encoder.destroy()
        sys.exit(0)

    debug(f"Volume knob using pins {GPIO_A} and {GPIO_B}")
    if GPIO_BUTTON is not None:
        debug(f"Volume button using pin {GPIO_BUTTON}")
    debug(f"Initial volume: {v.volume}")

    encoder = RotaryEncoder(GPIO_A, GPIO_B, callback=on_turn, buttonPin=GPIO_BUTTON, buttonCallback=on_press)
    signal.signal(signal.SIGINT, on_exit)

    while True:
        EVENT.wait(1200)
        consume_queue()
        EVENT.clear()
