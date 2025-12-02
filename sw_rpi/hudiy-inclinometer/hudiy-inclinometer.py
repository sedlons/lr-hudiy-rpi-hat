#!/usr/bin/env python3
import os, sys, json, asyncio, signal, math
import subprocess, venv, contextlib
from typing import Set
import time
import websockets
from websockets.legacy.server import WebSocketServerProtocol, serve
import smbus2

# ----------------- Configuration -----------------
HOST = "0.0.0.0"
PORT = 8765
BUS_NUM = 1
I2C_ADDR = 0x68
FAST_INTERVAL = 0.01 # Fast reading interval of I2C sensor
PUBLISH_INTERVAL = 0.4   # 5 Hz
DEBUG = True
CALIBRATION_FILE = "settings.json"

# ----------------- Globals -----------------
CLIENTS: Set[WebSocketServerProtocol] = set()
latest_data = {"roll":0.0, "pitch":0.0, "roll_alert":"none", "pitch_alert":"none"}
roll_filtered = 0.0
pitch_filtered = 0.0
alpha = 0.5  # complementary filter smoothing

# ----------------- Helper Functions -----------------
def load_calibration():
    if os.path.exists(CALIBRATION_FILE):
        with open(CALIBRATION_FILE,"r") as f: return json.load(f)
    return {
        "roll_alert_blink":15, "roll_alert_red":25,
        "pitch_alert_blink":10, "pitch_alert_red":20,
        "zero_roll":0.0, "zero_pitch":0.0
    }

bus = smbus2.SMBus(BUS_NUM)
# Wake up sensor
bus.write_byte_data(I2C_ADDR, 0x6B, 0)

def read_i2c_sensor():
    """
    Reads raw roll/pitch from the I2C sensor and applies zeroed offsets.
    Returns: roll, pitch in degrees
    """
    try:
        data = bus.read_i2c_block_data(I2C_ADDR, 0x3B, 6)  # Example: replace with your sensor
        acc_x = (data[0] << 8 | data[1])
        acc_y = (data[2] << 8 | data[3])
        acc_z = (data[4] << 8 | data[5])

        # Convert from 16-bit signed
        acc_x = acc_x if acc_x < 32768 else acc_x - 65536
        acc_y = acc_y if acc_y < 32768 else acc_y - 65536
        acc_z = acc_z if acc_z < 32768 else acc_z - 65536

        roll = math.degrees(math.atan2(acc_y, acc_z))
        pitch = math.degrees(math.atan2(-acc_x, math.sqrt(acc_y**2 + acc_z**2)))

    except Exception as e:
        print(f"[I2C] Read error: {e}", file=sys.stderr)


    return round(roll, 1), round(pitch, 1)


def compute_alert(value, blink_thresh, red_thresh):
    abs_val = abs(value)
    if abs_val >= red_thresh: return "red"
    if abs_val >= blink_thresh: return "blink"
    return "none"

async def sensor_loop(calib):
    global latest_data, roll_filtered, pitch_filtered
    while True:
        roll_raw, pitch_raw = read_i2c_sensor()
        roll = roll_raw - calib.get("zero_roll",0.0)
        pitch = pitch_raw - calib.get("zero_pitch",0.0)
        # complementary filter
        roll_filtered  = alpha * roll  + (1 - alpha) * roll_filtered
        pitch_filtered = alpha * pitch + (1 - alpha) * pitch_filtered
        # alerts
        roll_alert  = compute_alert(roll_filtered, calib["roll_alert_blink"], calib["roll_alert_red"])
        pitch_alert = compute_alert(pitch_filtered, calib["pitch_alert_blink"], calib["pitch_alert_red"])
        # store
        latest_data = {"roll":roll_filtered, "pitch":pitch_filtered,
                       "roll_alert":roll_alert, "pitch_alert":pitch_alert}
        await asyncio.sleep(FAST_INTERVAL)

# ----------------- WebSocket -----------------
async def broadcast(msg):
    if not CLIENTS: return
    text = json.dumps(msg)
    to_close = set()
    await asyncio.gather(*(safe_send(ws, text, to_close) for ws in list(CLIENTS)), return_exceptions=True)
    for ws in to_close: CLIENTS.discard(ws)

async def safe_send(ws, text, to_close):
    try: await ws.send(text)
    except: to_close.add(ws)

async def handler(ws):
    CLIENTS.add(ws)
    try:
        async for _ in ws: pass
    finally:
        CLIENTS.discard(ws)

async def producer_loop() -> None:
    while True:
        try:
            await broadcast(latest_data)
        except Exception as e:
            print(f"[producer] Error: {e}", file=sys.stderr)
        await asyncio.sleep(PUBLISH_INTERVAL)

async def main():
    loop = asyncio.get_running_loop()
    stop = loop.create_future()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set_result, None)
    async with serve(handler, HOST, PORT, ping_interval=20, ping_timeout=20):
        print(f"[WS] running on ws://{HOST}:{PORT}")
        sensor_task = asyncio.create_task(sensor_loop(calib))
        prod = asyncio.create_task(producer_loop())
        await stop
        sensor_task.cancel()
        prod.cancel()
        with contextlib.suppress(asyncio.CancelledError): 
            await sensor_task
            await prod

if __name__ == "__main__":
    calib = load_calibration()
    try: asyncio.run(main())
    except KeyboardInterrupt: pass


