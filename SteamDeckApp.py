# ============================================================
# SteamDeckApp.py  –  Bullseye Robot Steam Deck GUI
# ============================================================
# This is the Steam Deck entry point for the Bullseye controller.
# It replaces ControllerMessager.py as the primary controller-side
# program. ControllerMessager.py is intentionally kept intact as a
# headless fallback for hardware testing without the GUI.
#
# THREAD MODEL:
#   Main Thread     – CustomTkinter mainloop; all UI draws and callbacks
#   JoystickThread  – polls pygame joystick at ~20 Hz, writes to AppState
#   SerialTXThread  – drains tx_queue and sends joystick deltas over XBee
#   SerialRXThread  – reads inbound packets (battery, path_created events)
#
# LOCAL STORAGE:
#   routes.json     – maps route_id (int) → user-given name (str)
#   kfx_config.json – maps KFX button number ("3"–"8") → route_id or null
#
# ============================================================
# TOGGLE FLAGS
# ============================================================
DEBUG_OVERLAY      = False    # Show semi-transparent TX log overlay on screen
REQUIRE_CONNECTION = False   # True = halt on missing XBee; False = UI-only mode
STARTUP            = True    # True = show START button (debug bypass); False = lock on startup until ping_ack CHANGE THIS BACK BEFORE
KFX_SPEED          = 0.5     # Global KFX run speed (0.0–1.0); sent to Pi via kfx_speed packet
# ============================================================

import customtkinter as ctk
import tkinter as tk
import pygame
import serial
import threading
import queue
import json
import time
import os
import sys
from pathlib import Path
from collections import deque
from PIL import Image


# ── Package path so Comms/Robot imports resolve from project root ─────────────
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from Models.ControllerData import ControllerData
from Models.StateData import State, StateData
from Models.BatteryData import BatteryData
from Models.DataPacket import DataPacket
from Models.PingAckData import PingAckData
from Models.PosData import PosData
from Models.HomeCheckResult import HomeCheckResult
from Constants import Constants

# ============================================================
# APP-LEVEL CONSTANTS  –  hardware and timing
# ============================================================
WINDOW_W   = 1280 #steam deck dimensions - NEVER CHANGE
WINDOW_H   = 800

UPDATE_HZ  = Constants.controller_update_rate   # seconds per cycle (~20 Hz = 0.05 s)
DEADZONE   = Constants.controller_deadzone       # joystick axis dead-band
RECORD_FINISH_TIMEOUT = 10.0                    # seconds to wait for Pi path_created ack
PORT       = Constants.controller_serial_port    # XBee USB serial port
BAUD       = Constants.serial_baud_rate          # XBee baud rate
PING_INTERVAL_MS  = 500                          # ms between ping packets on startup screen
ACK_TIMEOUT_MS    = 1000                         # ms before any ack is considered timed out
HOME_MATCH_TOLERANCE = 0.0  # 0.0 = exact match; increase for positional tolerance (metres / degrees)

# ── Color palette ─────────────────────────────────────────────────────────────
C_PRIMARY   = "#5c5c5c"   # Dark grey   – inactive buttons, phone body
C_SECONDARY = "#0033A0"   # Deep blue   – primary action buttons, highlights
C_TERTIARY  = "#FFD100"   # Gold        – accent, selected states, active text
C_BG        = "#1a1a1a"   # Near-black  – root window background
C_SURFACE   = "#2e2e2e"   # Dark grey   – card and frame backgrounds
C_TEXT      = "#f0f0f0"   # Off-white   – primary readable text
C_MUTED     = "#9a9a9a"   # Grey        – secondary text, disabled items
C_DANGER    = "#cc2200"   # Red         – stop / cancel / warning actions
C_SUCCESS   = "#1a7a1a"   # Green       – set home, positive/confirm actions

# ── Local storage file paths ──────────────────────────────────────────────────
ROUTES_FILE     = Path(__file__).parent / "routes.json"
KFX_CONFIG_FILE = Path(__file__).parent / "kfx_config.json"
BOUNDARY_FILE   = Path(__file__).parent / "boundary.json"
HOME_FILE       = Path(__file__).parent / "home.json"

UI_SCALE = 1.6   # tweak this (1.3 – 2.0)

BASE_FONT = int(16 * UI_SCALE)
LARGE_FONT = int(24 * UI_SCALE)
TITLE_FONT = int(36 * UI_SCALE)

ENTRY_WIDTH = int(180 * UI_SCALE)
ENTRY_HEIGHT = int(40 * UI_SCALE)

# ============================================================
# DELTA ENCODING  –  mirrors ControllerMessager.py exactly
# The receiver (PiCommThread) uses the inverse map to reconstruct
# full ControllerData fields from the compact short keys.
# ============================================================
_FIELD_MAP = {
    'left_x': 'lx', 'left_y': 'ly', 'right_x': 'rx', 'right_y': 'ry',
    'dpad_up': 'du', 'dpad_down': 'dd', 'dpad_left': 'dl', 'dpad_right': 'dr',
    'btn_A': 'ba',  'btn_B': 'bb',  'btn_X': 'bx',  'btn_Y': 'by',
    'btn_LB': 'lb', 'btn_RB': 'rb', 'btn_LS': 'ls', 'btn_RS': 'rs',
    'btn_R2': 'r2', 'btn_L2': 'l2', 'btn_share': 'sh', 'btn_options': 'op',
}
_FLOAT_FIELDS = {'left_x', 'left_y', 'right_x', 'right_y', 'btn_R2', 'btn_L2'}


def _build_delta(current: ControllerData, previous: ControllerData | None) -> dict:
    """
    Return only the fields that changed since the last packet, using short keys.
    If previous is None (first packet ever) all fields are included.
    Floats are rounded to 2 dp to avoid sending floating-point noise.
    """
    delta = {}
    for field, short_key in _FIELD_MAP.items():
        curr_val = getattr(current, field)
        if field in _FLOAT_FIELDS:
            curr_val = round(curr_val, 2)
            prev_val = round(getattr(previous, field), 2) if previous else None
        else:
            curr_val = int(curr_val)
            prev_val = int(getattr(previous, field)) if previous else None
        if curr_val != prev_val:
            delta[short_key] = curr_val
    return delta


# ============================================================
# LOCAL STORAGE HELPERS
# ============================================================

def load_routes() -> dict:
    """
    Load route_id → {name, home} mapping from routes.json.
    Automatically migrates old format (route_id → name string) to new schema.
    Returns an empty dict if the file does not yet exist or cannot be parsed.
    Keys are stored as strings (JSON requirement); callers convert as needed.
    """
    if ROUTES_FILE.exists():
        try:
            with open(ROUTES_FILE, "r") as f:
                raw = json.load(f)
            return migrate_routes(raw)
        except Exception as e:
            print(f"[STORAGE] Could not load routes.json: {e}")
    return {}


def save_routes(routes: dict):
    """Persist the route_id → name mapping to routes.json."""
    try:
        with open(ROUTES_FILE, "w") as f:
            json.dump(routes, f, indent=2)
    except Exception as e:
        print(f"[STORAGE] Could not save routes.json: {e}")


def load_kfx_config() -> dict:
    """
    Load KFX button → route_id mapping from kfx_config.json.
    Buttons 1–6 default to None (unassigned).
    Buttons 7 and 8 are reserved (hardcoded on Pi) and are NOT stored here.
    """
    default = {"1": None, "2": None, "3": None, "4": None, "5": None, "6": None}
    if KFX_CONFIG_FILE.exists():
        try:
            with open(KFX_CONFIG_FILE, "r") as f:
                data = json.load(f)
            for k in default:          # Fill any missing keys with None
                data.setdefault(k, None)
            return data
        except Exception as e:
            print(f"[STORAGE] Could not load kfx_config.json: {e}")
    return default


def save_kfx_config(config: dict):
    """Persist KFX button assignments to kfx_config.json."""
    try:
        with open(KFX_CONFIG_FILE, "w") as f:
            json.dump(config, f, indent=2)
    except Exception as e:
        print(f"[STORAGE] Could not save kfx_config.json: {e}")


def load_boundary() -> dict | None:
    """
    Load boundary corners from boundary.json.
    Returns a dict with a 'corners' list, or None if not set yet.
    """
    if BOUNDARY_FILE.exists():
        try:
            with open(BOUNDARY_FILE, "r") as f:
                return json.load(f)
        except Exception as e:
            print(f"[STORAGE] Could not load boundary.json: {e}")
    return None


def save_boundary(boundary: dict):
    """Persist boundary corners to boundary.json."""
    try:
        with open(BOUNDARY_FILE, "w") as f:
            json.dump(boundary, f, indent=2)
    except Exception as e:
        print(f"[STORAGE] Could not save boundary.json: {e}")


def load_home() -> dict | None:
    """
    Load home position from home.json.
    Returns a dict with x, y, yaw, or None if not set yet.
    """
    if HOME_FILE.exists():
        try:
            with open(HOME_FILE, "r") as f:
                return json.load(f)
        except Exception as e:
            print(f"[STORAGE] Could not load home.json: {e}")
    return None


def save_home(home: dict):
    """Persist home position to home.json."""
    try:
        with open(HOME_FILE, "w") as f:
            json.dump(home, f, indent=2)
    except Exception as e:
        print(f"[STORAGE] Could not save home.json: {e}")


def homes_match(h1: dict | None, h2: dict | None) -> bool:
    """Return True if two home-position dicts are considered equal.

    With HOME_MATCH_TOLERANCE == 0.0 this is an exact float comparison.
    Raise the tolerance value to accept nearby positions.
    """
    if h1 is None or h2 is None:
        return False
    if HOME_MATCH_TOLERANCE == 0.0:
        return h1["x"] == h2["x"] and h1["y"] == h2["y"] and h1["yaw"] == h2["yaw"]
    return (
        abs(h1["x"] - h2["x"]) <= HOME_MATCH_TOLERANCE
        and abs(h1["y"] - h2["y"]) <= HOME_MATCH_TOLERANCE
        and abs(h1["yaw"] - h2["yaw"]) <= HOME_MATCH_TOLERANCE
    )


def clear_kfx_for_home_mismatch(state, new_home: dict):
    """Clear any KFX slot whose assigned route home doesn't match new_home.

    Saves kfx_config.json and enqueues the updated packet if anything changed.
    """
    with state.lock:
        routes = dict(state.routes)
        kfx = dict(state.kfx_config)
    changed = False
    for btn_num, assigned_id in kfx.items():
        if assigned_id is None:
            continue
        route = routes.get(str(assigned_id), {})
        route_home = route.get("home") if isinstance(route, dict) else None
        if not homes_match(route_home, new_home):
            kfx[btn_num] = None
            changed = True
    if changed:
        with state.lock:
            state.kfx_config = kfx
        save_kfx_config(kfx)
        enqueue_packet(state, "kfx_config", json.dumps(kfx))


def migrate_routes(raw: dict) -> dict:
    """
    Ensure routes.json is in the new schema:
      {"route_id": {"name": str, "home": {x, y, yaw} | None}}
    Old format was {"route_id": "name"} (plain string value).
    Converts any plain-string entries in place so the rest of the app
    can always assume the new structure.
    """
    migrated = {}
    for k, v in raw.items():
        if isinstance(v, str):
            # Old format — wrap it
            migrated[k] = {"name": v, "home": None}
        else:
            # Already new format — ensure 'home' key exists
            migrated[k] = {"name": v.get("name", ""), "home": v.get("home", None)}
    return migrated


# ============================================================
# SHARED APP STATE
# All data shared between the GUI thread and background threads
# lives here. Always acquire self.lock before reading or writing.
# ============================================================

class AppState:
    """
    Thread-safe container for all cross-thread shared data.
    The GUI reads battery/connection for display.
    Background threads write sensor data and read joystick_active.
    """
    def __init__(self):
        self.lock = threading.Lock()

        # ── Inbound data (written by RX thread, read by GUI) ─────────────
        self.battery = BatteryData(
            voltage=0.0, current=0.0, power=0.0,
            state_of_charge=0.0, time_remaining=0.0
        )

        # ── Controller input (written by JoystickThread) ──────────────────
        self.controller = ControllerData(
            left_x=0.0,   left_y=0.0,
            right_x=0.0,  right_y=0.0,
            dpad_up=False,  dpad_down=False,
            dpad_left=False, dpad_right=False,
            btn_A=False, btn_B=False, btn_X=False, btn_Y=False,
            btn_LB=False, btn_RB=False,
            btn_LS=False, btn_RS=False,
            btn_R2=-1.0, btn_L2=-1.0,
            btn_share=False, btn_options=False,
        )

        # ── Outbound packet queue (GUI → SerialTXThread) ──────────────────
        # Each item is a DataPacket ready to be serialized and sent.
        self.tx_queue: queue.Queue = queue.Queue()

        # ── Inbound event queue (SerialRXThread → GUI) ────────────────────
        # Each item is a plain dict e.g. {"type": "path_created", "route_id": 5}
        self.event_queue: queue.Queue = queue.Queue()

        # ── Current known robot state ─────────────────────────────────────
        self.robot_state: State = State.DISABLED

        # ── Joystick streaming gate ───────────────────────────────────────
        # TX thread only sends controller delta packets when this is True.
        # Set to True on entering Drive/Record screens; False on leaving.
        self.joystick_active: bool = False

        # ── Debug log (last 5 TX events for the overlay) ──────────────────
        self.debug_log: deque = deque(maxlen=5)

        # ── Route and KFX data (loaded from local JSON on startup) ────────
        self.routes: dict = load_routes()       # {"route_id_str": {"name": str, "home": {x,y,yaw}|None}}
        self.kfx_config: dict = load_kfx_config()  # {"3": id_or_None, ...}

        # ── Boundary and home position (loaded from local JSON on startup) ─
        self.boundary: dict | None = load_boundary()   # {"corners": [{x,y}×4]} or None
        self.home_pos: dict | None = load_home()       # {"x": f, "y": f, "yaw": f} or None

        # ── Connection state ──────────────────────────────────────────────
        # True once ping_ack received from Pi. When STARTUP=True this is
        # bypassed and the user can proceed directly from the startup screen.
        self.connected: bool = False

        # ── E-Stop state ──────────────────────────────────────────────────
        # Set True when the E-STOP overlay button is pressed.
        # Cleared when the user selects DRIVE or PATH MENU from MainMenuScreen.
        self.e_stop_active: bool = False


# ============================================================
# BACKGROUND THREADS
# ============================================================

class JoystickThread(threading.Thread):
    """
    Polls the Steam Deck (or any pygame-compatible) joystick at ~20 Hz.
    Writes fresh ControllerData into app_state on every cycle.
    Does NOT send packets itself – that is SerialTXThread's job.

    Button index mapping matches ControllerMessager.py so the Pi-side
    PiCommThread receives identical data regardless of which entry point
    is used.
    """
    def __init__(self, app_state: AppState):
        super().__init__(daemon=True, name="JoystickThread")
        self.app_state = app_state
        self._running = True

    def run(self):
        pygame.init()
        pygame.joystick.init()

        joystick = None
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"[JOYSTICK] Detected: {joystick.get_name()}")
        else:
            print("[JOYSTICK] No controller detected – joystick input disabled")

        while self._running:
            if joystick is None:
                time.sleep(UPDATE_HZ)
                continue

            pygame.event.pump()

            # ── Read axes ─────────────────────────────────────────────────
            left_x  =  joystick.get_axis(0)
            left_y  = -joystick.get_axis(1)   # Inverted: up = positive
            right_x = -joystick.get_axis(2)   # Inverted: right = positive
            right_y =  joystick.get_axis(3)
            l2_axis =  joystick.get_axis(4)
            r2_axis =  joystick.get_axis(5)

            # ── Apply deadzone ────────────────────────────────────────────
            left_x  = 0.0 if abs(left_x)  < DEADZONE else left_x
            left_y  = 0.0 if abs(left_y)  < DEADZONE else left_y
            right_x = 0.0 if abs(right_x) < DEADZONE else right_x
            right_y = 0.0 if abs(right_y) < DEADZONE else right_y
            l2_axis = 0.0 if abs(l2_axis) < DEADZONE else l2_axis
            r2_axis = 0.0 if abs(r2_axis) < DEADZONE else r2_axis

            # ── Read buttons (indices match ControllerMessager.py) ────────
            btn_A       = joystick.get_button(0)
            btn_B       = joystick.get_button(1)
            btn_X       = joystick.get_button(2)
            btn_Y       = joystick.get_button(3)
            btn_share   = joystick.get_button(4)
            btn_options = joystick.get_button(6)
            btn_LS      = joystick.get_button(7)
            btn_RS      = joystick.get_button(8)
            btn_LB      = joystick.get_button(9)
            btn_RB      = joystick.get_button(10)
            dpad_up     = joystick.get_button(11)
            dpad_down   = joystick.get_button(12)
            dpad_left   = joystick.get_button(13)
            dpad_right  = joystick.get_button(14)

            new_data = ControllerData(
                left_x=left_x,       left_y=left_y,
                right_x=right_x,     right_y=right_y,
                dpad_up=dpad_up,     dpad_down=dpad_down,
                dpad_left=dpad_left, dpad_right=dpad_right,
                btn_A=btn_A,         btn_B=btn_B,
                btn_X=btn_X,         btn_Y=btn_Y,
                btn_LB=btn_LB,       btn_RB=btn_RB,
                btn_LS=btn_LS,       btn_RS=btn_RS,
                btn_L2=l2_axis,      btn_R2=r2_axis,
                btn_share=btn_share, btn_options=btn_options,
            )

            with self.app_state.lock:
                self.app_state.controller = new_data

            time.sleep(UPDATE_HZ)

    def stop(self):
        self._running = False


class SerialTXThread(threading.Thread):
    """
    Drains app_state.tx_queue (command packets from GUI callbacks) and
    also sends joystick delta packets while app_state.joystick_active is True.

    Priority: command queue is always drained first each cycle so state
    changes (e.g. DISABLED) are sent before the next joystick packet.
    A small gap between consecutive packets prevents XBee from merging them.
    """
    def __init__(self, app_state: AppState, ser: serial.Serial | None):
        super().__init__(daemon=True, name="SerialTXThread")
        self.app_state = app_state
        self.ser = ser
        self._running = True
        self._prev_controller: ControllerData | None = None

    def _send(self, packet: DataPacket):
        """
        Serialize and write one packet over serial.
        Also appends a short log entry to the debug overlay queue.
        Sending is skipped if serial is unavailable (UI-only mode),
        but the log entry is always written so the overlay stays useful.
        """
        line = packet.model_dump_json() + "\n"
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(line.encode())
            except serial.SerialException as e:
                print(f"[TX ERROR] Serial write failed: {e}")

        # Always log – visible in debug overlay even without hardware
        with self.app_state.lock:
            self.app_state.debug_log.append(
                f"{time.strftime('%H:%M:%S')} TX:{packet.type} "
                f"{packet.json_data[:30]}"
            )

    def run(self):
        while self._running:
            # ── 1. Drain command queue first ──────────────────────────────
            while not self.app_state.tx_queue.empty():
                try:
                    packet = self.app_state.tx_queue.get_nowait()
                    self._send(packet)
                    # Small gap so XBee does not merge back-to-back packets
                    time.sleep(0.02)
                except queue.Empty:
                    break

            # ── 2. Send joystick delta if driving is active ───────────────
            with self.app_state.lock:
                active = self.app_state.joystick_active
                current = self.app_state.controller

            if active:
                delta = _build_delta(current, self._prev_controller)
                if delta:
                    packet = DataPacket(type="c", json_data=json.dumps(delta))
                    self._send(packet)
                self._prev_controller = current

            time.sleep(UPDATE_HZ)

    def stop(self):
        self._running = False


class SerialRXThread(threading.Thread):
    """
    Continuously reads packets arriving from the robot over serial.

    Handles:
      type="battery"       → updates app_state.battery in place
      type="path_created"  → pushes event into app_state.event_queue
                             so the GUI can prompt the user to name it

    On serial error: prints a warning and exits the receive loop.
    """
    def __init__(self, app_state: AppState, ser: serial.Serial | None):
        super().__init__(daemon=True, name="SerialRXThread")
        self.app_state = app_state
        self.ser = ser
        self._running = True

    def run(self):
        if self.ser is None:
            return   # No serial in UI-only mode – nothing to receive

        buffer = ""
        while self._running:
            try:
                if self.ser.in_waiting == 0:
                    time.sleep(0.02)
                    continue
                chunk = self.ser.read(self.ser.in_waiting).decode(errors="ignore")
                buffer += chunk

                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        self._process(line)

            except serial.SerialException as e:
                print(f"[RX ERROR] Serial lost: {e}")
                break
            except Exception as e:
                print(f"[RX ERROR] Unexpected: {e}")
                time.sleep(0.1)

    def _process(self, line: str):
        """Parse one line and update shared state or push a GUI event."""
        try:
            packet = DataPacket.model_validate_json(line)

            if packet.type == "battery":
                batt = BatteryData.model_validate_json(packet.json_data)
                with self.app_state.lock:
                    self.app_state.battery = batt
                self.app_state.event_queue.put({
                    "type": "battery_update",
                    "voltage": batt.voltage,
                    "current": batt.current,
                    "power": batt.power,
                    "state_of_charge": batt.state_of_charge,
                    "time_remaining": batt.time_remaining,
                })

            elif packet.type == "path_created":
                # Pi has finished saving a route; GUI needs the ID to prompt for a name.
                data = json.loads(packet.json_data)
                self.app_state.event_queue.put({
                    "type": "path_created",
                    "route_id": data.get("id"),
                })

            elif packet.type == "kfx_ack":
                self.app_state.event_queue.put({"type": "kfx_ack"})

            elif packet.type == "kfx_speed_ack":
                self.app_state.event_queue.put({"type": "kfx_speed_ack"})

            elif packet.type == "ping":
                # Pi's watchdog is checking we're alive — reply immediately
                enqueue_packet(self.app_state, "ping_ack")

            elif packet.type == "ping_ack":
                ack = PingAckData.model_validate_json(packet.json_data)
                with self.app_state.lock:
                    self.app_state.connected = True
                    self.app_state.battery.state_of_charge = ack.state_of_charge
                self.app_state.event_queue.put({"type": "ping_ack"})

            elif packet.type == "boundary_ack":
                self.app_state.event_queue.put({"type": "boundary_ack"})

            elif packet.type == "pos_data":
                pos = PosData.model_validate_json(packet.json_data)
                self.app_state.event_queue.put({
                    "type": "pos_data",
                    "x": pos.x, "y": pos.y, "yaw": pos.yaw,
                })

            elif packet.type == "home_ack":
                self.app_state.event_queue.put({"type": "home_ack"})

            elif packet.type == "record_home_check_result":
                result = HomeCheckResult.model_validate_json(packet.json_data)
                self.app_state.event_queue.put({
                    "type": "record_home_check_result",
                    "ok": result.ok,
                })

            elif packet.type == "error":
                data = json.loads(packet.json_data)
                self.app_state.event_queue.put({
                    "type": "error",
                    "errstr": data.get("errstr", "Unknown error"),
                })

            elif packet.type == "route_finished":
                data = json.loads(packet.json_data)
                self.app_state.event_queue.put({
                    "type": "route_finished",
                    "message": data.get("message", "Route complete"),
                })

        except Exception as e:
            print(f"[RX] Could not parse line: {line!r} → {e}")

    def stop(self):
        self._running = False

# ============================================================
# PACKET HELPERS  –  GUI callbacks use these, not the queue directly
# ============================================================

def enqueue_packet(app_state: AppState, packet_type: str, payload: str = "{}"):
    """
    Build a DataPacket and push it onto the TX queue.
    Used for simple command packets (record_start, record_cancel, etc.)
    that carry no payload or a pre-serialized JSON string.
    """
    packet = DataPacket(type=packet_type, json_data=payload)
    app_state.tx_queue.put(packet)


def enqueue_state(app_state: AppState, state: State,
                  path_id: int | None = None,
                  path_speed: float | None = None):
    """
    Send a robot mode-change packet and update the local state cache.
    Always call this (not enqueue_packet) for state transitions so
    app_state.robot_state stays in sync with what the Pi will receive.
    """
    sd = StateData(state=state, path_id=path_id, path_speed=path_speed)
    packet = DataPacket(type="state", json_data=sd.model_dump_json())
    app_state.tx_queue.put(packet)
    with app_state.lock:
        app_state.robot_state = state


# ============================================================
# REUSABLE UI HELPERS
# ============================================================

def get_steamdeck_battery() -> int | None:
    """
    Read the Steam Deck's own battery percentage from the Linux power supply
    sysfs interface. Returns an integer 0-100 on success, or None if the
    file cannot be read (e.g. running on Windows during development).

    The Steam Deck exposes battery capacity at one of these paths depending
    on kernel version; we try them in order and return the first that works.
    """
    candidates = [
        "/sys/class/power_supply/BAT1/capacity",
        "/sys/class/power_supply/BAT0/capacity",
        "/sys/class/power_supply/battery/capacity",
    ]
    for path in candidates:
        try:
            with open(path, "r") as f:
                return int(f.read().strip())
        except Exception:
            continue
    return None  # Not on Linux / sysfs not available (Windows dev machine)


def enable_touch_scroll(scrollable_frame: ctk.CTkScrollableFrame):
    """
    Bind drag-to-scroll on a CTkScrollableFrame so the user can drag the
    list up/down with a finger or mouse without needing the scrollbar thumb.

    A small movement threshold (8 px) distinguishes a tap from a drag so
    button click callbacks still fire correctly on light touches.

    Call this after building (or rebuilding) the frame's children so that
    all child widgets receive the bindings too.
    """
    _data = {"start_y": 0, "last_y": 0, "scrolling": False}

    def _on_press(e):
        _data["start_y"] = e.y_root
        _data["last_y"]  = e.y_root
        _data["scrolling"] = False

    def _on_drag(e):
        total = abs(e.y_root - _data["start_y"])
        if total > 8:
            _data["scrolling"] = True
        if not _data["scrolling"]:
            return
        dy = _data["last_y"] - e.y_root
        _data["last_y"] = e.y_root
        if dy:
            try:
                scrollable_frame._parent_canvas.yview_scroll(int(dy / 6), "units")
            except Exception:
                pass

    def _bind_widget(widget):
        widget.bind("<ButtonPress-1>", _on_press, add="+")
        widget.bind("<B1-Motion>",     _on_drag,  add="+")
        for child in widget.winfo_children():
            _bind_widget(child)

    _bind_widget(scrollable_frame)


# ============================================================
# NUMPAD OVERLAY  –  on-screen number input for entry fields
# ============================================================

class NumpadOverlay(ctk.CTkToplevel):
    """
    A floating numpad window that attaches to a CTkEntry.

    Layout:
      [ 7 ] [ 8 ] [ 9 ]  [ ⌫ ]
      [ 4 ] [ 5 ] [ 6 ]  [ C ]
      [ 1 ] [ 2 ] [ 3 ]  [ - ]
      [   0   ]  [ . ]   [DONE]

    Usage:
        NumpadOverlay(root_window, entry_widget)

    The overlay positions itself near the entry on screen.
    DONE closes it; clicking outside also closes it.
    Negative toggle (-) prepends/removes a minus sign.
    """

    _BTN_W  = 72
    _BTN_H  = 64
    _PAD    = 6

    def __init__(self, root: tk.Tk, entry: ctk.CTkEntry):
        super().__init__(root)
        self._entry = entry

        self.overrideredirect(True)          # no title bar
        self.attributes("-topmost", True)
        self.configure(bg=C_SURFACE)
        self.resizable(False, False)

        # ── Position near the entry widget ───────────────────────────────
        self.update_idletasks()
        ex = entry.winfo_rootx()
        ey = entry.winfo_rooty() + entry.winfo_height() + 4
        # Clamp so numpad doesn't go off the right/bottom of screen
        sw = self.winfo_screenwidth()
        sh = self.winfo_screenheight()
        pad_total = self._PAD * 5
        w = self._BTN_W * 4 + pad_total + 16
        h = self._BTN_H * 4 + pad_total + 16
        x = min(ex, sw - w - 4)
        y = min(ey, sh - h - 4)
        self.geometry(f"+{x}+{y}")

        # ── Build the grid ────────────────────────────────────────────────
        outer = ctk.CTkFrame(self, fg_color=C_SURFACE, corner_radius=12)
        outer.pack(padx=8, pady=8)

        def _btn(parent, text, cmd, col, row, colspan=1, color=C_PRIMARY):
            b = ctk.CTkButton(
                parent, text=text, command=cmd,
                width=self._BTN_W * colspan + self._PAD * (colspan - 1),
                height=self._BTN_H,
                font=("Arial Bold", 22),
                fg_color=color, hover_color=C_SECONDARY,
                text_color=C_TEXT, corner_radius=8,
            )
            b.grid(row=row, column=col, columnspan=colspan,
                   padx=self._PAD // 2, pady=self._PAD // 2)
            return b

        p = self._PAD // 2
        outer.grid_columnconfigure((0, 1, 2, 3), pad=p)
        outer.grid_rowconfigure((0, 1, 2, 3), pad=p)

        for digit, col, row in [
            ("7", 0, 0), ("8", 1, 0), ("9", 2, 0),
            ("4", 0, 1), ("5", 1, 1), ("6", 2, 1),
            ("1", 0, 2), ("2", 1, 2), ("3", 2, 2),
        ]:
            _btn(outer, digit, lambda d=digit: self._press(d), col, row)

        _btn(outer, "⌫",   self._backspace,        3, 0, color=C_DANGER)
        _btn(outer, "C",   self._clear,             3, 1, color=C_DANGER)
        _btn(outer, "±",   self._toggle_sign,       3, 2, color=C_PRIMARY)
        _btn(outer, "0",   lambda: self._press("0"), 0, 3, colspan=2)
        _btn(outer, ".",   lambda: self._press("."), 2, 3)
        _btn(outer, "DONE", self._done,             3, 3, color=C_SUCCESS)

        # Close on click outside
        self.bind("<FocusOut>", self._on_focus_out)
        self.after(100, lambda: self.focus_set())

    def _press(self, char: str):
        """Append char to entry, preventing duplicate decimal points."""
        current = self._entry.get()
        if char == "." and "." in current:
            return
        self._entry.insert("end", char)

    def _backspace(self):
        current = self._entry.get()
        if current:
            self._entry.delete(len(current) - 1, "end")

    def _clear(self):
        self._entry.delete(0, "end")

    def _toggle_sign(self):
        current = self._entry.get()
        if current.startswith("-"):
            self._entry.delete(0, 1)
        else:
            self._entry.insert(0, "-")

    def _done(self):
        self.destroy()

    def _on_focus_out(self, *_):
        # Give a short grace period so button clicks register first
        self.after(150, self._check_focus)

    def _check_focus(self):
        try:
            focused = self.focus_get()
            if focused is None or str(focused) not in str(self.winfo_children()):
                self.destroy()
        except Exception:
            self.destroy()


def attach_numpad(root: tk.Tk, *entries: ctk.CTkEntry):
    """
    Bind a numpad overlay to one or more CTkEntry widgets.
    Clicking/focusing an entry opens the numpad pointed at that entry.
    Only one numpad is open at a time — opening a new one closes the old.
    """
    _state = {"pad": None}

    def _open(entry):
        existing = _state["pad"]
        if existing is not None:
            try:
                existing.destroy()
            except Exception:
                pass
        pad = NumpadOverlay(root, entry)
        _state["pad"] = pad

    for e in entries:
        e.bind("<ButtonPress-1>", lambda *_, ent=e: _open(ent), add="+")


def make_nav_button(parent, text: str, command,
                    color: str = C_TEXT, #making the button a matching gray
                    width: int = 600,
                    height: int = 100) -> ctk.CTkButton:
    """
    Standard large touch-friendly navigation button used across all screens.
    Hover color is always gold (C_TERTIARY) for visual consistency.
    """
    return ctk.CTkButton(
        parent, text=text, command=command,
        font=("Arial", 56, "bold"),
        fg_color=color, hover_color=C_TERTIARY,
        text_color=C_BG, corner_radius=30, ##making the color black of the text
        width=width, height=height,
    )


# ============================================================
# DEBUG OVERLAY
# Rendered as a native tk.Frame (not CTk) so it can sit on top
# of everything using place() coordinates relative to the root.
# Persists across frame swaps because it lives on the root window.
# ============================================================

class DebugOverlay:
    """
    Semi-transparent command log panel, bottom-right corner.
    Shows the last 5 packets sent to the robot, refreshed every 500 ms.
    Only instantiated when DEBUG_OVERLAY = True.

    The overlay uses a native tk.Frame rather than CTkFrame because
    tk's wm_attributes('-alpha') and flat relief give a cleaner look
    for an informational HUD element.
    """
    def __init__(self, root: ctk.CTk, app_state: AppState):
        self.app_state = app_state

        self.frame = tk.Frame(root, bg="#0d0d0d", bd=1, relief="solid")
        self.frame.place(relx=1.0, rely=1.0, x=-12, y=-12,
                         anchor="se", width=370, height=165)

        tk.Label(self.frame, text="⬛ DEBUG  TX LOG",
                 bg="#0d0d0d", fg=C_TERTIARY,
                 font=("Courier", 10, "bold")).pack(anchor="w", padx=8, pady=(5, 0))

        self._log_var = tk.StringVar(value="(no packets sent yet)")
        tk.Label(self.frame, textvariable=self._log_var,
                 bg="#0d0d0d", fg="#00ff88",
                 font=("Courier", 10), justify="left",
                 anchor="nw").pack(fill="both", expand=True, padx=8, pady=4)

        self._poll()

    def _poll(self):
        with self.app_state.lock:
            lines = list(self.app_state.debug_log)
        self._log_var.set("\n".join(lines) if lines else "(no packets sent yet)")
        self.frame.after(500, self._poll)

    def lift(self):
        """Re-raise above any newly packed content frames."""
        self.frame.lift()


# ============================================================
# BASE SCREEN CLASS
# All screens inherit from this. Provides self.app, self.state,
# and the self.show() shortcut for frame navigation.
# ============================================================

class BaseScreen(ctk.CTkFrame):
    """
    Parent class for every screen. Subclasses call super().__init__()
    and then build their own widgets on top of this frame.
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, fg_color=C_BG, corner_radius=0)
        self.app = app
        self.state = app_state

    def show(self, screen_class: type, **kwargs):
        """Navigate to another screen. kwargs are forwarded to its __init__."""
        self.app.show_frame(screen_class, **kwargs)


# ============================================================
# SCREEN 1: STARTUP
# ============================================================

class StartupScreen(BaseScreen):
    """
    First screen shown on launch. Displays the Bullseye logo.

    STARTUP = True  (debug): shows a START button that bypasses the Pi connection
                             check and navigates immediately to MainMenuScreen.
    STARTUP = False (prod):  replaces the START button with a "CONNECTING..."
                             label; sends a ping packet every PING_INTERVAL_MS ms
                             until a ping_ack arrives, then auto-navigates.
                             The screen is locked — the user cannot proceed until
                             the Pi responds.

    TODO - JAY - Maybe get the font to match the bullseye screen - later
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        self._ping_id: str | None = None   # after() id for ping loop
        self._poll_id: str | None = None   # after() id for event poll
        self.bind("<Destroy>", self._on_destroy)

        # Center everything vertically and horizontally
        center = ctk.CTkFrame(self, fg_color="transparent")
        center.place(relx=0.5, rely=0.5, anchor="center")

        # ── Logo ──────────────────────────────────────────────────────────
        self.logo_image = ctk.CTkImage(
            Image.open("assets/logo.png"),
            size=(800, 500)
        )
        ctk.CTkLabel(center, image=self.logo_image, text="").pack()

        #ctk.CTkLabel(
        #   center, text="BULLSEYE",
        #   font=("Arial Black", 90, "bold"),
        #    text_color=C_PRIMARY,
        #).pack(pady=(0, 6))

        #ctk.CTkLabel(
        #    center, text="AUTONOMOUS Roping Dummy",
        #    font=("Arial", 22),
        #    text_color=C_MUTED,
        #).pack(pady=(0, 70))

        # ── START button (debug) or CONNECTING label (prod) ──────────────
        if STARTUP:
            # Debug bypass — go straight to main menu without pinging
            make_nav_button(
                center, "START",
                command=lambda: self.show(MainMenuScreen),
                width=600, height=100,
            ).pack(pady=(0, 100))
        else:
            # Production — show connecting state and start ping loop
            self._status_label = ctk.CTkLabel(
                center,
                text="CONNECTING TO BULLSEYE...",
                font=("Arial Black", 32),
                text_color=C_MUTED,
            )
            self._status_label.pack(pady=(0, 20))

            self._exit_btn = ctk.CTkButton(
                center,
                text="EXIT",
                width=200, height=60,
                font=("Arial Black", 24),
                fg_color=C_DANGER,
                hover_color="#991a00",
                command=self.app.quit,
            )
            self._exit_btn_id: str | None = None

            self._dot_count = 0
            self._start_ping_loop()
            self._exit_btn_id = self.after(5000, self._show_exit_button)

    def _start_ping_loop(self):
        """Begin sending ping packets every PING_INTERVAL_MS ms."""
        self._send_ping()
        self._poll_events()

    def _send_ping(self):
        """Enqueue one ping packet then reschedule."""
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return
        enqueue_packet(self.state, "ping")
        # Animate dots so the user knows it's alive
        self._dot_count = (self._dot_count + 1) % 4
        dots = "." * self._dot_count
        try:
            self._status_label.configure(
                text=f"CONNECTING TO BULLSEYE{dots}"
            )
        except Exception:
            pass
        try:
            self._ping_id = self.after(PING_INTERVAL_MS, self._send_ping)
        except Exception:
            pass

    def _poll_events(self):
        """Check event_queue every 200 ms for ping_ack from the Pi."""
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return
        try:
            unmatched = []
            found = None
            while not self.state.event_queue.empty():
                try:
                    event = self.state.event_queue.get_nowait()
                except Exception:
                    break
                if event.get("type") == "ping_ack":
                    found = event
                    break
                else:
                    unmatched.append(event)
            for e in unmatched:
                self.state.event_queue.put(e)

            if found is not None:
                self._cancel_timers()
                self.show(MainMenuScreen)
                return
        except Exception:
            pass
        try:
            self._poll_id = self.after(200, self._poll_events)
        except Exception:
            pass

    def _show_exit_button(self):
        try:
            if self.winfo_exists():
                self._exit_btn.pack(pady=(20, 80))
        except Exception:
            pass

    def _cancel_timers(self):
        for attr in ("_ping_id", "_poll_id", "_exit_btn_id"):
            id_ = getattr(self, attr, None)
            if id_ is not None:
                try:
                    self.after_cancel(id_)
                except Exception:
                    pass
                setattr(self, attr, None)

    def _on_destroy(self, *_):
        self._cancel_timers()


# ============================================================
# SCREEN 2: MAIN MENU
# ============================================================

class MainMenuScreen(BaseScreen):
    """
    Central navigation hub. No robot state changes are sent from here.
    Three large buttons route to Drive, Path, and Settings sections.

    If app_state.e_stop_active is True on entry a gold banner is shown at
    the top of the screen. The banner clears when DRIVE or PATH MENU is pressed.
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        # ── E-Stop banner (shown only when e_stop_active) ─────────────────
        with self.state.lock:
            e_stopped = self.state.e_stop_active

        if e_stopped:
            banner = ctk.CTkFrame(self, fg_color=C_TERTIARY, corner_radius=0, height=52)
            banner.pack(fill="x", side="top")
            banner.pack_propagate(False)
            ctk.CTkLabel(
                banner,
                text="E-STOP ACTIVATED — Select DRIVE or PATH MENU to continue",
                font=("Arial Bold", 18),
                text_color=C_BG,
            ).place(relx=0.5, rely=0.5, anchor="center")

        center = ctk.CTkFrame(self, fg_color="transparent")
        center.place(relx=0.5, rely=0.5, anchor="center")

        ctk.CTkLabel(center, text="MAIN MENU",
                     font=("Arial Bold", 30),
                     text_color=C_TEXT).pack(pady=(0, 50))

        make_nav_button(center, "DRIVE",
                        command=self._go_drive).pack(pady=14)

        make_nav_button(center, "PATH MENU",
                        command=self._go_path_menu).pack(pady=14)

        make_nav_button(center, "SETTINGS",
                        command=lambda: self.show(SettingsSubMenuScreen)).pack(pady=14)

        # ── EXIT button – bottom-right corner ─────────────────────────────
        ctk.CTkButton(
            self, text="EXIT",
            command=self.app._on_close,
            font=("Arial", 28, "bold"),
            fg_color=C_DANGER, hover_color="#991a00",
            text_color=C_TEXT, corner_radius=16,
            width=160, height=60,
        ).place(relx=1.0, rely=1.0, x=-20, y=-20, anchor="se")

    def _go_drive(self):
        with self.state.lock:
            self.state.e_stop_active = False
        self.show(FreeDriveScreen)

    def _go_path_menu(self):
        with self.state.lock:
            self.state.e_stop_active = False
        self.show(PathSubMenuScreen)


# ============================================================
# SCREEN 3: FREE DRIVE
# ============================================================

class FreeDriveScreen(BaseScreen):
    """
    Puts the robot in TELEOP state and streams joystick input until
    the user leaves the screen.

    On enter     → sends StateData(TELEOP) + sets joystick_active=True
    BACK         → sends StateData(DISABLED) + joystick_active=False → Main Menu
    SET HOME     → confirm dialog → pause joystick → request_pos → set_home
                   → home_ack → save home.json → resume joystick
                   On any ack timeout → show error label, re-enable button

    SET HOME flow detail:
      1. User presses SET HOME → confirm overlay appears
      2. On confirm → joystick paused, button disabled, send request_pos
      3. Poll for pos_data (ACK_TIMEOUT_MS)
      4. On pos_data → send set_home with those coordinates
      5. Poll for home_ack (ACK_TIMEOUT_MS)
      6. On home_ack → save home.json, update app_state.home_pos,
                        re-enable joystick + button, show brief success label
      7. On any timeout → show error label, restore joystick + button
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        self._poll_id: str | None = None
        self._set_home_stage: str = "idle"   # idle | confirm | req_pos | set_home
        self._pending_pos: dict | None = None
        self.bind("<Destroy>", self._on_destroy)

        # ── Center instruction text ───────────────────────────────────────
        center = ctk.CTkFrame(self, fg_color="transparent")
        center.place(relx=0.5, rely=0.5, anchor="center")

        ctk.CTkLabel(
            center,
            text="USE THE JOYSTICKS\nTO CONTROL BULLSEYE",
            font=("Arial Black", 42),
            text_color=C_TERTIARY,
            justify="center",
        ).pack(pady=40)

        # Status label — shown during set-home flow and on error/success
        self._status_lbl = ctk.CTkLabel(
            center, text="",
            font=("Arial", 22),
            text_color=C_MUTED,
        )
        self._status_lbl.pack(pady=(0, 10))

        # ── Confirm overlay (hidden until SET HOME pressed) ───────────────
        self._confirm_frame = ctk.CTkFrame(self, fg_color=C_SURFACE,
                                            corner_radius=16, width=600, height=260)
        ctk.CTkLabel(self._confirm_frame,
                     text="Set current location as home position?",
                     font=("Arial Bold", 26), text_color=C_TEXT).pack(pady=(30, 8))
        ctk.CTkLabel(self._confirm_frame,
                     text="Joystick will pause while home is being set.",
                     font=("Arial", 18), text_color=C_MUTED).pack(pady=(0, 24))
        confirm_btns = ctk.CTkFrame(self._confirm_frame, fg_color="transparent")
        confirm_btns.pack(pady=(0, 30))
        make_nav_button(confirm_btns, "CANCEL",
                        command=self._cancel_set_home,
                        color=C_DANGER, width=180, height=70).pack(side="left", padx=20)
        make_nav_button(confirm_btns, "CONFIRM",
                        command=self._confirmed_set_home,
                        color=C_SUCCESS, width=180, height=70).pack(side="right", padx=20)

        # ── Home error overlay (hidden until a set-home attempt times out) ─
        self._home_error_frame = ctk.CTkFrame(self, fg_color=C_SURFACE,
                                               corner_radius=16, width=680, height=300)
        ctk.CTkLabel(self._home_error_frame,
                     text="No response from Bullseye",
                     font=("Arial Bold", 28), text_color=C_DANGER).pack(pady=(34, 6))
        ctk.CTkLabel(self._home_error_frame,
                     text="Home position in error state\nSet home position again before running.",
                     font=("Arial", 20), text_color=C_TEXT, justify="center").pack(pady=(0, 24))
        make_nav_button(self._home_error_frame, "OK",
                        command=self._dismiss_home_error,
                        color=C_SECONDARY, width=180, height=60).pack(pady=(0, 30))

        # ── Bottom button bar ─────────────────────────────────────────────
        btn_bar = ctk.CTkFrame(self, fg_color="transparent")
        btn_bar.pack(side="bottom", pady=35)

        make_nav_button(btn_bar, "BACK",
                        command=self._back,
                        color=C_DANGER, width=220).pack(side="left", padx=40)

        self._set_home_btn = make_nav_button(btn_bar, "SET HOME",
                                              command=self._press_set_home,
                                              color=C_SUCCESS, width=220)
        self._set_home_btn.pack(side="right", padx=40)

        # ── Activate TELEOP on screen construction ────────────────────────
        self._enter()

    def _enter(self):
        """Send TELEOP state and open joystick stream."""
        enqueue_state(self.state, State.TELEOP)
        with self.state.lock:
            self.state.joystick_active = True

    def _back(self):
        """Disable robot, stop joystick stream, return to Main Menu."""
        self._cancel_poll()
        enqueue_state(self.state, State.DISABLED)
        with self.state.lock:
            self.state.joystick_active = False
        self.show(MainMenuScreen)

    # ── SET HOME flow ──────────────────────────────────────────────────────

    def _press_set_home(self):
        """Step 1: show confirm overlay."""
        if self._set_home_stage != "idle":
            return
        self._set_home_stage = "confirm"
        self._confirm_frame.place(relx=0.5, rely=0.5, anchor="center")
        self._confirm_frame.lift()

    def _cancel_set_home(self):
        """User cancelled the confirm dialog — restore idle state."""
        self._set_home_stage = "idle"
        self._confirm_frame.place_forget()

    def _confirmed_set_home(self):
        """Step 2: hide confirm, pause joystick, send request_pos."""
        self._confirm_frame.place_forget()
        self._set_home_stage = "req_pos"
        self._set_home_btn.configure(state="disabled", text="SETTING...")
        self._status_lbl.configure(text="Fetching position...", text_color=C_MUTED)
        with self.state.lock:
            self.state.joystick_active = False

        # Drop stale position samples so the next pos_data is from this request.
        unmatched = []
        while not self.state.event_queue.empty():
            try:
                event = self.state.event_queue.get_nowait()
            except Exception:
                break
            if event.get("type") != "pos_data":
                unmatched.append(event)
        for e in unmatched:
            self.state.event_queue.put(e)

        enqueue_packet(self.state, "request_pos")
        self._timeout_remaining = ACK_TIMEOUT_MS // 200
        self._poll_id = self.after(200, self._poll_set_home)

    def _poll_set_home(self):
        """Poll event_queue for pos_data then home_ack."""
        self._poll_id = None
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return

        try:
            # Drain the full queue so a stale unrelated event can't block the
            # target event.  Unmatched events are collected and put back.
            unmatched = []
            found = None
            while not self.state.event_queue.empty():
                try:
                    event = self.state.event_queue.get_nowait()
                except Exception:
                    break
                etype = event.get("type")
                if self._set_home_stage == "req_pos" and etype == "pos_data":
                    # Keep the newest pos_data if multiple are queued.
                    found = event
                elif self._set_home_stage == "set_home" and etype == "home_ack":
                    found = event
                    break
                else:
                    unmatched.append(event)
            for e in unmatched:
                self.state.event_queue.put(e)

            if found is not None:
                etype = found.get("type")
                if etype == "pos_data":
                    # Step 3: got position — now send set_home
                    self._pending_pos = {"x": found["x"], "y": found["y"],
                                         "yaw": found["yaw"]}
                    self._set_home_stage = "set_home"
                    self._status_lbl.configure(text="Updating home position...")
                    payload = json.dumps(self._pending_pos)
                    enqueue_packet(self.state, "set_home", payload)
                    self._timeout_remaining = ACK_TIMEOUT_MS // 200
                    self._poll_id = self.after(200, self._poll_set_home)
                    return
                elif etype == "home_ack":
                    # Step 4: Pi confirmed — save locally and restore UI
                    with self.state.lock:
                        self.state.home_pos = dict(self._pending_pos)
                    save_home(self._pending_pos)
                    clear_kfx_for_home_mismatch(self.state, self._pending_pos)
                    self._finish_set_home(success=True)
                    return
        except Exception:
            pass

        # Timeout countdown
        self._timeout_remaining -= 1
        if self._timeout_remaining <= 0:
            self._finish_set_home(success=False)
            return

        try:
            self._poll_id = self.after(200, self._poll_set_home)
        except Exception:
            pass

    def _finish_set_home(self, success: bool):
        """Restore UI after set-home flow succeeds or times out."""
        self._set_home_stage = "idle"
        self._pending_pos = None
        with self.state.lock:
            self.state.joystick_active = True
        self._set_home_btn.configure(state="normal", text="SET HOME")
        if success:
            self._status_lbl.configure(text="Home position set!", text_color=C_SUCCESS)
            self.after(3000, lambda: self._status_lbl.configure(text=""))
        else:
            self._home_error_frame.place(relx=0.5, rely=0.5, anchor="center")
            self._home_error_frame.lift()

    def _dismiss_home_error(self):
        """Dismiss the home-error modal."""
        self._home_error_frame.place_forget()

    def _cancel_poll(self):
        if self._poll_id is not None:
            try:
                self.after_cancel(self._poll_id)
            except Exception:
                pass
            self._poll_id = None

    def _on_destroy(self, *_):
        self._cancel_poll()


# ============================================================
# SCREEN 4: PATH SUB-MENU
# ============================================================

class PathSubMenuScreen(BaseScreen):
    """
    Intermediate menu between Main Menu and path-related screens.
    No robot state changes from here.
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        center = ctk.CTkFrame(self, fg_color="transparent")
        center.place(relx=0.5, rely=0.5, anchor="center")

        ctk.CTkLabel(center, text="PATH MENU",
                     font=("Arial Bold", 30),
                     text_color=C_TEXT).pack(pady=(0, 50))

        make_nav_button(center, "RECORD ROUTE",
                        command=lambda: self.show(RecordRouteScreen)).pack(pady=14)

        make_nav_button(center, "RUN ROUTE",
                        command=lambda: self.show(RunRouteScreen)).pack(pady=14)

        make_nav_button(center, "← BACK",
                        command=lambda: self.show(MainMenuScreen),
                        color=C_PRIMARY, width=220, height=65).pack(pady=(50, 0))


# ============================================================
# SCREEN 5: RECORD ROUTE
# ============================================================

class RecordRouteScreen(BaseScreen):
    """
    The user physically drives the robot while the Pi records a path.

    Entry flow (home-check gate):
      1. Screen opens → sends 'record_home_check', shows CHECKING state,
         disables buttons, does NOT start recording yet.
      2a. ok=True  → transition to RECORDING state; send record_start,
                     activate joystick.
      2b. ok=False → show error panel (home position from home.json) with
                     BACK TO MAIN MENU button; recording never starts.
      2c. Timeout  → same error panel as 2b with a timeout message.

    Recording flow (after gate passes):
      FINISH ROUTE → StateData(DISABLED); Pi saves CSV, sends 'path_created';
                     GUI waits then navigates to NameRouteScreen.
      CANCEL ROUTE → StateData(DISABLED) → PathSubMenu
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        # ── Internal state ────────────────────────────────────────────────
        # Stage: "checking" | "recording" | "saving" | "failed"
        self._stage: str = "checking"
        self._poll_id: str | None = None
        self._finish_timeout_id: str | None = None
        self._pos_poll_id: str | None = None
        self._pos_timeout_remaining: int = 0
        self._check_timeout_remaining: int = ACK_TIMEOUT_MS // 200
        self.bind("<Destroy>", self._on_destroy)

        # ── Center content ────────────────────────────────────────────────
        self._center = ctk.CTkFrame(self, fg_color="transparent")
        self._center.place(relx=0.5, rely=0.5, anchor="center")

        self._status_label = ctk.CTkLabel(
            self._center,
            text="CHECKING HOME POSITION...",
            font=("Arial Black", 36),
            text_color=C_MUTED,
            justify="center",
        )
        self._status_label.pack(pady=30)

        # Error detail label — shown only when check fails
        self._error_detail_lbl = ctk.CTkLabel(
            self._center, text="",
            font=("Arial", 22), text_color=C_MUTED, justify="center",
        )
        # Current position label — shown below error detail when pos fetch succeeds
        self._cur_pos_lbl = ctk.CTkLabel(
            self._center, text="",
            font=("Arial", 20), text_color=C_MUTED, justify="center",
        )

        # ── Bottom button bar ─────────────────────────────────────────────
        btn_bar = ctk.CTkFrame(self, fg_color="transparent")
        btn_bar.pack(side="bottom", pady=35)

        self._cancel_btn = make_nav_button(
            btn_bar, "CANCEL ROUTE",
            command=self._cancel,
            color=C_DANGER, width=240,
        )
        self._cancel_btn.pack(side="left", padx=40)
        self._cancel_btn.configure(state="disabled")

        self._finish_btn = make_nav_button(
            btn_bar, "FINISH ROUTE",
            command=self._finish,
            color=C_SECONDARY, width=240,
        )
        self._finish_btn.pack(side="right", padx=40)
        self._finish_btn.configure(state="disabled")

        # Back-to-menu button shown only on check failure
        self._back_btn = make_nav_button(
            btn_bar, "BACK TO MAIN MENU",
            command=self._back_to_menu,
            color=C_PRIMARY, width=300,
        )
        # Not packed yet — only shown on failure

        # ── Kick off home check ───────────────────────────────────────────
        enqueue_packet(self.state, "record_home_check")
        self._poll_id = self.after(200, self._poll_check)

    # ── Home check polling ─────────────────────────────────────────────────

    def _poll_check(self):
        """Poll for record_home_check_result during the checking stage."""
        self._poll_id = None
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return

        try:
            unmatched = []
            found = None
            while not self.state.event_queue.empty():
                try:
                    event = self.state.event_queue.get_nowait()
                except Exception:
                    break
                if event.get("type") == "record_home_check_result":
                    found = event
                    break
                else:
                    unmatched.append(event)
            for e in unmatched:
                self.state.event_queue.put(e)

            if found is not None:
                if found.get("ok"):
                    self._start_recording()
                else:
                    self._show_check_failure(timed_out=False)
                return
        except Exception:
            pass

        self._check_timeout_remaining -= 1
        if self._check_timeout_remaining <= 0:
            self._show_check_failure(timed_out=True)
            return

        try:
            self._poll_id = self.after(200, self._poll_check)
        except Exception:
            pass

    def _start_recording(self):
        """Home check passed — transition to RECORDING state."""
        self._stage = "recording"
        self._status_label.configure(
            text="● RECORDING\nDRIVE THE ROUTE YOU WANT TO SAVE",
            text_color=C_DANGER,
        )
        self._cancel_btn.configure(state="normal")
        self._finish_btn.configure(state="normal")
        enqueue_state(self.state, State.RECORD_PATH)
        enqueue_packet(self.state, "record_start")
        with self.state.lock:
            self.state.joystick_active = True

    def _show_check_failure(self, timed_out: bool):
        """Home check failed or timed out — show error and back button."""
        self._stage = "failed"
        self._status_label.configure(
            text="NOT AT HOME POSITION",
            text_color=C_DANGER,
        )

        # Build a helpful detail message using the locally stored home position
        with self.state.lock:
            home = self.state.home_pos

        if timed_out:
            detail = "No response from Bullseye.\nCheck connection and try again."
        elif home:
            detail = (
                f"Home:     X: {home['x']:.2f}  Y: {home['y']:.2f}  "
                f"Yaw: {home['yaw']:.1f}°\n"
                "Return to that position before recording."
            )
        else:
            detail = (
                "No home position is set.\n"
                "Go to Settings → Bot Settings → Home Settings to set one."
            )

        self._error_detail_lbl.configure(text=detail)
        self._error_detail_lbl.pack(pady=(0, 6))

        # Fetch and display current position (non-timeout failures only)
        if not timed_out and home:
            self._cur_pos_lbl.configure(text="Current:  fetching...", text_color=C_MUTED)
            self._cur_pos_lbl.pack(pady=(0, 14))
            # Drop stale position samples so the next pos_data is from this request.
            unmatched = []
            while not self.state.event_queue.empty():
                try:
                    event = self.state.event_queue.get_nowait()
                except Exception:
                    break
                if event.get("type") != "pos_data":
                    unmatched.append(event)
            for e in unmatched:
                self.state.event_queue.put(e)
            enqueue_packet(self.state, "request_pos")
            self._pos_timeout_remaining = 10  # 10 × 200 ms = 2 s
            self._pos_poll_id = self.after(200, self._poll_cur_pos)
        else:
            self._cur_pos_lbl.pack_forget()

        # Swap finish/cancel for a single back-to-menu button
        self._cancel_btn.pack_forget()
        self._finish_btn.pack_forget()
        self._back_btn.pack(pady=10)

    def _poll_cur_pos(self):
        """Poll for pos_data after a failed home check to display current position."""
        self._pos_poll_id = None
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return

        try:
            unmatched = []
            found = None
            while not self.state.event_queue.empty():
                try:
                    event = self.state.event_queue.get_nowait()
                except Exception:
                    break
                if event.get("type") == "pos_data":
                    found = event
                    break
                else:
                    unmatched.append(event)
            for e in unmatched:
                self.state.event_queue.put(e)

            if found is not None:
                x, y, yaw = found["x"], found["y"], found["yaw"]
                self._cur_pos_lbl.configure(
                    text=f"Current:  X: {x:.2f}  Y: {y:.2f}  Yaw: {yaw:.1f}°"
                )
                return
        except Exception:
            pass

        self._pos_timeout_remaining -= 1
        if self._pos_timeout_remaining <= 0:
            self._cur_pos_lbl.pack_forget()
            return
        try:
            self._pos_poll_id = self.after(200, self._poll_cur_pos)
        except Exception:
            pass

    def _back_to_menu(self):
        """Navigate back to the main menu after a check failure."""
        self._cancel_all_timers()
        self.show(MainMenuScreen)

    # ── Recording flow ─────────────────────────────────────────────────────

    def _finish(self):
        """Send finish command and wait for Pi to confirm the saved route ID."""
        if self._stage != "recording":
            return
        self._stage = "saving"
        self._cancel_all_timers()
        enqueue_state(self.state, State.DISABLED)
        with self.state.lock:
            self.state.joystick_active = False

        enqueue_packet(self.state, "record_finish")
        self._status_label.configure(
            text="SAVING ROUTE...\nWaiting for Pi confirmation",
            text_color=C_TERTIARY,
        )
        self._finish_btn.configure(state="disabled")
        self._cancel_btn.configure(state="disabled")
        self._finish_timeout_id = self.after(
            int(RECORD_FINISH_TIMEOUT * 1000),
            self._on_finish_timeout,
        )
        self._poll_id = self.after(200, self._poll_events)

    def _cancel(self):
        """Discard the in-progress route and return to the path menu."""
        self._cancel_all_timers()
        enqueue_packet(self.state, "record_cancel")
        enqueue_state(self.state, State.DISABLED)
        with self.state.lock:
            self.state.joystick_active = False
        self.show(PathSubMenuScreen)

    def _poll_events(self):
        """Poll for path_created confirmation during saving stage."""
        self._poll_id = None
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return
        try:
            unmatched = []
            found = None
            while not self.state.event_queue.empty():
                try:
                    event = self.state.event_queue.get_nowait()
                except Exception:
                    break
                if event.get("type") == "path_created":
                    found = event
                    break
                else:
                    unmatched.append(event)
            for e in unmatched:
                self.state.event_queue.put(e)

            if found is not None:
                self._cancel_all_timers()
                with self.state.lock:
                    self.state.joystick_active = False
                route_id = found.get("route_id")
                self.show(NameRouteScreen, route_id=route_id)
                return
        except Exception:
            pass
        try:
            self._poll_id = self.after(200, self._poll_events)
        except Exception:
            pass

    def _on_finish_timeout(self):
        self._finish_timeout_id = None
        if self._stage != "saving":
            return
        self._stage = "recording"
        self._status_label.configure(
            text="SAVE TIMED OUT\nCheck connection and try again",
            text_color=C_DANGER,
        )
        self._finish_btn.configure(state="normal")
        self._cancel_btn.configure(state="normal")

    # ── Cleanup ────────────────────────────────────────────────────────────

    def _cancel_all_timers(self):
        for attr in ("_poll_id", "_finish_timeout_id", "_pos_poll_id"):
            id_ = getattr(self, attr, None)
            if id_ is not None:
                try:
                    self.after_cancel(id_)
                except Exception:
                    pass
                setattr(self, attr, None)

    def _on_destroy(self, *_):
        self._cancel_all_timers()


# ============================================================
# SCREEN 6: NAME ROUTE
# ============================================================

class NameRouteScreen(BaseScreen):
    """
    Prompts the user to give a human-readable name to a newly recorded route.
    Shown automatically after RecordRouteScreen receives 'path_created' from Pi.

    SAVE   → writes route_id → name into routes.json, navigates to PathSubMenu
    CANCEL → discards the name entry (route still exists on Pi by its numeric ID)
    ⌨ btn  → launches the Steam Deck on-screen keyboard (Desktop Mode via Steam)
    """
    def __init__(self, parent, app, app_state: AppState,
                 route_id: int = None,
                 initial_name: str = "",
                 heading: str = "NAME YOUR ROUTE",
                 return_screen=None):
        super().__init__(parent, app, app_state)
        self._route_id = route_id
        self._return_screen = return_screen  # None → PathSubMenuScreen

        center = ctk.CTkFrame(self, fg_color="transparent")
        center.place(relx=0.5, rely=0.5, anchor="center")

        ctk.CTkLabel(center, text=heading,
                     font=("Arial Bold", 34),
                     text_color=C_TEXT).pack(pady=(0, 8))

        ctk.CTkLabel(center, text=f"Route ID: {route_id}",
                     font=("Arial", 18),
                     text_color=C_MUTED).pack(pady=(0, 36))

        # ── Text entry + keyboard trigger button ──────────────────────────
        entry_row = ctk.CTkFrame(center, fg_color="transparent")
        entry_row.pack()

        self._entry = ctk.CTkEntry(
            entry_row,
            placeholder_text="Enter route name...",
            font=("Arial", 22), width=420, height=58,
            fg_color=C_SURFACE, border_color=C_SECONDARY,
            text_color=C_TEXT,
        )
        self._entry.pack(side="left", padx=(0, 10))
        if initial_name:
            self._entry.insert(0, initial_name)

        # Keyboard button – opens the in-app on-screen keyboard
        ctk.CTkButton(
            entry_row, text="⌨", width=58, height=58,
            font=("Arial", 28), fg_color=C_PRIMARY,
            hover_color=C_SECONDARY,
            command=self._open_keyboard,
        ).pack(side="left")

        # Build the in-app keyboard (hidden until ⌨ is pressed)
        self._keyboard = OnScreenKeyboard(self, app_state, self._entry)

        # ── Action buttons ────────────────────────────────────────────────
        action_row = ctk.CTkFrame(center, fg_color="transparent")
        action_row.pack(pady=44)

        make_nav_button(action_row, "CANCEL",
                        command=lambda: self.show(self._return_screen or PathSubMenuScreen),
                        color=C_DANGER, width=190, height=70).pack(side="left", padx=20)

        make_nav_button(action_row, "SAVE",
                        command=self._save,
                        width=190, height=70).pack(side="right", padx=20)

    def _open_keyboard(self):
        """Show the in-app on-screen keyboard overlay."""
        self._keyboard.show()

    def _save(self):
        """Validate, persist, and navigate away."""
        name = self._entry.get().strip()
        if not name:
            # Flash border red for 1 second as a no-popup error indicator
            self._entry.configure(border_color=C_DANGER)
            self.after(1000, lambda: self._entry.configure(border_color=C_SECONDARY))
            return

        # Capture current home position to associate with this route
        with self.state.lock:
            home_snapshot = dict(self.state.home_pos) if self.state.home_pos else None

        with self.state.lock:
            existing = self.state.routes.get(str(self._route_id), {})
            # Preserve existing home if it was already set; only override if we
            # are saving a brand-new route (home_snapshot is the current global home)
            if isinstance(existing, dict) and existing.get("home") is not None:
                home_to_save = existing["home"]
            else:
                home_to_save = home_snapshot

            self.state.routes[str(self._route_id)] = {
                "name": name,
                "home": home_to_save,
            }
            routes_snapshot = dict(self.state.routes)
        save_routes(routes_snapshot)
        self.show(self._return_screen or PathSubMenuScreen)


# ============================================================
# SCREEN 7: RUN ROUTE
# ============================================================

class RunRouteScreen(BaseScreen):
    """
    Displays saved routes for selection, a speed slider, and run controls.

    Flow:
      1. Select route → press RUN
      2. Home check gate: sends record_home_check, polls for result
         ok=True  → show run-confirm overlay (route name + speed, START / CANCEL)
         ok=False → show inline error with home coords, re-enable RUN
         timeout  → same error
      3. START confirmed → AUTONOMOUS state sent; RUN/HOME hidden, only STOP shown
      4. STOP → DISABLED; restore RUN + HOME buttons

    RETURN HOME → confirm overlay → RETURN_TO_HOME state
    BACK        → DISABLED if running → PathSubMenu
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        self._selected_id: int | None = None
        self._route_buttons: dict = {}        # route_id (int) → CTkButton (main row)
        self._route_action_frames: dict = {}  # route_id (int) → CTkFrame (hidden sub-row)
        self._route_valid: dict = {}          # route_id (int) → bool (home matches current)
        self._expanded_id: int | None = None  # which row is currently expanded
        self._is_running: bool = False
        self._poll_id: str | None = None
        self._pos_poll_id: str | None = None  # separate poll for current-pos fetch
        self._pos_timeout_remaining: int = 0
        self._check_timeout_remaining: int = 0
        self.bind("<Destroy>", self._on_destroy)

        # ── Main two-column content ───────────────────────────────────────
        content = ctk.CTkFrame(self, fg_color="transparent")
        content.pack(fill="both", expand=True, padx=30, pady=20)

        # Left: scrollable route list.
        # Stored as self._list_frame so _run() can flash its border when
        # the user presses RUN without first selecting a route.
        self._list_frame = ctk.CTkFrame(content, fg_color=C_SURFACE, corner_radius=12)
        self._list_frame.pack(side="left", fill="both", expand=True, padx=(0, 24))
        list_frame = self._list_frame   # Local alias used below for readability

        ctk.CTkLabel(list_frame, text="SELECT ROUTE",
                     font=("Arial Bold", 18),
                     text_color=C_MUTED).pack(pady=(14, 6))

        self._route_scroll = ctk.CTkScrollableFrame(
            list_frame, fg_color="transparent"
        )
        self._route_scroll.pack(fill="both", expand=True, padx=10, pady=10)
        self._build_route_list()

        # Right: speed control and action buttons
        controls = ctk.CTkFrame(content, fg_color="transparent", width=280)
        controls.pack(side="right", fill="y")
        controls.pack_propagate(False)

        ctk.CTkLabel(controls, text="SPEED",
                     font=("Arial Bold", 20),
                     text_color=C_TEXT).pack(pady=(24, 4))

        self._speed_label = ctk.CTkLabel(controls, text="50%",
                                          font=("Arial Bold", 38),
                                          text_color=C_TERTIARY)
        self._speed_label.pack()

        self._speed_slider = ctk.CTkSlider(
            controls, from_=10, to=100, number_of_steps=9,
            width=230, button_color=C_TERTIARY,
            progress_color=C_SECONDARY,
            command=self._on_speed_change,
        )
        self._speed_slider.set(50)
        self._speed_slider.pack(pady=(4, 24))

        # Status label shown during home check
        self._run_status_lbl = ctk.CTkLabel(
            controls, text="", font=("Arial", 15),
            text_color=C_MUTED, wraplength=220, justify="center",
        )
        self._run_status_lbl.pack(pady=(0, 4))

        # Action buttons — RUN and HOME visible at rest; STOP shown while running
        self._run_btn = make_nav_button(
            controls, "RUN",
            command=self._press_run,
            width=230, height=72,
        )
        self._run_btn.pack(pady=8)

        self._home_btn = make_nav_button(
            controls, "HOME",
            command=self._press_return_home,
            color=C_SUCCESS, width=230, height=62,
        )
        self._home_btn.pack(pady=8)

        self._stop_btn = make_nav_button(
            controls, "STOP",
            command=self._stop,
            color=C_DANGER, width=230, height=72,
        )
        # Not packed yet — shown only while running

        make_nav_button(controls, "← BACK",
                        command=self._back,
                        color=C_PRIMARY, width=230, height=62).pack(pady=(20, 0))

        # ── Full-screen dim backdrop (shared by all three overlays) ──────────
        self._modal_backdrop = ctk.CTkFrame(self, fg_color="#0d0d0d", corner_radius=0)
        # Not placed yet — shown via _show_backdrop() whenever a modal opens

        # ── Home-check result overlay (shown after check fails) ───────────
        self._check_overlay = ctk.CTkFrame(
            self, fg_color=C_SURFACE, corner_radius=20,
            width=700, height=320,
            border_width=2, border_color=C_DANGER,
        )
        self._check_overlay_lbl = ctk.CTkLabel(
            self._check_overlay, text="",
            font=("Arial Bold", 24), text_color=C_DANGER,
            wraplength=640, justify="center",
        )
        self._check_overlay_lbl.pack(pady=(28, 8))
        self._check_cur_pos_lbl = ctk.CTkLabel(
            self._check_overlay, text="",
            font=("Arial", 16), text_color=C_MUTED,
            wraplength=640, justify="center",
        )
        self._check_cur_pos_lbl.pack(pady=(0, 2))
        self._check_detail_lbl = ctk.CTkLabel(
            self._check_overlay, text="",
            font=("Arial", 16), text_color=C_MUTED,
            wraplength=640, justify="center",
        )
        self._check_detail_lbl.pack(pady=(0, 10))
        ctk.CTkButton(
            self._check_overlay, text="OK",
            command=self._dismiss_check_overlay,
            font=("Arial Bold", 22), fg_color=C_PRIMARY,
            hover_color=C_SECONDARY, text_color=C_TEXT,
            corner_radius=12, width=160, height=55,
        ).pack(pady=(0, 24))

        # ── Run-confirm overlay (shown after home check passes) ───────────
        self._run_confirm_frame = ctk.CTkFrame(
            self, fg_color=C_SURFACE, corner_radius=20,
            width=580, height=260,
            border_width=2, border_color=C_SUCCESS,
        )
        self._run_confirm_lbl = ctk.CTkLabel(
            self._run_confirm_frame, text="",
            font=("Arial Bold", 22), text_color=C_TEXT,
            wraplength=540, justify="center",
        )
        self._run_confirm_lbl.pack(pady=(32, 4))
        self._run_confirm_detail = ctk.CTkLabel(
            self._run_confirm_frame, text="",
            font=("Arial", 16), text_color=C_MUTED,
        )
        self._run_confirm_detail.pack(pady=(0, 12))
        rcfm_btns = ctk.CTkFrame(self._run_confirm_frame, fg_color="transparent")
        rcfm_btns.pack(pady=(0, 28))
        ctk.CTkButton(
            rcfm_btns, text="CANCEL",
            command=self._cancel_run_confirm,
            font=("Arial Bold", 22), fg_color=C_DANGER,
            hover_color="#991a00", text_color=C_TEXT,
            corner_radius=12, width=160, height=60,
        ).pack(side="left", padx=16)
        ctk.CTkButton(
            rcfm_btns, text="START",
            command=self._confirmed_run,
            font=("Arial Bold", 22), fg_color=C_SUCCESS,
            hover_color=C_TERTIARY, text_color=C_TEXT,
            corner_radius=12, width=160, height=60,
        ).pack(side="right", padx=16)

        # ── HOME confirm overlay ──────────────────────────────────────────
        self._confirm_frame = ctk.CTkFrame(
            self, fg_color=C_SURFACE, corner_radius=20,
            width=540, height=220,
            border_width=2, border_color=C_SUCCESS,
        )
        ctk.CTkLabel(self._confirm_frame,
                     text="Send Bullseye back to its home position?",
                     font=("Arial Bold", 22), text_color=C_TEXT,
                     wraplength=500, justify="center").pack(pady=(32, 8))
        cfm_btns = ctk.CTkFrame(self._confirm_frame, fg_color="transparent")
        cfm_btns.pack(pady=(8, 28))
        ctk.CTkButton(
            cfm_btns, text="CANCEL",
            command=self._cancel_home_confirm,
            font=("Arial Bold", 22), fg_color=C_DANGER,
            hover_color="#991a00", text_color=C_TEXT,
            corner_radius=12, width=160, height=60,
        ).pack(side="left", padx=16)
        ctk.CTkButton(
            cfm_btns, text="CONFIRM",
            command=self._confirmed_return_home,
            font=("Arial Bold", 22), fg_color=C_SUCCESS,
            hover_color=C_TERTIARY, text_color=C_TEXT,
            corner_radius=12, width=160, height=60,
        ).pack(side="right", padx=16)

    def _build_route_list(self):
        """Populate the scrollable route list from app_state.routes.

        Valid routes (home matches current home position) are sorted to the top
        and shown normally.  Routes with a mismatched or missing home are grayed
        out and cannot be selected for running, but can still be renamed/deleted.
        """
        for w in self._route_scroll.winfo_children():
            w.destroy()
        self._route_buttons.clear()
        self._route_action_frames.clear()
        self._route_valid.clear()
        self._expanded_id = None
        self._selected_id = None

        with self.state.lock:
            routes = dict(self.state.routes)
            current_home = dict(self.state.home_pos) if self.state.home_pos else None

        if not routes:
            ctk.CTkLabel(self._route_scroll,
                         text="No saved routes.\nRecord a route first.",
                         font=("Arial", 16), text_color=C_MUTED,
                         justify="center").pack(pady=50)
            return

        # Determine validity and sort: valid routes first, invalid last.
        def _route_valid_check(item):
            entry = item[1]
            route_home = entry.get("home") if isinstance(entry, dict) else None
            return homes_match(route_home, current_home)

        sorted_routes = sorted(routes.items(), key=_route_valid_check, reverse=True)

        for route_id_str, entry in sorted_routes:
            rid = int(route_id_str)
            name = entry.get("name", "") if isinstance(entry, dict) else str(entry)
            home = entry.get("home") if isinstance(entry, dict) else None
            valid = homes_match(home, current_home)
            self._route_valid[rid] = valid

            # Home position sub-text
            if home:
                home_text = (f"Home: X {home['x']:.2f}  Y {home['y']:.2f}"
                             f"  Yaw {home['yaw']:.1f}°")
            else:
                home_text = "Home: not set"

            # Visual treatment based on validity
            if valid:
                btn_fg       = C_SURFACE
                btn_hover    = C_SECONDARY
                btn_text_clr = C_TEXT
            else:
                btn_fg       = C_PRIMARY
                btn_hover    = C_PRIMARY   # no hover effect — non-runnable
                btn_text_clr = C_MUTED

            # Outer container — holds row button + collapsible action frame
            outer = ctk.CTkFrame(self._route_scroll, fg_color="transparent")
            outer.pack(fill="x", pady=2)

            # Main row button — two-line display (name + home summary)
            btn = ctk.CTkButton(
                outer,
                text=f"  {name}   (ID {rid})\n  {home_text}",
                font=("Arial Bold", 16),
                fg_color=btn_fg, hover_color=btn_hover,
                text_color=btn_text_clr, corner_radius=8,
                height=72, anchor="w",
                command=lambda r=rid: self._tap_route(r),
            )
            btn.pack(fill="x")
            self._route_buttons[rid] = btn

            # Action sub-frame — hidden until the row is expanded
            action = ctk.CTkFrame(outer, fg_color=C_SURFACE, corner_radius=8)
            # Not packed yet; _tap_route will pack/forget it
            self._route_action_frames[rid] = action

        # Enable drag-to-scroll after all rows are built
        enable_touch_scroll(self._route_scroll)

    def _tap_route(self, route_id: int):
        """
        Tap a route row:
          - Selects it for running (gold highlight).
          - Expands its action sub-frame; collapses any previously open one.
          - Tapping the same row again collapses it (keeps it selected).
        """
        # Collapse previously expanded row (different from this one)
        if self._expanded_id is not None and self._expanded_id != route_id:
            old = self._expanded_id
            af = self._route_action_frames.get(old)
            if af:
                af.pack_forget()
            if old in self._route_buttons:
                if old == self._selected_id:
                    color = C_TERTIARY
                elif self._route_valid.get(old, True):
                    color = C_SURFACE
                else:
                    color = C_PRIMARY
                self._route_buttons[old].configure(fg_color=color)

        if self._expanded_id == route_id:
            # Same row tapped again → collapse
            af = self._route_action_frames.get(route_id)
            if af:
                af.pack_forget()
            self._expanded_id = None
            # Restore the base color for this row on collapse
            if route_id in self._route_buttons:
                if route_id == self._selected_id:
                    self._route_buttons[route_id].configure(fg_color=C_TERTIARY)
                elif self._route_valid.get(route_id, True):
                    self._route_buttons[route_id].configure(fg_color=C_SURFACE)
                else:
                    self._route_buttons[route_id].configure(fg_color=C_PRIMARY)
            return
        else:
            # Expand this row
            self._expanded_id = route_id
            af = self._route_action_frames.get(route_id)
            if af:
                self._populate_action_normal(route_id, af)
                af.pack(fill="x", padx=4, pady=(0, 4))

        # Select this route for running only if its home matches the current home.
        # Invalid routes still expand (for rename/delete) but cannot be queued to run.
        is_valid = self._route_valid.get(route_id, True)
        for rid, btn in self._route_buttons.items():
            if self._route_valid.get(rid, True):
                btn.configure(fg_color=C_SURFACE)
            else:
                btn.configure(fg_color=C_PRIMARY)
        if is_valid:
            self._selected_id = route_id
            if route_id in self._route_buttons:
                self._route_buttons[route_id].configure(fg_color=C_TERTIARY)
        else:
            # Don't change _selected_id — invalid route can't be run.
            # Give the expanded invalid row a slightly lighter shade so the user
            # can see it opened, but keep gold on any still-selected valid route.
            if route_id in self._route_buttons:
                self._route_buttons[route_id].configure(fg_color=C_SECONDARY)
            if self._selected_id is not None and self._selected_id in self._route_buttons:
                self._route_buttons[self._selected_id].configure(fg_color=C_TERTIARY)

    def _populate_action_normal(self, route_id: int, frame: ctk.CTkFrame):
        """Fill the action frame with [RENAME] [DELETE] buttons."""
        for w in frame.winfo_children():
            w.destroy()
        frame.configure(fg_color=C_SURFACE)

        with self.state.lock:
            entry = self.state.routes.get(str(route_id), {})
            name = entry.get("name", "") if isinstance(entry, dict) else str(entry)

        ctk.CTkButton(
            frame, text="RENAME",
            font=("Arial Bold", 14),
            fg_color=C_SECONDARY, hover_color=C_TERTIARY,
            text_color=C_TEXT, corner_radius=6,
            height=40, width=130,
            command=lambda: self._rename_route(route_id, name),
        ).pack(side="left", padx=(8, 4), pady=6)

        ctk.CTkButton(
            frame, text="DELETE",
            font=("Arial Bold", 14),
            fg_color=C_DANGER, hover_color="#991a00",
            text_color=C_TEXT, corner_radius=6,
            height=40, width=130,
            command=lambda: self._delete_prompt(route_id),
        ).pack(side="left", padx=(4, 8), pady=6)

    def _delete_prompt(self, route_id: int):
        """Switch the action frame to delete-confirmation mode."""
        af = self._route_action_frames.get(route_id)
        if af is None:
            return
        af.configure(fg_color="#4a0000")
        if route_id in self._route_buttons:
            self._route_buttons[route_id].configure(fg_color=C_DANGER)

        for w in af.winfo_children():
            w.destroy()

        ctk.CTkButton(
            af, text="CONFIRM DELETE",
            font=("Arial Bold", 14),
            fg_color=C_DANGER, hover_color="#991a00",
            text_color=C_TEXT, corner_radius=6,
            height=40, width=170,
            command=lambda: self._confirm_delete(route_id),
        ).pack(side="left", padx=(8, 4), pady=6)

        ctk.CTkButton(
            af, text="CANCEL",
            font=("Arial Bold", 14),
            fg_color=C_PRIMARY, hover_color=C_SECONDARY,
            text_color=C_TEXT, corner_radius=6,
            height=40, width=110,
            command=lambda: self._cancel_delete(route_id),
        ).pack(side="left", padx=(4, 8), pady=6)

    def _cancel_delete(self, route_id: int):
        """Restore action frame to normal RENAME/DELETE state."""
        af = self._route_action_frames.get(route_id)
        if af is None:
            return
        self._populate_action_normal(route_id, af)
        if route_id in self._route_buttons:
            self._route_buttons[route_id].configure(fg_color=C_TERTIARY)

    def _confirm_delete(self, route_id: int):
        """Remove route from storage, clear KFX refs, send updated config, rebuild."""
        with self.state.lock:
            routes_snapshot = dict(self.state.routes)
        routes_snapshot.pop(str(route_id), None)
        with self.state.lock:
            self.state.routes = routes_snapshot
        save_routes(routes_snapshot)

        # Clear any KFX button that was pointing to this route
        with self.state.lock:
            kfx = dict(self.state.kfx_config)
        changed = False
        for btn_num, assigned_id in kfx.items():
            if assigned_id == route_id:
                kfx[btn_num] = None
                changed = True
        if changed:
            with self.state.lock:
                self.state.kfx_config = kfx
            save_kfx_config(kfx)
            enqueue_packet(self.state, "kfx_config", json.dumps(kfx))

        self._build_route_list()

    def _rename_route(self, route_id: int, current_name: str):
        """Navigate to NameRouteScreen pre-filled for renaming."""
        self.show(NameRouteScreen,
                  route_id=route_id,
                  initial_name=current_name,
                  heading="RENAME ROUTE",
                  return_screen=RunRouteScreen)



    def _on_speed_change(self, value):
        """Update the speed label as the slider moves. Value is 10–100 (int steps)."""
        self._speed_label.configure(text=f"{int(value)}%")

    # ── RUN flow ──────────────────────────────────────────────────────────

    def _press_run(self):
        """Step 1: validate selection, send home check, start polling."""
        if self._selected_id is None:
            self._list_frame.configure(fg_color=C_DANGER)
            self.after(800, lambda: self._list_frame.configure(fg_color=C_SURFACE))
            return
        self._run_btn.configure(state="disabled")
        self._home_btn.configure(state="disabled")
        self._run_status_lbl.configure(text="Checking home position...",
                                        text_color=C_MUTED)
        enqueue_packet(self.state, "record_home_check")
        self._check_timeout_remaining = ACK_TIMEOUT_MS // 200
        self._poll_id = self.after(200, self._poll_home_check)

    def _poll_home_check(self):
        """Poll for record_home_check_result."""
        self._poll_id = None
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return

        try:
            unmatched = []
            found = None
            while not self.state.event_queue.empty():
                try:
                    event = self.state.event_queue.get_nowait()
                except Exception:
                    break
                if event.get("type") == "record_home_check_result":
                    found = event
                    break
                else:
                    unmatched.append(event)
            for e in unmatched:
                self.state.event_queue.put(e)

            if found is not None:
                self._run_status_lbl.configure(text="")
                if found.get("ok"):
                    self._show_run_confirm()
                else:
                    self._show_check_failed(timed_out=False)
                return
        except Exception:
            pass

        self._check_timeout_remaining -= 1
        if self._check_timeout_remaining <= 0:
            self._run_status_lbl.configure(text="")
            self._show_check_failed(timed_out=True)
            return

        try:
            self._poll_id = self.after(200, self._poll_home_check)
        except Exception:
            pass

    # ── Modal backdrop helpers ─────────────────────────────────────────────

    def _show_backdrop(self):
        self._modal_backdrop.place(x=0, y=0, relwidth=1, relheight=1)

    def _hide_backdrop(self):
        self._modal_backdrop.place_forget()

    # ── Check-failed overlay ───────────────────────────────────────────────

    def _show_check_failed(self, timed_out: bool):
        """Home check failed — show overlay with reason, re-enable RUN."""
        self._run_btn.configure(state="normal")
        self._home_btn.configure(state="normal")
        self._check_cur_pos_lbl.configure(text="")
        if timed_out:
            self._check_overlay_lbl.configure(text="NO RESPONSE FROM BULLSEYE")
            self._check_detail_lbl.configure(text="Check connection and try again.")
        else:
            with self.state.lock:
                home = self.state.home_pos
            if home:
                home_line = (f"Home:     X: {home['x']:.2f}  Y: {home['y']:.2f}"
                             f"  Yaw: {home['yaw']:.1f}°\n"
                             "Return to that position before running.")
            else:
                home_line = "No home position set.\nGo to Settings → Bot Settings → Home Settings."
            self._check_overlay_lbl.configure(text="NOT AT HOME POSITION")
            self._check_detail_lbl.configure(text=home_line)
            # Request current position from Pi so we can display it
            self._check_cur_pos_lbl.configure(text="Current:  fetching...", text_color=C_MUTED)
            # Drop stale position samples so the next pos_data is from this request.
            unmatched = []
            while not self.state.event_queue.empty():
                try:
                    event = self.state.event_queue.get_nowait()
                except Exception:
                    break
                if event.get("type") != "pos_data":
                    unmatched.append(event)
            for e in unmatched:
                self.state.event_queue.put(e)
            enqueue_packet(self.state, "request_pos")
            self._pos_timeout_remaining = 10  # 10 × 200 ms = 2 s
            self._pos_poll_id = self.after(200, self._poll_cur_pos)
        self._show_backdrop()
        self._check_overlay.place(relx=0.5, rely=0.5, anchor="center")
        self._check_overlay.lift()

    def _dismiss_check_overlay(self):
        # Cancel any in-flight position poll before closing
        if self._pos_poll_id is not None:
            try:
                self.after_cancel(self._pos_poll_id)
            except Exception:
                pass
            self._pos_poll_id = None
        self._check_overlay.place_forget()
        self._hide_backdrop()

    def _poll_cur_pos(self):
        """Poll event_queue for pos_data after a failed home check."""
        self._pos_poll_id = None
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return

        try:
            unmatched = []
            found = None
            while not self.state.event_queue.empty():
                try:
                    event = self.state.event_queue.get_nowait()
                except Exception:
                    break
                if event.get("type") == "pos_data":
                    found = event
                    break
                else:
                    unmatched.append(event)
            for e in unmatched:
                self.state.event_queue.put(e)

            if found is not None:
                x, y, yaw = found["x"], found["y"], found["yaw"]
                self._check_cur_pos_lbl.configure(
                    text=f"Current:  X: {x:.2f}  Y: {y:.2f}  Yaw: {yaw:.1f}°"
                )
                return
        except Exception:
            pass

        self._pos_timeout_remaining -= 1
        if self._pos_timeout_remaining <= 0:
            self._check_cur_pos_lbl.configure(text="")
            return
        try:
            self._pos_poll_id = self.after(200, self._poll_cur_pos)
        except Exception:
            pass

    # ── Run-confirm overlay ────────────────────────────────────────────────

    def _show_run_confirm(self):
        """Home check passed — show run-confirm overlay with route name + speed."""
        with self.state.lock:
            entry = self.state.routes.get(str(self._selected_id), {})
        name = entry.get("name", f"Route {self._selected_id}") if isinstance(entry, dict) else str(entry)
        speed_pct = int(self._speed_slider.get())
        self._run_confirm_lbl.configure(text=f"Run  \"{name}\"?")
        self._run_confirm_detail.configure(
            text=f"Speed: {speed_pct}%    Route ID: {self._selected_id}"
        )
        self._show_backdrop()
        self._run_confirm_frame.place(relx=0.5, rely=0.5, anchor="center")
        self._run_confirm_frame.lift()

    def _cancel_run_confirm(self):
        self._run_confirm_frame.place_forget()
        self._hide_backdrop()
        self._run_btn.configure(state="normal")
        self._home_btn.configure(state="normal")

    def _confirmed_run(self):
        """User confirmed — send AUTONOMOUS and switch to running UI."""
        self._run_confirm_frame.place_forget()
        self._hide_backdrop()
        speed = self._speed_slider.get() / 100.0
        enqueue_state(self.state, State.AUTONOMOUS,
                      path_id=self._selected_id, path_speed=speed)
        self._is_running = True
        self._run_btn.pack_forget()
        self._home_btn.pack_forget()
        self._stop_btn.pack(pady=8)

    def _stop(self):
        """Send DISABLED, restore RUN + HOME buttons."""
        enqueue_state(self.state, State.DISABLED)
        self._is_running = False
        self._stop_btn.pack_forget()
        self._run_btn.configure(state="normal")
        self._run_btn.pack(pady=8)
        self._home_btn.configure(state="normal")
        self._home_btn.pack(pady=8)

    def _on_destroy(self, *_):
        for attr in ("_poll_id", "_pos_poll_id"):
            id_ = getattr(self, attr, None)
            if id_ is not None:
                try:
                    self.after_cancel(id_)
                except Exception:
                    pass

    def _press_return_home(self):
        """Show confirm overlay before returning home."""
        self._show_backdrop()
        self._confirm_frame.place(relx=0.5, rely=0.5, anchor="center")
        self._confirm_frame.lift()

    def _cancel_home_confirm(self):
        self._confirm_frame.place_forget()
        self._hide_backdrop()

    def _confirmed_return_home(self):
        """Confirmed — send RETURN_TO_HOME state and switch to running UI."""
        self._confirm_frame.place_forget()
        self._hide_backdrop()
        enqueue_state(self.state, State.RETURN_TO_HOME)
        self._is_running = True
        self._run_btn.pack_forget()
        self._home_btn.pack_forget()
        self._stop_btn.pack(pady=8)

    def _back(self):
        """Stop the robot if running, then return to the path sub-menu."""
        if self._is_running:
            enqueue_state(self.state, State.DISABLED)
        self.show(PathSubMenuScreen)


# ============================================================
# SCREEN 8: SETTINGS SUB-MENU
# ============================================================

class SettingsSubMenuScreen(BaseScreen):
    """
    Settings navigation hub. No robot state changes from here.
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        center = ctk.CTkFrame(self, fg_color="transparent")
        center.place(relx=0.5, rely=0.5, anchor="center")

        ctk.CTkLabel(center, text="SETTINGS",
                     font=("Arial Bold", 30),
                     text_color=C_TEXT).pack(pady=(0, 50))

        make_nav_button(center, "CONTROLLER SETTINGS",
                        command=lambda: self.show(ControllerSettingsScreen)).pack(pady=14)

        make_nav_button(center, "BOT SETTINGS",
                        command=lambda: self.show(BotSettingsScreen)).pack(pady=14)

        make_nav_button(center, "KFX SETTINGS",
                        command=lambda: self.show(KFXSettingsScreen)).pack(pady=14)

        make_nav_button(center, "← BACK",
                        command=lambda: self.show(MainMenuScreen),
                        color=C_PRIMARY, width=220, height=65).pack(pady=(50, 0))


# ============================================================
# SCREEN 9: CONTROLLER SETTINGS
# ============================================================

class ControllerSettingsScreen(BaseScreen):
    """
    Controller and robot settings:

    Deadzone    – live global read by JoystickThread on every axis-read cycle
    Request Battery        – sends 'request_battery'; Pi replies with a 'battery'
                             packet; screen shows voltage, current, SOC, time remaining
    KFX Speed              – slider 10-100 %; SEND sends 'kfx_speed' packet;
                             Pi replies with 'kfx_speed_ack'; updates KFX_SPEED global
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        self._poll_id: str | None = None
        self._poll_stage: str = "idle"   # idle | battery | kfx_speed
        self._timeout_remaining: int = 0
        self.bind("<Destroy>", self._on_destroy)

        card = ctk.CTkFrame(self, fg_color=C_SURFACE, corner_radius=16,
                            width=720, height=600)
        card.place(relx=0.5, rely=0.5, anchor="center")
        card.pack_propagate(False)

        ctk.CTkLabel(card, text="CONTROLLER SETTINGS",
                     font=("Arial Bold", 28),
                     text_color=C_TEXT).pack(pady=(28, 16))

        # ── Deadzone slider ───────────────────────────────────────────────
        ctk.CTkLabel(card, text="Joystick Deadzone",
                     font=("Arial", 16), text_color=C_MUTED).pack()
        self._dz_label = ctk.CTkLabel(card, text=f"{DEADZONE:.2f}",
                                       font=("Arial Bold", 24), text_color=C_TERTIARY)
        self._dz_label.pack()
        dz_slider = ctk.CTkSlider(
            card, from_=0.0, to=0.5, number_of_steps=50,
            width=440, button_color=C_TERTIARY, progress_color=C_SECONDARY,
            command=self._dz_changed,
        )
        dz_slider.set(DEADZONE)
        dz_slider.pack(pady=(0, 16))

        # ── Divider ───────────────────────────────────────────────────────
        ctk.CTkFrame(card, fg_color=C_PRIMARY, height=2, width=560).pack(pady=(0, 16))

        # ── Two-column lower section ──────────────────────────────────────
        lower = ctk.CTkFrame(card, fg_color="transparent")
        lower.pack(fill="x", padx=32)

        # Left: Request Battery
        left = ctk.CTkFrame(lower, fg_color="transparent")
        left.pack(side="left", expand=True, fill="both", padx=(0, 16))

        ctk.CTkLabel(left, text="ROBOT BATTERY",
                     font=("Arial Bold", 17), text_color=C_TEXT).pack(pady=(0, 8))

        self._batt_display = ctk.CTkLabel(
            left, text="--",
            font=("Arial Bold", 15), text_color=C_MUTED,
            justify="left", wraplength=240,
        )
        self._batt_display.pack(pady=(0, 10))

        self._batt_btn = make_nav_button(
            left, "REQUEST",
            command=self._request_battery,
            color=C_SECONDARY, width=180, height=55,
        )
        self._batt_btn.pack()

        # Right: KFX Speed
        right = ctk.CTkFrame(lower, fg_color="transparent")
        right.pack(side="right", expand=True, fill="both", padx=(16, 0))

        ctk.CTkLabel(right, text="KFX RUN SPEED",
                     font=("Arial Bold", 17), text_color=C_TEXT).pack(pady=(0, 4))

        self._kfx_speed_label = ctk.CTkLabel(
            right, text=f"{int(KFX_SPEED * 100)}%",
            font=("Arial Bold", 30), text_color=C_TERTIARY,
        )
        self._kfx_speed_label.pack()

        self._kfx_speed_slider = ctk.CTkSlider(
            right, from_=10, to=100, number_of_steps=9,
            width=220, button_color=C_TERTIARY, progress_color=C_SECONDARY,
            command=self._kfx_speed_changed,
        )
        self._kfx_speed_slider.set(int(KFX_SPEED * 100))
        self._kfx_speed_slider.pack(pady=(0, 8))

        self._kfx_send_btn = make_nav_button(
            right, "SEND",
            command=self._send_kfx_speed,
            color=C_SECONDARY, width=180, height=55,
        )
        self._kfx_send_btn.pack()

        # Shared status label
        self._status_lbl = ctk.CTkLabel(
            card, text="",
            font=("Arial", 15), text_color=C_MUTED,
        )
        self._status_lbl.pack(pady=(14, 0))

        # ── Debug Overlay toggle ──────────────────────────────────────────
        ctk.CTkFrame(card, fg_color=C_PRIMARY, height=2, width=560).pack(pady=(14, 8))

        debug_row = ctk.CTkFrame(card, fg_color="transparent")
        debug_row.pack()

        ctk.CTkLabel(debug_row, text="Debug TX Overlay",
                     font=("Arial", 16), text_color=C_MUTED).pack(side="left", padx=(0, 16))

        self._debug_switch = ctk.CTkSwitch(
            debug_row, text="",
            command=self._toggle_debug_overlay,
            button_color=C_TERTIARY, progress_color=C_SECONDARY,
        )
        if DEBUG_OVERLAY:
            self._debug_switch.select()
        else:
            self._debug_switch.deselect()
        self._debug_switch.pack(side="left")

        make_nav_button(card, "← BACK",
                        command=lambda: self.show(SettingsSubMenuScreen),
                        color=C_PRIMARY, width=220, height=60).pack(pady=(8, 16))

    # ── Deadzone / rate ───────────────────────────────────────────────────

    def _dz_changed(self, value):
        global DEADZONE
        DEADZONE = round(float(value), 2)
        self._dz_label.configure(text=f"{DEADZONE:.2f}")

    # ── Debug overlay toggle ──────────────────────────────────────────────

    def _toggle_debug_overlay(self):
        self.app.toggle_debug_overlay(self._debug_switch.get() == 1)

    # ── Request Battery ───────────────────────────────────────────────────

    def _request_battery(self):
        if self._poll_stage != "idle":
            return
        self._poll_stage = "battery"
        self._batt_btn.configure(state="disabled", text="WAITING...")
        self._status_lbl.configure(text="Requesting battery data...", text_color=C_MUTED)
        enqueue_packet(self.state, "request_battery")
        self._timeout_remaining = ACK_TIMEOUT_MS // 200
        self._start_poll()

    # ── KFX Speed ─────────────────────────────────────────────────────────

    def _kfx_speed_changed(self, value):
        self._kfx_speed_label.configure(text=f"{int(value)}%")

    def _send_kfx_speed(self):
        if self._poll_stage != "idle":
            return
        global KFX_SPEED
        KFX_SPEED = self._kfx_speed_slider.get() / 100.0
        self._poll_stage = "kfx_speed"
        self._kfx_send_btn.configure(state="disabled", text="SENDING...")
        self._status_lbl.configure(text="Sending KFX speed...", text_color=C_MUTED)
        enqueue_packet(self.state, "kfx_speed", json.dumps({"speed": KFX_SPEED}))
        self._timeout_remaining = ACK_TIMEOUT_MS // 200
        self._start_poll()

    # ── Shared poll ───────────────────────────────────────────────────────

    def _start_poll(self):
        if self._poll_id is not None:
            try:
                self.after_cancel(self._poll_id)
            except Exception:
                pass
        try:
            self._poll_id = self.after(200, self._poll_events)
        except Exception:
            pass

    def _poll_events(self):
        self._poll_id = None
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return

        try:
            unmatched = []
            found = None
            while not self.state.event_queue.empty():
                try:
                    event = self.state.event_queue.get_nowait()
                except Exception:
                    break
                etype = event.get("type")
                if self._poll_stage == "battery" and etype == "battery_update":
                    found = event
                    break
                elif self._poll_stage == "kfx_speed" and etype == "kfx_speed_ack":
                    found = event
                    break
                else:
                    unmatched.append(event)
            for e in unmatched:
                self.state.event_queue.put(e)

            if found is not None:
                if self._poll_stage == "battery":
                    soc  = found["state_of_charge"]
                    v    = found["voltage"]
                    a    = found["current"]
                    w    = found["power"]
                    mins = found["time_remaining"]
                    self._batt_display.configure(
                        text=(f"SOC:  {soc:.1f}%\n"
                              f"Voltage:  {v:.2f} V\n"
                              f"Current:  {a:.2f} A\n"
                              f"Power:    {w:.1f} W\n"
                              f"Remaining:  {mins:.0f} min"),
                        text_color=C_TEXT,
                    )
                    self._batt_btn.configure(state="normal", text="REQUEST")
                    self._status_lbl.configure(text="Battery data updated.", text_color=C_SUCCESS)
                elif self._poll_stage == "kfx_speed":
                    self._kfx_send_btn.configure(state="normal", text="SEND")
                    self._status_lbl.configure(
                        text=f"KFX speed set to {int(KFX_SPEED * 100)}%.",
                        text_color=C_SUCCESS,
                    )
                self._poll_stage = "idle"
                self.after(4000, lambda: self._status_lbl.configure(text=""))
                return
        except Exception:
            pass

        self._timeout_remaining -= 1
        if self._timeout_remaining <= 0:
            if self._poll_stage == "battery":
                self._batt_btn.configure(state="normal", text="REQUEST")
            elif self._poll_stage == "kfx_speed":
                self._kfx_send_btn.configure(state="normal", text="SEND")
            self._status_lbl.configure(
                text="No response from Bullseye. Try again.",
                text_color=C_DANGER,
            )
            self._poll_stage = "idle"
            return

        try:
            self._poll_id = self.after(200, self._poll_events)
        except Exception:
            pass

    def _on_destroy(self, *_):
        if self._poll_id is not None:
            try:
                self.after_cancel(self._poll_id)
            except Exception:
                pass


# ============================================================
# SCREEN 10: BOT SETTINGS
# ============================================================

class BotSettingsScreen(BaseScreen):
    """
    Sub-menu for robot-side configuration.
    Routes to Boundary Settings and Home Settings sub-screens.
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        center = ctk.CTkFrame(self, fg_color="transparent")
        center.place(relx=0.5, rely=0.5, anchor="center")

        ctk.CTkLabel(center, text="BOT SETTINGS",
                     font=("Arial Bold", 30),
                     text_color=C_TEXT).pack(pady=(0, 50))

        make_nav_button(center, "BOUNDARY SETTINGS",
                        command=lambda: self.show(BoundarySettingsScreen)).pack(pady=14)

        make_nav_button(center, "HOME SETTINGS",
                        command=lambda: self.show(HomeSettingsScreen)).pack(pady=14)

        make_nav_button(center, "← BACK",
                        command=lambda: self.show(SettingsSubMenuScreen),
                        color=C_PRIMARY, width=220, height=65).pack(pady=(50, 0))


# ============================================================
# SCREEN 10a: BOUNDARY SETTINGS
# ============================================================

class BoundarySettingsScreen(BaseScreen):
    """
    Set the 4 corner coordinates of the arena boundary.

    Pre-populated from boundary.json if it exists.
    SAVE & SEND → validates inputs → sends 'set_boundary' → waits for
    'boundary_ack' (ACK_TIMEOUT_MS) → saves boundary.json on success.
    On timeout → shows inline error message.
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        self._poll_id: str | None = None
        self._waiting: bool = False
        self._timeout_remaining: int = 0
        self.bind("<Destroy>", self._on_destroy)

        # ── Title ─────────────────────────────────────────────────────────
        ctk.CTkLabel(self, text="BOUNDARY SETTINGS",
                     font=("Arial Bold", 40),
                     text_color=C_TEXT).pack(pady=(28, 4))
        ctk.CTkLabel(self,
                     text="Enter the X and Y coordinates of each arena corner.",
                     font=("Arial", 24), text_color=C_MUTED).pack(pady=(0, 16))

        # ── Corner entry grid ─────────────────────────────────────────────
        grid = ctk.CTkFrame(self, fg_color="transparent")
        grid.pack(pady=8)

        with self.state.lock:
            existing = self.state.boundary  # {"corners": [{x,y}×4]} or None

        self._entries: list[tuple] = []   # list of (x_entry, y_entry) per corner

        corner_labels = ["Corner 1\n(Top-Left)", "Corner 2\n(Top-Right)",
                         "Corner 3\n(Bottom-Left)", "Corner 4\n(Bottom-Right)"]

        for i, label in enumerate(corner_labels):
            col = i % 2
            row = (i // 2) * 3   # 3 rows per corner pair: label, x, y

            ctk.CTkLabel(grid, text=label,
                         font=("Arial Bold", 24), text_color=C_TEXT,
                         justify="center").grid(row=row, column=col, padx=40, pady=(12, 4))

            x_frame = ctk.CTkFrame(grid, fg_color="transparent")
            x_frame.grid(row=row + 1, column=col, padx=40, pady=2)
            ctk.CTkLabel(x_frame, text="X:", font=("Arial", 28),
                         text_color=C_MUTED, width=24).pack(side="left")
            #x_entry = ctk.CTkEntry(x_frame, width=140, font=("Arial", 16),
            #                        fg_color=C_SURFACE, text_color=C_TEXT,
            #                        border_color=C_SECONDARY)
            
            x_entry = ctk.CTkEntry(x_frame, width=ENTRY_WIDTH, height=ENTRY_HEIGHT,font=("Arial", BASE_FONT),
                                   fg_color=C_SURFACE,text_color=C_TEXT,border_color=C_SECONDARY)
            x_entry.pack(side="left")

            y_frame = ctk.CTkFrame(grid, fg_color="transparent")
            y_frame.grid(row=row + 2, column=col, padx=40, pady=(2, 8))
            ctk.CTkLabel(y_frame, text="Y:", font=("Arial", 28),
                         text_color=C_MUTED, width=24).pack(side="left")
            #y_entry = ctk.CTkEntry(y_frame, width=140, font=("Arial", 16),
            #                        fg_color=C_SURFACE, text_color=C_TEXT,
            #                        border_color=C_SECONDARY)
            y_entry = ctk.CTkEntry(y_frame,width=ENTRY_WIDTH, height=ENTRY_HEIGHT,font=("Arial",BASE_FONT),
                                   fg_color=C_SURFACE,text_color=C_TEXT,border_color=C_SECONDARY)
            y_entry.pack(side="left")

            # Pre-populate from existing boundary
            if existing and i < len(existing.get("corners", [])):
                c = existing["corners"][i]
                x_entry.insert(0, str(c.get("x", "")))
                y_entry.insert(0, str(c.get("y", "")))

            self._entries.append((x_entry, y_entry))

        # Attach numpad to all 8 entry fields
        all_entries = [e for pair in self._entries for e in pair]
        attach_numpad(self.app, *all_entries)

        # ── Status label ──────────────────────────────────────────────────
        self._status_lbl = ctk.CTkLabel(self, text="",
                                         font=("Arial", 18), text_color=C_MUTED)
        self._status_lbl.pack(pady=(8, 0))

        # ── Bottom buttons ────────────────────────────────────────────────
        btn_bar = ctk.CTkFrame(self, fg_color="transparent")
        btn_bar.pack(side="bottom", pady=24)

        make_nav_button(btn_bar, "← BACK",
                        command=lambda: self.show(BotSettingsScreen),
                        color=C_PRIMARY, width=210, height=65).pack(side="left", padx=20)

        self._save_btn = make_nav_button(btn_bar, "SAVE & SEND",
                                          command=self._save,
                                          width=230, height=65)
        self._save_btn.pack(side="right", padx=20)

    def _save(self):
        """Validate entries, send set_boundary, wait for boundary_ack."""
        if self._waiting:
            return

        corners = []
        for i, (x_ent, y_ent) in enumerate(self._entries):
            try:
                x = float(x_ent.get().strip())
                y = float(y_ent.get().strip())
                corners.append({"x": x, "y": y})
            except ValueError:
                self._status_lbl.configure(
                    text=f"Corner {i + 1}: X and Y must be numbers.",
                    text_color=C_DANGER,
                )
                x_ent.configure(border_color=C_DANGER)
                y_ent.configure(border_color=C_DANGER)
                self.after(1500, lambda xe=x_ent, ye=y_ent: (
                    xe.configure(border_color=C_SECONDARY),
                    ye.configure(border_color=C_SECONDARY),
                ))
                return

        self._waiting = True
        self._timeout_remaining = ACK_TIMEOUT_MS // 200
        self._save_btn.configure(state="disabled", text="SENDING...")
        self._status_lbl.configure(text="Sending boundary to Bullseye...",
                                    text_color=C_MUTED)

        payload = json.dumps({"corners": corners})
        enqueue_packet(self.state, "set_boundary", payload)
        self._poll_id = self.after(200, self._poll_ack)

    def _poll_ack(self):
        self._poll_id = None
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return

        try:
            unmatched = []
            found = None
            while not self.state.event_queue.empty():
                try:
                    event = self.state.event_queue.get_nowait()
                except Exception:
                    break
                if event.get("type") == "boundary_ack":
                    found = event
                    break
                else:
                    unmatched.append(event)
            for e in unmatched:
                self.state.event_queue.put(e)

            if found is not None:
                # Persist and update shared state
                boundary_data = {"corners": []}
                for xe, ye in self._entries:
                    boundary_data["corners"].append(
                        {"x": float(xe.get()), "y": float(ye.get())}
                    )
                with self.state.lock:
                    self.state.boundary = boundary_data
                save_boundary(boundary_data)
                self._waiting = False
                self._save_btn.configure(state="normal", text="SAVE & SEND")
                self._status_lbl.configure(
                    text="Boundary saved!", text_color=C_SUCCESS
                )
                self.after(3000, lambda: self._status_lbl.configure(text=""))
                return
        except Exception:
            pass

        self._timeout_remaining -= 1
        if self._timeout_remaining <= 0:
            self._waiting = False
            self._save_btn.configure(state="normal", text="SAVE & SEND")
            self._status_lbl.configure(
                text="No response from Bullseye — boundary not saved. Try again.",
                text_color=C_DANGER,
            )
            return

        try:
            self._poll_id = self.after(200, self._poll_ack)
        except Exception:
            pass

    def _on_destroy(self, *_):
        if self._poll_id is not None:
            try:
                self.after_cancel(self._poll_id)
            except Exception:
                pass


# ============================================================
# SCREEN 10b: HOME SETTINGS
# ============================================================

class HomeSettingsScreen(BaseScreen):
    """
    Visual map of the arena with Bullseye's current position and home controls.

    Left panel — tkinter Canvas showing:
      • Arena boundary rectangle (scaled from boundary.json corners)
      • Green dot at current bot position with a line indicating yaw
      • REFRESH button to re-request pos_data

    Right panel — controls:
      • Editable X, Y, Yaw fields (pre-filled from home.json)
      • UPDATE HOME → confirm → set_home → home_ack → save home.json
      • RETURN TO HOME → confirm → StateData(RETURN_TO_HOME)

    On any ack timeout → inline error label shown under the action buttons.
    """
    _CANVAS_W = 420
    _CANVAS_H = 360

    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        self._poll_id: str | None = None
        self._action_stage: str = "idle"   # idle | set_home | confirm_home | confirm_rth
        self._timeout_remaining: int = 0
        self._pending_home: dict | None = None   # set before entering set_home stage
        self.bind("<Destroy>", self._on_destroy)

        with self.state.lock:
            self._boundary = self.state.boundary    # may be None
            self._home     = dict(self.state.home_pos) if self.state.home_pos else None

        # ── Title ─────────────────────────────────────────────────────────
        ctk.CTkLabel(self, text="HOME SETTINGS",
                     font=("Arial Bold", 30), text_color=C_TEXT).pack(pady=(20, 10))

        # ── Two-column body ───────────────────────────────────────────────
        body = ctk.CTkFrame(self, fg_color="transparent")
        body.pack(fill="both", expand=True, padx=24, pady=8)

        # ── Left: canvas map ──────────────────────────────────────────────
        left = ctk.CTkFrame(body, fg_color=C_SURFACE, corner_radius=12)
        left.pack(side="left", fill="both", expand=True, padx=(0, 16))

        ctk.CTkLabel(left, text="ARENA MAP",
                     font=("Arial Bold", 16), text_color=C_MUTED).pack(pady=(12, 4))

        self._canvas = tk.Canvas(
            left,
            width = self._CANVAS_W, height = self._CANVAS_H,
            bg="#1a1a1a", highlightthickness=0,
        )
        #self._canvas.pack(padx=12, pady=8)
        self._canvas.pack(fill="both", expand=True, padx=12, pady=8)
        self._canvas.bind("<Configure>", lambda _: self._draw_map())

        # ── Right: controls ───────────────────────────────────────────────
        right = ctk.CTkFrame(body, fg_color="transparent", width=320)
        right.pack(side="right", fill="y")
        right.pack_propagate(False)

        ctk.CTkLabel(right, text="HOME POSITION",
                     font=("Arial Bold", 40), text_color=C_TEXT).pack(pady=(8, 16))

        # X / Y / Yaw entry fields
        #for label, attr in [("X", "_x_entry"), ("Y", "_y_entry"), ("Yaw (°)", "_yaw_entry")]:
        #    row = ctk.CTkFrame(right, fg_color="transparent")
        #    row.pack(fill="x", pady=4)
        #    ctk.CTkLabel(row, text=label, font=("Arial Bold", 16),
        #                text_color=C_MUTED, width=70).pack(side="left", padx=(0, 8))
        #    entry = ctk.CTkEntry(row, width=180, font=("Arial", 16),
        #                          fg_color=C_SURFACE, text_color=C_TEXT,
        #                          border_color=C_SECONDARY)
        #    entry.pack(side="left")
        #    setattr(self, attr, entry)

        for label, attr in [("X", "_x_entry"), ("Y", "_y_entry"), ("Yaw (°)", "_yaw_entry")]:
            field = ctk.CTkFrame(right, fg_color="transparent")
            field.pack(fill="x", pady=4)

            ctk.CTkLabel(
             field,
            text=label,
            font=("Arial Bold", 20),
            text_color=C_MUTED,
            justify="left"
            ).pack(anchor="w", pady=(0, 6))

            entry = ctk.CTkEntry(
                field,
                width=260,
                height=70,
                font=("Arial", 22),
                fg_color=C_SURFACE,
                text_color=C_TEXT,
                border_color=C_SECONDARY
            )
            entry.pack(fill="x")

            setattr(self, attr, entry)

        # Pre-fill from home.json
        if self._home:
            self._x_entry.insert(0, str(self._home.get("x", "")))
            self._y_entry.insert(0, str(self._home.get("y", "")))
            self._yaw_entry.insert(0, str(self._home.get("yaw", "")))

        # Attach numpad to all three entry fields
        attach_numpad(self.app, self._x_entry, self._y_entry, self._yaw_entry)

        # Status label for update-home feedback
        self._status_lbl = ctk.CTkLabel(right, text="",
                                         font=("Arial", 15), text_color=C_MUTED,
                                         wraplength=280, justify="center")
        self._status_lbl.pack(pady=(12, 0))

        # UPDATE HOME button — uses direct CTkButton (smaller font fits narrow panel)
        self._update_btn = ctk.CTkButton(
            right, text="UPDATE HOME",
            command=self._press_update_home,
            font=("Arial", 26, "bold"),
            fg_color=C_SECONDARY, hover_color=C_TERTIARY,
            text_color=C_BG, corner_radius=20,
            width=260, height=70,
        )
        self._update_btn.pack(pady=(16, 8))

        # RETURN TO HOME button
        #ctk.CTkButton(
        #    right, text="RETURN TO HOME",
        #    command=self._press_return_home,
        #    font=("Arial", 26, "bold"),
        #    fg_color=C_SUCCESS, hover_color=C_TERTIARY,
        #    text_color=C_BG, corner_radius=20,
        #    width=260, height=62,
        #).pack(pady=8)

        # ── Confirm overlay (hidden) ───────────────────────────────────────
        self._confirm_frame = ctk.CTkFrame(self, fg_color=C_SURFACE, corner_radius=16,
                                            width=540, height=200)
        self._confirm_lbl = ctk.CTkLabel(self._confirm_frame, text="",
                                          font=("Arial Bold", 22), text_color=C_TEXT,
                                          wraplength=500, justify="center")
        self._confirm_lbl.pack(pady=(28, 8))
        cfm_btns = ctk.CTkFrame(self._confirm_frame, fg_color="transparent")
        cfm_btns.pack(pady=(8, 28))
        make_nav_button(cfm_btns, "CANCEL",
                        command=self._cancel_confirm,
                        color=C_DANGER, width=160, height=60).pack(side="left", padx=16)
        make_nav_button(cfm_btns, "CONFIRM",
                        command=self._confirmed_action,
                        color=C_SUCCESS, width=160, height=60).pack(side="right", padx=16)

        # ── Bottom back button ────────────────────────────────────────────
        make_nav_button(self, "← BACK",
                        command=lambda: self.show(BotSettingsScreen),
                        color=C_PRIMARY, width=210, height=60).pack(side="bottom", pady=16)

        # ── Initial map draw ─────────────────────────────────────────────
        self._draw_map()

    # ── Canvas map ────────────────────────────────────────────────────────

    def _draw_map(self):
        """Draw the arena boundary and home position diamond."""
        self._canvas.delete("all")
        W = self._canvas.winfo_width()
        H = self._canvas.winfo_height()
        if W <= 1 or H <= 1:
            return  # Not yet laid out
        PAD = 32

        if not self._boundary or not self._boundary.get("corners"):
            self._canvas.create_text(
                W // 2, H // 2,
                text="No boundary set.\nSet boundary in Boundary Settings.",
                fill=C_MUTED, font=("Arial", 14), justify="center",
            )
            return

        corners = self._boundary["corners"]
        xs = [c["x"] for c in corners]
        ys = [c["y"] for c in corners]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        span_x = max_x - min_x or 1.0
        span_y = max_y - min_y or 1.0

        def to_canvas(x, y):
            cx = PAD + (x - min_x) / span_x * (W - 2 * PAD)
            cy = PAD + (1 - (y - min_y) / span_y) * (H - 2 * PAD)
            return cx, cy

        # Draw boundary polygon — order must be TL, TR, BR, BL (clockwise)
        # corners list is [TL, TR, BL, BR], so reorder to [0, 1, 3, 2]
        ordered = [corners[0], corners[1], corners[3], corners[2]] if len(corners) == 4 else corners
        pts = []
        for c in ordered:
            pts.extend(to_canvas(c["x"], c["y"]))
        self._canvas.create_polygon(pts, outline=C_SECONDARY, fill="", width=2)

        # Draw corner markers
        for c in corners:
            cx, cy = to_canvas(c["x"], c["y"])
            self._canvas.create_oval(cx - 4, cy - 4, cx + 4, cy + 4,
                                      fill=C_SECONDARY, outline="")

        # Draw home position (green diamond + yaw arrow)
        if self._home:
            import math
            hx, hy = to_canvas(self._home["x"], self._home["y"])
            size = 10
            self._canvas.create_polygon(
                hx, hy - size, hx + size, hy, hx, hy + size, hx - size, hy,
                fill=C_SUCCESS, outline="white", width=1, tags="home_marker",
            )
            yaw_rad = math.radians(self._home["yaw"])
            arrow_len = 28
            ax = hx + arrow_len * math.cos(yaw_rad)
            ay = hy - arrow_len * math.sin(yaw_rad)  # canvas y is flipped
            self._canvas.create_line(
                hx, hy, ax, ay,
                fill="white", width=2, arrow="last",
            )

    def _start_poll(self):
        self._cancel_poll()
        try:
            self._poll_id = self.after(200, self._poll_events)
        except Exception:
            pass

    def _poll_events(self):
        self._poll_id = None
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return

        try:
            # Drain the full queue so a stale event can't block the target.
            unmatched = []
            found = None
            while not self.state.event_queue.empty():
                try:
                    event = self.state.event_queue.get_nowait()
                except Exception:
                    break
                etype = event.get("type")
                if self._action_stage == "set_home" and etype == "home_ack":
                    found = event
                    break
                else:
                    unmatched.append(event)
            for e in unmatched:
                self.state.event_queue.put(e)

            if found is not None:
                with self.state.lock:
                    self.state.home_pos = dict(self._pending_home)
                save_home(self._pending_home)
                clear_kfx_for_home_mismatch(self.state, self._pending_home)
                self._home = dict(self._pending_home)
                self._draw_map()
                self._action_stage = "idle"
                self._update_btn.configure(state="normal", text="UPDATE HOME")
                self._status_lbl.configure(text="Home position updated!",
                                            text_color=C_SUCCESS)
                self.after(3000, lambda: self._status_lbl.configure(text=""))
                return
        except Exception:
            pass

        self._timeout_remaining -= 1
        if self._timeout_remaining <= 0:
            self._update_btn.configure(state="normal", text="UPDATE HOME")
            self._status_lbl.configure(
                text="No response from Bullseye — home not updated. Try again.",
                text_color=C_DANGER,
            )
            self._action_stage = "idle"
            return

        try:
            self._poll_id = self.after(200, self._poll_events)
        except Exception:
            pass

    # ── Update home flow ──────────────────────────────────────────────────

    def _press_update_home(self):
        """Validate entries and show confirm overlay."""
        # Only block if a home-update is already in flight; a pos refresh is fine
        # to interrupt (the new flow will cancel the old poll via _start_poll).
        if self._action_stage == "set_home":
            return
        try:
            x = float(self._x_entry.get().strip())
            y = float(self._y_entry.get().strip())
            yaw = float(self._yaw_entry.get().strip())
        except ValueError:
            self._status_lbl.configure(
                text="X, Y and Yaw must be numbers.", text_color=C_DANGER
            )
            return
        self._pending_home = {"x": x, "y": y, "yaw": yaw}
        self._action_stage = "confirm_home"
        self._confirm_lbl.configure(
            text=f"Set home to  X: {x:.2f}  Y: {y:.2f}  Yaw: {yaw:.1f}°?"
        )
        self._confirm_frame.place(relx=0.5, rely=0.5, anchor="center")
        self._confirm_frame.lift()

    # ── Return to home flow ───────────────────────────────────────────────

    def _press_return_home(self):
        """Show confirm overlay for return-to-home."""
        if self._action_stage == "set_home":
            return  # don't interrupt an active set_home flow
        self._action_stage = "confirm_rth"
        self._confirm_lbl.configure(text="Send Bullseye back to its home position?")
        self._confirm_frame.place(relx=0.5, rely=0.5, anchor="center")
        self._confirm_frame.lift()

    # ── Confirm overlay actions ───────────────────────────────────────────

    def _cancel_confirm(self):
        self._confirm_frame.place_forget()
        self._action_stage = "idle"

    def _confirmed_action(self):
        self._confirm_frame.place_forget()
        stage = self._action_stage

        if stage == "confirm_home":
            self._action_stage = "set_home"
            self._update_btn.configure(state="disabled", text="UPDATING...")
            self._status_lbl.configure(text="Updating home position...",
                                        text_color=C_MUTED)
            payload = json.dumps(self._pending_home)
            enqueue_packet(self.state, "set_home", payload)
            self._timeout_remaining = ACK_TIMEOUT_MS // 200
            self._start_poll()

        elif stage == "confirm_rth":
            self._action_stage = "idle"
            enqueue_state(self.state, State.RETURN_TO_HOME)

    # ── Cleanup ───────────────────────────────────────────────────────────

    def _cancel_poll(self):
        if self._poll_id is not None:
            try:
                self.after_cancel(self._poll_id)
            except Exception:
                pass
            self._poll_id = None

    def _on_destroy(self, *_):
        self._cancel_poll()


# ============================================================
# SCREEN 11: KFX SETTINGS
# ============================================================

class KFXSettingsScreen(BaseScreen):
    """
    Lets the user assign saved routes to KFX remote buttons 1–6.

    Buttons 1–6 = user-assignable to any saved route
    Button 7 = reserved (hardcoded on Pi – shown greyed, not assignable)
    Button 8 = reserved (hardcoded on Pi – shown greyed, not assignable)

    Interaction flow:
      1. Tap a numbered button on the phone graphic (1–6) → gold highlight
      2. Tap a route row in the table → assigns that route to the button
      3. Press SAVE & SEND → persists kfx_config.json + sends kfx_config packet

    Packet sent on SAVE:
      DataPacket(type="kfx_config",
                 json_data='{"1": route_id_or_null, ..., "6": route_id_or_null}')

    Pi side is handled by Comms/KFX.py (KFXController) which:
               - Receives 'kfx_config' via PiCommThread and persists it
               - Sends 'kfx_ack' back so this screen can confirm the save
               - Listens on /dev/ttyAMA0 for raw KFX button bytes (ASCII 49–56)
               - Buttons 1–6 → AUTONOMOUS with the user-assigned path_id
               - Buttons 7–8 → reserved/hardcoded behavior on Pi
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        with self.state.lock:
            # Local working copy – only committed to shared state on SAVE
            self._config: dict = dict(self.state.kfx_config)
            self._routes: dict = dict(self.state.routes)
            self._home_pos: dict | None = dict(self.state.home_pos) if self.state.home_pos else None

        self._selected_btn: str | None = None    # e.g. "3", "5"
        self._phone_buttons: dict = {}           # btn_num_str → CTkButton widget
        self._waiting_for_ack: bool = False      # True while awaiting kfx_ack from Pi
        self._ack_timeout_remaining: int = 0     # Countdown ticks before giving up
        self._save_btn: ctk.CTkButton | None = None  # Reference for state changes during ack wait

        # ── Page title ────────────────────────────────────────────────────
        ctk.CTkLabel(self, text="KFX REMOTE SETTINGS",
                     font=("Arial Bold", 28),
                     text_color=C_TEXT).pack(pady=(22, 10))

        # ── Bottom action bar ─────────────────────────────────────────────
        # IMPORTANT: pack side="bottom" frames BEFORE any frame that uses
        # expand=True. tkinter pack allocates space in pack order — if the
        # expanding columns frame is packed first it consumes all remaining
        # height and the bottom bar has nowhere to go.
        bottom = ctk.CTkFrame(self, fg_color="transparent")
        bottom.pack(side="bottom", pady=18)

        # Status label sits just above the bottom bar and shows send/error feedback
        self._status_lbl = ctk.CTkLabel(
            self, text="",
            font=("Arial", 16), text_color=C_MUTED,
        )
        self._status_lbl.pack(side="bottom", pady=(0, 4))

        make_nav_button(bottom, "← BACK",
                        command=lambda: self.show(SettingsSubMenuScreen),
                        color=C_PRIMARY, width=210, height=62).pack(side="left", padx=20)

        # Store reference so _save() can disable/re-enable it during ack wait
        self._save_btn = make_nav_button(bottom, "SAVE & SEND",
                                         command=self._save,
                                         width=230, height=62)
        self._save_btn.pack(side="right", padx=20)

        # ── Two-column layout ─────────────────────────────────────────────
        # Packed after bottom bar so expand=True fills only the remaining space.
        columns = ctk.CTkFrame(self, fg_color="transparent")
        columns.pack(fill="both", expand=True, padx=30, pady=10)

        # Left half: stylized phone graphic with numbered buttons
        phone_col = ctk.CTkFrame(columns, fg_color="transparent")
        phone_col.pack(side="left", fill="both", expand=True)
        self._build_phone(phone_col)

        # Right half: scrollable route assignment table
        route_col = ctk.CTkFrame(columns, fg_color=C_SURFACE, corner_radius=12)
        route_col.pack(side="right", fill="both", expand=True, padx=(20, 0))
        self._build_route_table(route_col)

    def _build_phone(self, parent):
        """
        Renders a stylized phone body containing an 8-button grid.
        Layout matches image 5 in the reference screenshots:
          Row 0: 7 | 8   (both reserved/greyed)
          Row 1: 5 | 6
          Row 2: 3 | 4
          Row 3: 1 | 2

        Buttons 7–8 are disabled (state="disabled") to show they are reserved.
        Buttons 1–6 are interactive and highlighted gold when selected.
        """
        #ctk.CTkLabel(parent, text="KFX REMOTE",
        #             font=("Arial Bold", 18),
        #             text_color=C_MUTED).pack(pady=(10, 6))

        # Phone body – height kept tight so it fits within the content area
        # after the 44px top bar and ~80px bottom action bar are accounted for.
        # pady is small so the body isn't pushed out of view.
        phone_body = ctk.CTkFrame(parent, fg_color=C_PRIMARY,
                                   corner_radius=28, width=300, height=440)
        phone_body.pack(pady=12)
        phone_body.pack_propagate(False)

        # Button grid centered inside the phone body.
        # rely=0.45 keeps the grid slightly above center, leaving room for the
        # legend label near the bottom of the phone body.
        grid = ctk.CTkFrame(phone_body, fg_color="transparent")
        grid.place(relx=0.5, rely=0.45, anchor="center")

        # (label_text, button_number_str, grid_row, grid_col)
        btn_layout = [
            ("7", "7", 0, 0), ("8", "8", 0, 1),
            ("5", "5", 1, 0), ("6", "6", 1, 1),
            ("3", "3", 2, 0), ("4", "4", 2, 1),
            ("1", "1", 3, 0), ("2", "2", 3, 1),
        ]

        for label, num_str, row, col in btn_layout:
            locked = num_str in ("7", "8")

            if locked:
                btn = ctk.CTkButton(
                    grid, text=label,
                    width=110, height=80,
                    font=("Arial", 20, "bold"),
                    fg_color=C_MUTED, hover_color=C_MUTED,
                    text_color=C_BG, corner_radius=8,
                    state="disabled",
                    command=lambda: None,
                )
            else:
                btn = ctk.CTkButton(
                    grid, text=self._btn_display_text(num_str),
                    width=110, height=80,
                    font=("Arial Bold", 16),
                    fg_color=C_SURFACE, hover_color=C_SECONDARY,
                    text_color=C_TEXT, corner_radius=8,
                    command=lambda n=num_str: self._select_kfx_button(n),
                )
                self._phone_buttons[num_str] = btn

            btn.grid(row=row, column=col, padx=6, pady=6)

        # Small legend at bottom of phone body explaining the fixed buttons.
        ctk.CTkLabel(phone_body,
                     text="7=RESERVED  |  8=RESERVED",
                     font=("Arial", 12), text_color=C_MUTED).place(
            relx=0.5, rely=0.94, anchor="center"
        )

    def _btn_display_text(self, num_str: str) -> str:
        """
        Returns display text for a KFX phone button.
        Shows the button number plus a truncated route name if assigned.
        """
        rid = self._config.get(num_str)
        if rid is None:
            return num_str
        entry = self._routes.get(str(rid), {})
        name = entry.get("name", f"ID{rid}") if isinstance(entry, dict) else str(entry)
        return f"{num_str}\n{name[:7]}"

    def _build_route_table(self, parent):
        """
        Right-side scrollable table listing all saved routes.

        Each row highlights on hover (Enter/Leave events) and confirms the
        assignment with a gold flash on click. A red 'REMOVE ASSIGNMENT' row
        sits at the top of the list so the user can clear a button without
        having to assign a different route first.

        Hover color is slightly brighter than the surface so the lift is
        subtle — not distracting while the user is reading the list.
        """
        C_ROW_HOVER  = "#3d3d3d"   # Slightly lighter than C_SURFACE for hover state
        C_ROW_ACTIVE = C_TERTIARY  # Gold flash on click/assign confirmation

        ctk.CTkLabel(parent, text="SAVED ROUTES",
                     font=("Arial Bold", 18),
                     text_color=C_TEXT).pack(pady=(16, 4))

        ctk.CTkLabel(parent,
                     text="① Select a remote button (1–6)\n② Tap a route to assign it",
                     font=("Arial", 13), text_color=C_MUTED,
                     justify="center").pack(pady=(0, 2))

        ctk.CTkLabel(parent,
                     text="Only routes matching your current home are shown.",
                     font=("Arial", 11), text_color=C_MUTED,
                     justify="center").pack(pady=(0, 8))

        # Column header row
        hdr = ctk.CTkFrame(parent, fg_color=C_BG, corner_radius=0)
        hdr.pack(fill="x", padx=10)
        ctk.CTkLabel(hdr, text="Path Name", font=("Arial Bold", 15),
                     text_color=C_TEXT, width=200,
                     anchor="w").pack(side="left", padx=12, pady=6)
        ctk.CTkLabel(hdr, text="ID", font=("Arial Bold", 15),
                     text_color=C_TEXT, width=56,
                     anchor="center").pack(side="right", padx=12)

        scroll = ctk.CTkScrollableFrame(parent, fg_color="transparent")
        scroll.pack(fill="both", expand=True, padx=10, pady=6)

        def _bind_row_events(row_frame, on_click, normal_color, hover_color):
            """
            Helper that attaches hover and click bindings to a row frame and
            all of its children. Binding children is necessary because tkinter
            child widgets absorb pointer events before the parent frame sees them.

            on_click    – callable with no arguments, called on left-click
            normal_color – fg_color when the pointer is not over the row
            hover_color  – fg_color to apply while the pointer is inside the row
            """
            def _enter(_):
                # _ is the tkinter event object – required by the binding protocol
                # but not needed here since we only care that the pointer entered.
                row_frame.configure(fg_color=hover_color)

            def _leave(_):
                row_frame.configure(fg_color=normal_color)

            def _click(_):
                # Brief gold flash to confirm the tap registered, then restore hover
                row_frame.configure(fg_color=C_ROW_ACTIVE)
                row_frame.after(180, lambda: row_frame.configure(fg_color=hover_color))
                on_click()

            for widget in [row_frame] + list(row_frame.winfo_children()):
                widget.bind("<Enter>",    _enter)
                widget.bind("<Leave>",    _leave)
                widget.bind("<Button-1>", _click)

        # ── Remove-assignment row (always at top, styled red) ─────────────
        # Clicking this sets the selected KFX button's assignment back to None.
        # Styled differently from route rows so it is clearly a destructive action.
        remove_row = ctk.CTkFrame(scroll, fg_color="#4a1010", corner_radius=6)
        remove_row.pack(fill="x", pady=(0, 8))

        ctk.CTkLabel(remove_row, text="✕  REMOVE ASSIGNMENT",
                     font=("Arial Bold", 15), text_color="#ff6b6b",
                     anchor="w").pack(side="left", padx=14, pady=12)

        _bind_row_events(
            remove_row,
            on_click=self._remove_assignment,
            normal_color="#4a1010",
            hover_color="#6b1a1a",
        )

        if not self._routes:
            ctk.CTkLabel(scroll, text="No routes saved yet.",
                         font=("Arial", 15), text_color=C_MUTED).pack(pady=30)
            return

        # Filter routes to only those whose home matches the current home position
        matching_routes = {
            rid_str: entry
            for rid_str, entry in self._routes.items()
            if homes_match(
                entry.get("home") if isinstance(entry, dict) else None,
                self._home_pos,
            )
        }

        if not matching_routes:
            msg = (
                "No routes match your current home position."
                if self._home_pos is not None
                else "No home position set.\nSet a home position to assign routes."
            )
            ctk.CTkLabel(scroll, text=msg,
                         font=("Arial", 15), text_color=C_MUTED,
                         justify="center").pack(pady=30)
            return

        # ── Route rows ────────────────────────────────────────────────────
        for route_id_str, entry in matching_routes.items():
            rid = int(route_id_str)
            name = entry.get("name", "") if isinstance(entry, dict) else str(entry)
            row = ctk.CTkFrame(scroll, fg_color=C_SURFACE, corner_radius=6)
            row.pack(fill="x", pady=3)

            ctk.CTkLabel(row, text=name, font=("Arial", 16),
                         text_color=C_TEXT, width=200,
                         anchor="w").pack(side="left", padx=12, pady=12)
            ctk.CTkLabel(row, text=str(rid), font=("Arial Bold", 16),
                         text_color=C_MUTED, width=56,
                         anchor="center").pack(side="right", padx=12)

            # Each row needs its own captured rid in the closure (r=rid default arg)
            _bind_row_events(
                row,
                on_click=lambda r=rid: self._assign_route(r),
                normal_color=C_SURFACE,
                hover_color=C_ROW_HOVER,
            )

        # Enable drag-to-scroll after all route rows are built
        enable_touch_scroll(scroll)

    def _select_kfx_button(self, btn_num: str):
        """Gold-highlight the tapped KFX button; deselect previous."""
        for btn in self._phone_buttons.values():
            btn.configure(fg_color=C_SURFACE)
        self._selected_btn = btn_num
        self._phone_buttons[btn_num].configure(fg_color=C_TERTIARY)

    def _assign_route(self, route_id: int):
        """
        Assign a route to the currently selected KFX button.
        Updates the local config copy and refreshes the button label.
        Nothing is sent to the Pi until the user presses SAVE & SEND.
        """
        if self._selected_btn is None:
            # No button highlighted – do nothing (user needs to pick one first)
            return
        self._config[self._selected_btn] = route_id
        # Refresh label on the phone button to show the assignment
        self._phone_buttons[self._selected_btn].configure(
            text=self._btn_display_text(self._selected_btn)
        )
        # Change to blue to indicate confirmed assignment, then deselect
        self._phone_buttons[self._selected_btn].configure(fg_color=C_SECONDARY)
        self._selected_btn = None

    def _remove_assignment(self):
        """
        Clear the assignment on the currently selected KFX button, setting it
        back to None (unassigned). Restores the phone button label to just its
        number and resets its color to the default surface color.
        Nothing is sent to the Pi until the user presses SAVE & SEND.
        """
        if self._selected_btn is None:
            # Nothing selected – ignore the tap
            return
        self._config[self._selected_btn] = None
        # Restore the phone button to its unassigned appearance
        self._phone_buttons[self._selected_btn].configure(
            text=self._selected_btn,   # Just the number, no route name
            fg_color=C_SURFACE,
        )
        self._selected_btn = None

    def _save(self):
        """
        Send 'kfx_config' to the Pi and wait for 'kfx_ack' before committing
        the local config file.

        Flow:
          1. Disable SAVE & SEND button and show "Sending..." status.
          2. Enqueue the kfx_config packet (SerialTXThread sends it).
          3. Poll event_queue every 300 ms for kfx_ack (up to 10 s timeout).
          4a. On ack → write to shared state + kfx_config.json → navigate away.
          4b. On timeout → re-enable button and show error so user can retry.

        This guarantees the Steam Deck and Pi configs are always in sync:
        the local file is only written after the Pi confirms it saved its copy.
        """
        if self._waiting_for_ack:
            return   # Ignore double-taps while waiting
        self._waiting_for_ack = True
        self._ack_timeout_remaining = ACK_TIMEOUT_MS // 200   # ticks at 200 ms each

        self._save_btn.configure(state="disabled", text="SENDING...")
        self._status_lbl.configure(text="Sending KFX config to Bullseye...", text_color=C_MUTED)

        payload = json.dumps(self._config)
        enqueue_packet(self.state, "kfx_config", payload)

        self._poll_ack()

    def _poll_ack(self):
        """
        Poll event_queue every 200 ms for 'kfx_ack' from the Pi.
        Stops on ack (success), timeout (failure), or frame destruction.
        """
        # ── Check for ack in queue ────────────────────────────────────────
        try:
            unmatched = []
            found = None
            while not self.state.event_queue.empty():
                try:
                    event = self.state.event_queue.get_nowait()
                except Exception:
                    break
                if event.get("type") == "kfx_ack":
                    found = event
                    break
                else:
                    unmatched.append(event)
            for e in unmatched:
                self.state.event_queue.put(e)

            if found is not None:
                # Pi confirmed save – now safe to write local config
                with self.state.lock:
                    self.state.kfx_config = dict(self._config)
                save_kfx_config(self._config)
                self.show(SettingsSubMenuScreen)
                return   # Frame destroyed – stop polling
        except Exception:
            pass

        # ── Timeout check ─────────────────────────────────────────────────
        self._ack_timeout_remaining -= 1
        if self._ack_timeout_remaining <= 0:
            self._waiting_for_ack = False
            self._save_btn.configure(state="normal", text="SAVE & SEND")
            self._status_lbl.configure(
                text="No response from Bullseye — config not saved. Try again.",
                text_color=C_DANGER,
            )
            print("[KFX] No ack received within timeout – Pi may be disconnected")
            return

        # ── Reschedule ───────────────────────────────────────────────────
        try:
            self.after(200, self._poll_ack)
        except Exception:
            pass   # Widget destroyed during navigation – stop silently


# ============================================================
# ON-SCREEN KEYBOARD
# In-app keyboard overlay for text entry. Parented to the calling screen
# so it always sits above the content. Supports:
#   Touch  – tap any key
#   D-pad  – move highlight; btn_A (×) selects current key
# ============================================================

class OnScreenKeyboard:
    """
    Compact keyboard card that slides up from the bottom of the screen.
    Parented directly to the calling screen — no full-screen overlay needed.

    A live text preview at the top of the card shows what has been typed
    so the user can always see their input regardless of screen layout.

    Navigation:
      D-pad left/right  – move between keys in current row (wraps)
      D-pad up/down     – move between rows (col clamped to row width)
      btn_A (×)         – press highlighted key
      Touch             – tap any key directly

    Key layout:
      1 2 3 4 5 6 7 8 9 0
      Q W E R T Y U I O P
      A S D F G H J K L ⌫
      Z X C V B N M . - _
      [      SPACE      ] [DONE]
    """

    _ROWS = [
        list("1234567890"),
        list("QWERTYUIOP"),
        list("ASDFGHJKL⌫"),
        list("ZXCVBNM.-_"),
    ]
    _KEY_W = 104
    _KEY_H = 56
    _PAD   = 4

    def __init__(self, parent: ctk.CTkFrame, app_state: AppState,
                 entry: ctk.CTkEntry, on_done=None):
        self._state   = app_state
        self._entry   = entry
        self._on_done = on_done
        self._visible = False

        # Navigation cursor (row into _ROWS; len(_ROWS) = SPACE/DONE row)
        self._row = 0
        self._col = 0

        # D-pad / btn_A edge-detection — each physical press fires exactly once
        self._prev_up    = False
        self._prev_down  = False
        self._prev_left  = False
        self._prev_right = False
        self._prev_a     = False

        # Widget refs: (row, col) for main grid; "SPC"/"DONE" for bottom row
        self._btns: dict = {}

        # Card placed directly on parent at the bottom — not shown until show()
        self._card = ctk.CTkFrame(parent, fg_color="#1e1e1e", corner_radius=16,
                                   border_width=2, border_color=C_PRIMARY)

        self._build()
        self._highlight()

    # ── Build ─────────────────────────────────────────────────────────────

    def _build(self):
        # ── Typed-text preview bar ────────────────────────────────────────
        preview = ctk.CTkFrame(self._card, fg_color=C_BG, corner_radius=8)
        preview.pack(fill="x", padx=12, pady=(10, 6))

        ctk.CTkLabel(preview, text="Typing:",
                     font=("Arial", 14), text_color=C_MUTED,
                     anchor="w").pack(side="left", padx=(10, 6), pady=8)

        self._display = ctk.CTkLabel(preview, text="",
                                      font=("Arial Bold", 20),
                                      text_color=C_TERTIARY, anchor="w")
        self._display.pack(side="left", fill="x", expand=True, pady=8)

        # ── Key grid ─────────────────────────────────────────────────────
        grid_frame = ctk.CTkFrame(self._card, fg_color="transparent")
        grid_frame.pack(padx=10, pady=(0, 4))

        for r, row_keys in enumerate(self._ROWS):
            for c, key in enumerate(row_keys):
                is_back = key == "⌫"
                btn = ctk.CTkButton(
                    grid_frame,
                    text=key,
                    width=self._KEY_W,
                    height=self._KEY_H,
                    font=("Arial Bold", 18),
                    fg_color=C_DANGER if is_back else C_PRIMARY,
                    hover_color="#991a00" if is_back else C_SECONDARY,
                    text_color=C_TEXT,
                    corner_radius=6,
                    command=lambda k=key: self._press(k),
                )
                btn.grid(row=r, column=c,
                         padx=self._PAD // 2, pady=self._PAD // 2)
                self._btns[(r, c)] = btn

        # ── Bottom row: SPACE (wide) + DONE ──────────────────────────────
        bottom_frame = ctk.CTkFrame(self._card, fg_color="transparent")
        bottom_frame.pack(padx=10, pady=(0, 10))

        spc_w  = self._KEY_W * 7 + self._PAD * 6
        done_w = self._KEY_W * 3 + self._PAD * 2

        self._btns["SPC"] = ctk.CTkButton(
            bottom_frame, text="SPACE",
            width=spc_w, height=self._KEY_H,
            font=("Arial Bold", 16),
            fg_color=C_PRIMARY, hover_color=C_SECONDARY,
            text_color=C_TEXT, corner_radius=6,
            command=lambda: self._press("SPC"),
        )
        self._btns["SPC"].pack(side="left", padx=self._PAD // 2)

        self._btns["DONE"] = ctk.CTkButton(
            bottom_frame, text="DONE",
            width=done_w, height=self._KEY_H,
            font=("Arial Bold", 16),
            fg_color=C_SECONDARY, hover_color=C_TERTIARY,
            text_color=C_TEXT, corner_radius=6,
            command=lambda: self._press("DONE"),
        )
        self._btns["DONE"].pack(side="left", padx=self._PAD // 2)

    # ── Highlight ─────────────────────────────────────────────────────────

    def _highlight(self):
        """Gold-highlight the cursor key; restore all others to defaults."""
        is_bottom = self._row == len(self._ROWS)

        for (r, c), btn in [(k, v) for k, v in self._btns.items()
                             if isinstance(k, tuple)]:
            key = self._ROWS[r][c]
            is_cur = (r == self._row and c == self._col and not is_bottom)
            if key == "⌫":
                btn.configure(fg_color=C_TERTIARY if is_cur else C_DANGER)
            else:
                btn.configure(fg_color=C_TERTIARY if is_cur else C_PRIMARY)

        self._btns["SPC"].configure(
            fg_color=C_TERTIARY if (is_bottom and self._col == 0) else C_PRIMARY)
        self._btns["DONE"].configure(
            fg_color=C_TERTIARY if (is_bottom and self._col == 1) else C_SECONDARY)

    # ── Key press ─────────────────────────────────────────────────────────

    def _press(self, key: str):
        if key == "⌫":
            current = self._entry.get()
            if current:
                self._entry.delete(len(current) - 1, "end")
        elif key == "SPC":
            self._entry.insert("end", " ")
        elif key == "DONE":
            self.hide()
            return
        else:
            self._entry.insert("end", key)
        # Keep preview in sync after every keystroke
        self._display.configure(text=self._entry.get())

    # ── Visibility ────────────────────────────────────────────────────────

    def show(self):
        # Sync preview to whatever is already in the entry (e.g. initial_name)
        self._display.configure(text=self._entry.get())
        self._card.place(relx=0.5, rely=1.0, anchor="s", x=0, y=-10)
        self._card.lift()
        self._visible = True
        self._poll()   # Start gamepad polling — must be after _visible = True

    def hide(self):
        self._card.place_forget()
        self._visible = False
        if self._on_done:
            self._on_done()

    # ── D-pad polling (50 ms loop, runs only while keyboard is visible) ───

    def _poll(self):
        if not self._visible:
            return

        with self._state.lock:
            ctrl = self._state.controller

        up    = bool(ctrl.dpad_up)
        down  = bool(ctrl.dpad_down)
        left  = bool(ctrl.dpad_left)
        right = bool(ctrl.dpad_right)
        a_btn = bool(ctrl.btn_A)

        total_rows = len(self._ROWS) + 1   # +1 for SPACE/DONE row

        if right and not self._prev_right:
            max_col = 1 if self._row == len(self._ROWS) else len(self._ROWS[self._row]) - 1
            self._col = (self._col + 1) % (max_col + 1)
            self._highlight()

        if left and not self._prev_left:
            max_col = 1 if self._row == len(self._ROWS) else len(self._ROWS[self._row]) - 1
            self._col = (self._col - 1) % (max_col + 1)
            self._highlight()

        if down and not self._prev_down:
            self._row = (self._row + 1) % total_rows
            if self._row == len(self._ROWS):
                # Map proportionally: cols 0–6 → SPACE, 7–9 → DONE
                self._col = 0 if self._col <= 6 else 1
            else:
                self._col = min(self._col, len(self._ROWS[self._row]) - 1)
            self._highlight()

        if up and not self._prev_up:
            self._row = (self._row - 1) % total_rows
            if self._row == len(self._ROWS):
                self._col = min(self._col, 1)
            else:
                self._col = min(self._col, len(self._ROWS[self._row]) - 1)
            self._highlight()

        if a_btn and not self._prev_a:
            if self._row == len(self._ROWS):
                self._press("SPC" if self._col == 0 else "DONE")
            else:
                self._press(self._ROWS[self._row][self._col])

        self._prev_up    = up
        self._prev_down  = down
        self._prev_left  = left
        self._prev_right = right
        self._prev_a     = a_btn

        try:
            self._card.after(50, self._poll)
        except Exception:
            pass   # Widget destroyed on navigation – stop silently


# ============================================================
# PERSISTENT RIBBON WIDGETS
# Full-width banners placed on the root window via place() so they
# survive frame swaps.  Each has an ✕ button to dismiss.
# ============================================================

class ErrorRibbonWidget:
    """
    Red full-width ribbon shown whenever the Pi sends a type="error" packet.
    Dismissed by pressing the ✕ button on the right side.
    Lives on the root window so it persists across screen transitions.
    """
    _HEIGHT = 52

    def __init__(self, root: ctk.CTk):
        self._frame = ctk.CTkFrame(root, fg_color=C_DANGER, corner_radius=0,
                                    height=self._HEIGHT)
        self._lbl = ctk.CTkLabel(
            self._frame, text="",
            font=("Arial Bold", 16), text_color=C_TEXT,
            anchor="w",
        )
        self._lbl.pack(side="left", fill="x", expand=True, padx=(16, 8), pady=8)

        ctk.CTkButton(
            self._frame, text="✕",
            font=("Arial Bold", 18), width=44, height=36,
            fg_color="#991a00", hover_color="#660d00",
            text_color=C_TEXT, corner_radius=8,
            command=self.hide,
        ).pack(side="right", padx=(0, 10), pady=8)

    def show(self, message: str):
        self._lbl.configure(text=f"ERROR: {message}")
        self._frame.place(x=0, y=50, anchor="nw", relwidth=1.0)
        self._frame.lift()

    def hide(self):
        self._frame.place_forget()

    def lift(self):
        self._frame.lift()


class RouteFinishedRibbonWidget:
    """
    Green full-width ribbon shown when the Pi signals a route or
    return-to-home has completed (type="route_finished").
    Dismissed by pressing the ✕ button on the right side.
    Lives on the root window so it persists across screen transitions.
    """
    _HEIGHT = 52

    def __init__(self, root: ctk.CTk):
        self._frame = ctk.CTkFrame(root, fg_color=C_SUCCESS, corner_radius=0,
                                    height=self._HEIGHT)
        self._lbl = ctk.CTkLabel(
            self._frame, text="",
            font=("Arial Bold", 16), text_color=C_TEXT,
            anchor="w",
        )
        self._lbl.pack(side="left", fill="x", expand=True, padx=(16, 8), pady=8)

        ctk.CTkButton(
            self._frame, text="✕",
            font=("Arial Bold", 18), width=44, height=36,
            fg_color="#145214", hover_color="#0d3a0d",
            text_color=C_TEXT, corner_radius=8,
            command=self.hide,
        ).pack(side="right", padx=(0, 10), pady=8)

    def show(self, message: str):
        self._lbl.configure(text=message)
        self._frame.place(x=0, y=50, anchor="nw", relwidth=1.0)
        self._frame.lift()

    def hide(self):
        self._frame.place_forget()

    def lift(self):
        self._frame.lift()


# ============================================================
# E-STOP OVERLAY WIDGET
# Persistent bottom-left button that lives on the root window
# via place() so it survives frame swaps.
# Hidden on StartupScreen and MainMenuScreen; shown everywhere else.
# ============================================================

class EStopWidget:
    """
    120×60 red E-STOP button placed on the root window.
    On press: sends DISABLED, stops joystick stream, sets e_stop_active,
    then navigates to MainMenuScreen.

    show() / hide() are called by BullseyeApp.show_frame() on every
    screen transition. lift() keeps it above the content frame.
    """
    def __init__(self, root: ctk.CTk, app, app_state: AppState):
        self._app = app
        self._state = app_state

        self._btn = ctk.CTkButton(
            root,
            text="E-STOP",
            font=("Arial", 22, "bold"),
            fg_color=C_DANGER, hover_color="#991a00",
            text_color=C_TEXT, corner_radius=12,
            width=120, height=60,
            command=self._on_press,
        )
        # Start hidden; show_frame() will call show() when appropriate
        # _place coordinates are set in show() so we only place when needed

    def show(self):
        self._btn.place(x=12, y=WINDOW_H - 12, anchor="sw")
        self._btn.lift()

    def hide(self):
        self._btn.place_forget()

    def lift(self):
        """Re-raise above any newly packed content frame."""
        self._btn.lift()

    def _on_press(self):
        self._app.trigger_emergency_stop()

    #def _on_press(self):
    #    enqueue_state(self._state, State.DISABLED)
    #    with self._state.lock:
    #        self._state.joystick_active = False
    #        self._state.e_stop_active = True
    #    self._app.show_frame(MainMenuScreen)


# ============================================================
# ROOT APPLICATION WINDOW
# ============================================================

class BullseyeApp(ctk.CTk):
    """
    Root CustomTkinter window. Owns:
      - A persistent top status bar (robot battery left, Steam Deck battery right)
        hidden only on the StartupScreen.
      - The single full-screen content container below the bar.
      - Frame swap logic (show_frame).
      - DebugOverlay instance (if DEBUG_OVERLAY=True).
      - References to all background threads for clean shutdown.

    All screen navigation goes through show_frame(), which destroys
    the old frame and constructs the new one fresh each time.
    """
    def __init__(self, app_state: AppState,
                 joystick_thread: JoystickThread,
                 tx_thread: SerialTXThread,
                 rx_thread: SerialRXThread):
        super().__init__()

        self.app_state = app_state
        self._prev_y_pressed = False
        self._joystick_thread = joystick_thread
        self._tx_thread = tx_thread
        self._rx_thread = rx_thread

        # ── Window configuration ──────────────────────────────────────────
        self.title("Bullseye Controller")
        self.geometry(f"{WINDOW_W}x{WINDOW_H}")
        self.resizable(False, False)
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        self.configure(fg_color=C_BG)

        # ── Fullscreen on Steam Deck (Linux) ──────────────────────────────
        # On Windows (dev machine) the 1280×800 window stays windowed.
        import platform
        if platform.system() == "Linux":
            self.attributes("-fullscreen", True)

        # ── Hide mouse cursor ─────────────────────────────────────────────
        self.config(cursor="none")

        # ── Persistent top status bar ─────────────────────────────────────
        # Lives on the root window so it survives frame swaps.
        # Hidden on StartupScreen; shown on every other screen.
        # Left  side: robot battery (received from Pi over XBee).
        # Right side: Steam Deck battery (read from Linux sysfs).
        self._top_bar = ctk.CTkFrame(self, fg_color=C_SURFACE,
                                      height=50, corner_radius=0)
        self._top_bar.pack(fill="x", side="top")
        self._top_bar.pack_propagate(False)   # Keep fixed height

        # ── Load and crop the combined bull+controller icon image ─────────
        # The source image is a side-by-side pair: bull (left half) and
        # controller (right half) on a steel background.
        # We crop each half, then use ImageOps to tighten to the brightest
        # region by thresholding only the very bright neon core (≥195/255)
        # so the glow halo doesn't inflate the bounding box.
        def _tight_crop(half: Image.Image) -> Image.Image:
            gray = half.convert("L")
            # Mask: only pixels brighter than 195 count as "content"
            mask = gray.point(lambda p: 255 if p >= 195 else 0)
            bbox = mask.getbbox()
            if bbox:
                pad = 18  # extra padding to keep some neon halo
                x0 = max(0,           bbox[0] - pad)
                y0 = max(0,           bbox[1] - pad)
                x1 = min(half.width,  bbox[2] + pad)
                y1 = min(half.height, bbox[3] + pad)
                # Make it square so CTkImage doesn't distort
                w, h = x1 - x0, y1 - y0
                if w > h:
                    diff = w - h
                    y0 = max(0, y0 - diff // 2)
                    y1 = min(half.height, y1 + diff // 2)
                elif h > w:
                    diff = h - w
                    x0 = max(0, x0 - diff // 2)
                    x1 = min(half.width, x1 + diff // 2)
                return half.crop((x0, y0, x1, y1))
            return half

        _icon_src = Image.open("assets/Bull and controller on steel background.png")
        _iw, _ih  = _icon_src.size
        _bull_ctk = ctk.CTkImage(_tight_crop(_icon_src.crop((0, 0, _iw // 2, _ih))), size=(44, 44))
        _ctrl_ctk = ctk.CTkImage(_tight_crop(_icon_src.crop((_iw // 2, 0, _iw, _ih))), size=(44, 44))

        # ── Robot battery (left side) ─────────────────────────────────────
        robot_frame = ctk.CTkFrame(self._top_bar, fg_color="transparent")
        robot_frame.pack(side="left", padx=16)

        ctk.CTkLabel(robot_frame, image=_bull_ctk, text="").pack(side="left", padx=(0, 6))

        self._robot_batt_lbl = ctk.CTkLabel(
            robot_frame, text="--%",
            font=("Arial Bold", 18), text_color=C_TEXT,
        )
        self._robot_batt_lbl.pack(side="left", padx=(0, 8))

        self._robot_batt_canvas = tk.Canvas(
            robot_frame, width=40, height=18,
            bg=C_SURFACE, highlightthickness=0,
        )
        self._robot_batt_canvas.pack(side="left")

        # ── Steam Deck battery (right side) ───────────────────────────────
        deck_frame = ctk.CTkFrame(self._top_bar, fg_color="transparent")
        deck_frame.pack(side="right", padx=16)

        # Pack right-to-left: canvas → percentage → icon
        self._deck_batt_canvas = tk.Canvas(
            deck_frame, width=40, height=18,
            bg=C_SURFACE, highlightthickness=0,
        )
        self._deck_batt_canvas.pack(side="right")

        self._deck_batt_lbl = ctk.CTkLabel(
            deck_frame, text="--%",
            font=("Arial Bold", 18), text_color=C_TEXT,
        )
        self._deck_batt_lbl.pack(side="right", padx=(0, 8))

        ctk.CTkLabel(deck_frame, image=_ctrl_ctk, text="").pack(side="right", padx=(0, 6))


        # Start refreshing both battery labels every 5 seconds.
        # Robot battery updates at robot pace (~every 5 s from Pi).
        # Steam Deck battery changes slowly so 5 s is more than enough.
        self._refresh_top_bar()

        # ── Content container (fills remaining space below the top bar) ───
        self._container = ctk.CTkFrame(self, fg_color=C_BG, corner_radius=0)
        self._container.pack(fill="both", expand=True)
        self._current_frame: BaseScreen | None = None

        # ── Debug overlay (placed on root so it survives frame swaps) ─────
        self._overlay: DebugOverlay | None = None
        if DEBUG_OVERLAY:
            self._overlay = DebugOverlay(self, app_state)

        # ── E-Stop overlay (persistent bottom-left button) ────────────────
        self._estop = EStopWidget(self, self, app_state)

        # ── Persistent ribbon overlays ────────────────────────────────────
        self._error_ribbon = ErrorRibbonWidget(self)
        self._route_ribbon = RouteFinishedRibbonWidget(self)

        # ── Navigate to startup screen ────────────────────────────────────
        self.show_frame(StartupScreen)

        # ── Global event polling (path_created etc.) ──────────────────────
        self._poll_global_events()

        self.after(50, self._poll_global_inputs)

        # ── TEMP: show ribbons immediately for visual testing ─────────────
        #self.after(2000, lambda: self._error_ribbon.show("Motor controller timeout"))
        #self.after(10000, lambda: self._route_ribbon.show("Route complete"))

        # ── Handle window close button ────────────────────────────────────
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def trigger_emergency_stop(self):
        """Global emergency stop used by widget and Steam Deck Y button."""
        enqueue_state(self.app_state, State.DISABLED)
        with self.app_state.lock:
            self.app_state.joystick_active = False
            self.app_state.e_stop_active = True

        if not isinstance(self._current_frame, MainMenuScreen):
            self.show_frame(MainMenuScreen)


    def _poll_global_inputs(self):
        """Catch global controller shortcuts that should work on every screen."""
        try:
            with self.app_state.lock:
                y_pressed = bool(self.app_state.controller.btn_Y)

        # edge detect so it only fires once per press
            if y_pressed and not self._prev_y_pressed:
                self.trigger_emergency_stop()

            self._prev_y_pressed = y_pressed
            self.after(50, self._poll_global_inputs)

        except Exception:
            pass


    def _draw_battery(self, canvas: tk.Canvas, percent: float):
        """Fill the canvas to visually represent battery percentage."""
        canvas.delete("all")
        percent = max(0, min(100, percent))  # clamp 0–100
    
        # fill width proportional to soc
        fill_w = int((percent / 100) * 36)
        color = "#00ff00" if percent > 50 else ("#ffaa00" if percent > 20 else "#ff0000")

        # outer border (battery body)
        canvas.create_rectangle(1, 1, 38, 16, outline=C_TEXT, width=1)
        # terminal
        canvas.create_rectangle(38, 5, 40, 12, fill=C_TEXT, outline=C_TEXT)
        # fill level
        if fill_w > 0:
            canvas.create_rectangle(2, 2, 2 + fill_w, 15, fill=color, width=0)




    def _refresh_top_bar(self):
        """
        Poll both batteries every 5 seconds and update the top bar labels.
        Runs forever via after() chaining; safe to call even when the bar
        is hidden because label.configure() on a hidden widget is a no-op.

        Robot battery: read from app_state.battery (updated by SerialRXThread).
        Deck battery:  read from Linux sysfs via get_steamdeck_battery().
                       Returns None on Windows; label shows '--' in that case.
        """
        try:
            with self.app_state.lock:
                robot_soc = self.app_state.battery.state_of_charge
            self._robot_batt_lbl.configure(text=f"{robot_soc:.0f}%")
            self._draw_battery(self._robot_batt_canvas, robot_soc)

            deck_soc = get_steamdeck_battery()
            if deck_soc is None:
                self._deck_batt_lbl.configure(text="--%")
                self._draw_battery(self._deck_batt_canvas, 0)
            else:
                self._deck_batt_lbl.configure(text=f"{deck_soc}%")
                self._draw_battery(self._deck_batt_canvas, deck_soc)



        except Exception as e:
            print(f"[TOP BAR] Refresh error: {e}")

        self.after(5000, self._refresh_top_bar)

    def show_frame(self, screen_class: type, **kwargs):
        """
        Destroy the current screen frame and construct the new one.
        Extra kwargs (e.g. route_id=5) are forwarded to the screen's __init__.

        The top bar is hidden on StartupScreen and visible on all other screens.
        The debug overlay is re-raised after each swap so it stays on top.
        """
        if self._current_frame is not None:
            self._current_frame.destroy()

        # Hide top bar on startup screen; show on everything else
        if screen_class is StartupScreen:
            self._top_bar.pack_forget()
        else:
            # Re-pack at top if it was previously hidden.
            # Must pack before _container so it stays at the top.
            self._top_bar.pack(fill="x", side="top", before=self._container)

        frame = screen_class(self._container, self, self.app_state, **kwargs)
        frame.pack(fill="both", expand=True)
        self._current_frame = frame

        # E-Stop overlay: hidden on Startup and Main Menu, visible everywhere else
        if screen_class in (StartupScreen, MainMenuScreen):
            self._estop.hide()
        else:
            self._estop.show()

        # Re-raise overlays above the new frame
        if self._overlay:
            self._overlay.lift()
        self._estop.lift()
        self._error_ribbon.lift()
        self._route_ribbon.lift()

    def toggle_debug_overlay(self, enabled: bool):
        """Show or hide the debug TX-log overlay."""
        global DEBUG_OVERLAY
        DEBUG_OVERLAY = enabled
        if enabled and self._overlay is None:
            self._overlay = DebugOverlay(self, self.app_state)
            self._overlay.lift()
        elif not enabled and self._overlay is not None:
            self._overlay.frame.place_forget()
            self._overlay = None

    def _poll_global_events(self):
        """
        Check the event queue every 500 ms for cross-screen events.

        path_created: only kept alive while RecordRouteScreen is active.
        If the user e-stopped or navigated away mid-save, discard it so it
        cannot trigger NameRouteScreen on a future recording.

        All other events are put back for the active screen to consume.
        """
        try:
            # Snapshot the queue depth so we don't loop on events we just
            # re-queued (which would cause an infinite loop this tick).
            depth = self.app_state.event_queue.qsize()
            for _ in range(depth):
                try:
                    event = self.app_state.event_queue.get_nowait()
                except Exception:
                    break
                if event.get("type") == "path_created":
                    # Only keep if RecordRouteScreen is still waiting for it
                    if isinstance(self._current_frame, RecordRouteScreen):
                        self.app_state.event_queue.put(event)
                    # else: stale — discard
                elif event.get("type") == "error":
                    self._error_ribbon.show(event.get("errstr", "Unknown error"))
                elif event.get("type") == "route_finished":
                    msg = event.get("message", "Route complete")
                    self._route_ribbon.show(msg)
                else:
                    # All other events belong to screen-level pollers; put back
                    self.app_state.event_queue.put(event)
        except Exception:
            pass
        self.after(500, self._poll_global_events)

    def _on_close(self):
        """
        Clean shutdown sequence:
          1. Send DISABLED so robot stops regardless of current state
          2. Give TX thread one cycle to transmit it
          3. Stop all threads, quit pygame, destroy window
        """
        print("[APP] Shutting down – sending DISABLED to robot...")
        enqueue_state(self.app_state, State.DISABLED)
        with self.app_state.lock:
            self.app_state.joystick_active = False
        time.sleep(0.12)   # One TX cycle at 20 Hz

        self._joystick_thread.stop()
        self._tx_thread.stop()
        self._rx_thread.stop()
        pygame.quit()
        self.destroy()


# ============================================================
# ENTRY POINT
# ============================================================

def main():
    # ── Shared state ──────────────────────────────────────────────────────
    app_state = AppState()

    # ── Serial connection ─────────────────────────────────────────────────
    ser = None
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)   # Allow XBee to initialize
        print(f"[OK] Connected to XBee on {PORT} at {BAUD} baud")
    except serial.SerialException:
        if REQUIRE_CONNECTION:
            print(f"[ERROR] Could not open {PORT}. "
                  f"Check XBee connection and try again.")
            sys.exit(1)
        else:
            print(f"[WARN] No serial on {PORT} – running in UI-only mode. "
                  f"Set REQUIRE_CONNECTION=True to enforce hardware presence.")

    # ── Background threads ────────────────────────────────────────────────
    joystick_thread = JoystickThread(app_state)
    tx_thread       = SerialTXThread(app_state, ser)
    rx_thread       = SerialRXThread(app_state, ser)

    joystick_thread.start()
    tx_thread.start()
    rx_thread.start()

    # ── GUI (blocks until window is closed) ───────────────────────────────
    app = BullseyeApp(app_state, joystick_thread, tx_thread, rx_thread)
    app.mainloop()

    # ── Post-mainloop cleanup ─────────────────────────────────────────────
    if ser and ser.is_open:
        ser.close()
    print("[APP] Exited cleanly.")


if __name__ == "__main__":
    main()
