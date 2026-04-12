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
# TOGGLE FLAGS  –  change only these two lines to flip behavior
# ============================================================
DEBUG_OVERLAY      = False    # Show semi-transparent TX log overlay on screen
REQUIRE_CONNECTION = False   # True = halt on missing XBee; False = UI-only mode
FAKE_ROUTE_SAVE    = False   # True = skip Pi; FINISH ROUTE immediately generates a random route ID
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
import subprocess
from pathlib import Path
from collections import deque
from PIL import Image
from PIL import Image, ImageTk


# ── Package path so Comms/Robot imports resolve from project root ─────────────
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from Models.ControllerData import ControllerData
from Models.StateData import State, StateData
from Models.BatteryData import BatteryData
from Models.DataPacket import DataPacket
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
    Load route_id → name mapping from routes.json.
    Returns an empty dict if the file does not yet exist or cannot be parsed.
    Keys are stored as strings (JSON requirement); callers convert as needed.
    """
    if ROUTES_FILE.exists():
        try:
            with open(ROUTES_FILE, "r") as f:
                return json.load(f)
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
        self.routes: dict = load_routes()       # {"route_id_str": "name", ...}
        self.kfx_config: dict = load_kfx_config()  # {"3": id_or_None, ...}

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

            elif packet.type == "path_created":
                # Pi has finished saving a route; GUI needs the ID to prompt for a name.
                # Pi sends PathCreated(id=N) via PiCommThread.send_new_path_data().
                data = json.loads(packet.json_data)
                self.app_state.event_queue.put({
                    "type": "path_created",
                    "route_id": data.get("id"),
                })

            elif packet.type == "kfx_ack":
                # Pi confirmed it received and saved the KFX config.
                # KFXSettingsScreen._save() is polling for this event before
                # it commits the local config file on the Steam Deck side.
                self.app_state.event_queue.put({"type": "kfx_ack"})

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
    First screen shown on launch. No robot communication happens here.
    Displays the Bullseye logo and a START button.

    TODO - JAY - Maybe get the font to match the bullseye screen - later
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        # Center everything vertically and horizontally
        center = ctk.CTkFrame(self, fg_color="transparent")
        center.place(relx=0.5, rely=0.5, anchor="center")

        # ── Logo placeholder ──────────────────────────────────────────────
        # Replace this CTkLabel with a CTkImage widget once assets/logo.png exists

        self.logo_image = ctk.CTkImage(
            Image.open("assets/logo.png"),
            size=(800,500)
        )

        ctk.CTkLabel(
            center,
            image = self.logo_image,
            text = ""
        ).pack()
       
       
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

        # ── START button ──────────────────────────────────────────────────
        make_nav_button(
            center, "START",
            command=lambda: self.show(MainMenuScreen),
            width=600, height=100,
        ).pack(pady=(0,100))


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

    On enter   → sends StateData(TELEOP) + sets joystick_active=True
    BACK       → sends StateData(DISABLED) + joystick_active=False → Main Menu
    RETURN     → returns to Main Menu; robot stays in TELEOP and joystick
                 keeps streaming (joystick_active remains True).
                 This lets the user navigate the menu while the robot is live.
                 Add enqueue_state(DISABLED) here if that behavior is unwanted.
    """
    def __init__(self, parent, app, app_state: AppState): #the state needs to match the state in the statedata.py file - 2 for teleop
        super().__init__(parent, app, app_state)

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

        # ── Bottom button bar ─────────────────────────────────────────────
        btn_bar = ctk.CTkFrame(self, fg_color="transparent")
        btn_bar.pack(side="bottom", pady=35)

        make_nav_button(btn_bar, "BACK",
                        command=self._back,
                        color=C_DANGER, width=220).pack(side="left", padx=40)

        make_nav_button(btn_bar, "SET HOME",
                        command=self._return_to_menu,
                        color=C_SUCCESS, width=220).pack(side="right", padx=40)

        # ── Activate TELEOP on screen construction ────────────────────────
        self._enter()

    def _enter(self):
        """Send TELEOP state and open joystick stream."""
        enqueue_state(self.state, State.TELEOP)
        with self.state.lock:
            self.state.joystick_active = True

    def _back(self):
        """Disable robot, stop joystick stream, return to Main Menu."""
        enqueue_state(self.state, State.DISABLED)
        with self.state.lock:
            self.state.joystick_active = False
        self.show(MainMenuScreen)

    def _return_to_menu(self):
        """
        Return to Main Menu while keeping the robot active.
        joystick_active stays True so the TX thread keeps streaming.
        The user can re-enter Free Drive or use BACK from Main Menu to stop.
        """
        self.show(MainMenuScreen)


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

    On enter     → StateData(RECORD_PATH) + joystick_active=True
    FINISH ROUTE → StateData(DISABLED); Pi cancels CreatePathCmd, saves CSV,
                   sends DataPacket(type="path_created", json_data={"id": N});
                   GUI waits for 'path_created' event then navigates to NameRouteScreen.
    CANCEL ROUTE → StateData(DISABLED) → PathSubMenu
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        self._waiting_for_confirm = False   # True after FINISH pressed
        self._finish_timeout_id: str | None = None
        self._poll_id: str | None = None      # after() id for _poll_events; cancelled on destroy
        self.bind("<Destroy>", self._on_destroy)

        # ── Center content ────────────────────────────────────────────────
        center = ctk.CTkFrame(self, fg_color="transparent")
        center.place(relx=0.5, rely=0.5, anchor="center")

        self._status_label = ctk.CTkLabel(
            center,
            text="● RECORDING\nDRIVE THE ROUTE YOU WANT TO SAVE",
            font=("Arial Black", 36),
            text_color=C_DANGER,
            justify="center",
        )
        self._status_label.pack(pady=30)

        # ── Bottom button bar ─────────────────────────────────────────────
        btn_bar = ctk.CTkFrame(self, fg_color="transparent")
        btn_bar.pack(side="bottom", pady=35)

        self._cancel_btn = make_nav_button(
            btn_bar, "CANCEL ROUTE",
            command=self._cancel,
            color=C_DANGER, width=240,
        )
        self._cancel_btn.pack(side="left", padx=40)

        self._finish_btn = make_nav_button(
            btn_bar, "FINISH ROUTE",
            command=self._finish,
            color=C_SECONDARY, width=240,
        )
        self._finish_btn.pack(side="right", padx=40)

        # ── Start recording on screen construction ────────────────────────
        self._enter()
        self._poll_events()   # Begin polling for path_created confirmation

    def _enter(self):
        """
        Called once on screen construction.
        Sends TELEOP so the robot accepts joystick input, then sends
        'record_start' so the Pi enables localization and begins collecting
        path points. Opens the joystick stream so the user can drive.
        """
        enqueue_state(self.state, State.RECORD_PATH)
        enqueue_packet(self.state, "record_start")
        with self.state.lock:
            self.state.joystick_active = True

    def _finish(self):
        """Send finish command and wait for Pi to confirm the saved route ID."""
        if self._waiting_for_confirm:
            return
        self._waiting_for_confirm = True
        self._cancel_finish_timeout()
        enqueue_state(self.state, State.DISABLED)
        with self.state.lock:
            self.state.joystick_active = False

        if FAKE_ROUTE_SAVE:
            import random
            fake_id = random.randint(1000, 9999)
            self.show(NameRouteScreen, route_id=fake_id)
            return

        enqueue_packet(self.state, "record_finish")
        self._status_label.configure(
            text="⏳ SAVING ROUTE...\nWaiting for Pi confirmation",
            text_color=C_TERTIARY,
        )
        # Disable both buttons while waiting so the user cannot double-send
        self._finish_btn.configure(state="disabled")
        self._cancel_btn.configure(state="disabled")
        self._finish_timeout_id = self.after(
            int(RECORD_FINISH_TIMEOUT * 1000),
            self._on_finish_timeout,
        )

    def _on_destroy(self, *_):
        """Cancel any pending after() callbacks when the frame is destroyed."""
        if self._poll_id is not None:
            try:
                self.after_cancel(self._poll_id)
            except Exception:
                pass
            self._poll_id = None
        self._cancel_finish_timeout()

    def _cancel_finish_timeout(self):
        if self._finish_timeout_id is not None:
            try:
                self.after_cancel(self._finish_timeout_id)
            except Exception:
                pass
            self._finish_timeout_id = None

    def _on_finish_timeout(self):
        self._finish_timeout_id = None
        if not self._waiting_for_confirm:
            return
        self._waiting_for_confirm = False
        self._status_label.configure(
            text="SAVE TIMED OUT\nCheck connection and try again",
            text_color=C_DANGER,
        )
        self._finish_btn.configure(state="normal")
        self._cancel_btn.configure(state="normal")

    def _cancel(self):
        """Discard the in-progress route and return to the path menu."""
        self._cancel_finish_timeout()
        enqueue_packet(self.state, "record_cancel")
        enqueue_state(self.state, State.DISABLED)
        with self.state.lock:
            self.state.joystick_active = False
        self.show(PathSubMenuScreen)

    def _poll_events(self):
        """
        Check the inbound event queue every 200 ms for a 'path_created'
        confirmation from the Pi. When received, navigate to NameRouteScreen.
        Stops when the frame is destroyed (after_id cancelled in _on_destroy).
        """
        self._poll_id = None
        # If this widget was already destroyed (e.g. e-stop fired), do nothing.
        try:
            if not self.winfo_exists():
                return
        except Exception:
            return
        try:
            while not self.state.event_queue.empty():
                event = self.state.event_queue.get_nowait()
                if event.get("type") == "path_created":
                    self._cancel_finish_timeout()
                    with self.state.lock:
                        self.state.joystick_active = False
                    route_id = event.get("route_id")
                    self.show(NameRouteScreen, route_id=route_id)
                    return   # Frame will be destroyed; do not reschedule
                elif event.get("type") == "connection_lost":
                    # Put it back for the global app poller to display the warning.
                    # Do NOT return here – keep rescheduling so we still catch
                    # path_created if the Pi manages to send it before fully disconnecting.
                    self.state.event_queue.put(event)
        except Exception:
            pass
        try:
            self._poll_id = self.after(200, self._poll_events)
        except Exception:
            pass   # Widget destroyed – stop polling


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

        with self.state.lock:
            self.state.routes[str(self._route_id)] = name
            routes_snapshot = dict(self.state.routes)   # Copy under lock for safe JSON write
        save_routes(routes_snapshot)
        self.show(self._return_screen or PathSubMenuScreen)


# ============================================================
# SCREEN 7: RUN ROUTE
# ============================================================

class RunRouteScreen(BaseScreen):
    """
    Displays saved routes for selection, a speed slider, and run controls.

    RUN          → StateData(AUTONOMOUS, path_id=selected, path_speed=slider/100)
    STOP         → StateData(DISABLED) — replaces RUN button after route starts
    RETURN HOME  → StateData(AUTONOMOUS, path_id=-1, path_speed=0.5)
                   Convention: path_id = -1 signals the Pi to run the home route.
                   TODO: Confirm path_id=-1 as 'home' convention with Pi team.
    BACK         → StateData(DISABLED) if running, then → PathSubMenu

    TODO: for testing need to get "fake" routes inside so that we can test 
    the state logic change for the run route and the return home
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        self._selected_id: int | None = None
        self._route_buttons: dict = {}        # route_id (int) → CTkButton (main row)
        self._route_action_frames: dict = {}  # route_id (int) → CTkFrame (hidden sub-row)
        self._expanded_id: int | None = None  # which row is currently expanded
        self._is_running: bool = False

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

        # Action buttons
        self._run_btn = make_nav_button(
            controls, "RUN",
            command=self._run,
            width=230, height=72,
        )
        self._run_btn.pack(pady=8)

        self._stop_btn = make_nav_button(
            controls, "STOP",
            command=self._stop,
            color=C_DANGER, width=230, height=72,
        )
        self._stop_btn.pack(pady=8)
        self._stop_btn.configure(state="disabled")

        make_nav_button(controls, "HOME",
                        command=self._return_home,
                        color=C_SUCCESS, width=230, height=62).pack(pady=8)

        make_nav_button(controls, "← BACK",
                        command=self._back,
                        color=C_PRIMARY, width=230, height=62).pack(pady=(20, 0))

    def _build_route_list(self):
        """Populate the scrollable route list from app_state.routes."""
        for w in self._route_scroll.winfo_children():
            w.destroy()
        self._route_buttons.clear()
        self._route_action_frames.clear()
        self._expanded_id = None
        self._selected_id = None

        with self.state.lock:
            routes = dict(self.state.routes)

        if not routes:
            ctk.CTkLabel(self._route_scroll,
                         text="No saved routes.\nRecord a route first.",
                         font=("Arial", 16), text_color=C_MUTED,
                         justify="center").pack(pady=50)
            return

        for route_id_str, name in routes.items():
            rid = int(route_id_str)

            # Outer container — holds row button + collapsible action frame
            outer = ctk.CTkFrame(self._route_scroll, fg_color="transparent")
            outer.pack(fill="x", pady=2)

            # Main row button
            btn = ctk.CTkButton(
                outer,
                text=f"  {name}   (ID {rid})",
                font=("Arial Bold", 18),
                fg_color=C_SURFACE, hover_color=C_SECONDARY,
                text_color=C_TEXT, corner_radius=8,
                height=62, anchor="w",
                command=lambda r=rid: self._tap_route(r),
            )
            btn.pack(fill="x")
            self._route_buttons[rid] = btn

            # Action sub-frame — hidden until the row is expanded
            action = ctk.CTkFrame(outer, fg_color=C_SURFACE, corner_radius=8)
            # Not packed yet; _tap_route will pack/forget it
            self._route_action_frames[rid] = action

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
                color = C_TERTIARY if old == self._selected_id else C_SURFACE
                self._route_buttons[old].configure(fg_color=color)

        if self._expanded_id == route_id:
            # Same row tapped again → collapse
            af = self._route_action_frames.get(route_id)
            if af:
                af.pack_forget()
            self._expanded_id = None
        else:
            # Expand this row
            self._expanded_id = route_id
            af = self._route_action_frames.get(route_id)
            if af:
                self._populate_action_normal(route_id, af)
                af.pack(fill="x", padx=4, pady=(0, 4))

        # Always select this route for running
        for btn in self._route_buttons.values():
            btn.configure(fg_color=C_SURFACE)
        self._selected_id = route_id
        if route_id in self._route_buttons:
            self._route_buttons[route_id].configure(fg_color=C_TERTIARY)

    def _populate_action_normal(self, route_id: int, frame: ctk.CTkFrame):
        """Fill the action frame with [RENAME] [DELETE] buttons."""
        for w in frame.winfo_children():
            w.destroy()
        frame.configure(fg_color=C_SURFACE)

        with self.state.lock:
            name = self.state.routes.get(str(route_id), "")

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

    def _run(self):
        """
        Send AUTONOMOUS command with selected route ID and speed.
        If no route is selected the list panel flashes red for 800 ms as a
        no-popup error indicator so the user knows to pick a route first.
        """
        if self._selected_id is None:
            # Flash the list panel background red then restore it.
            # Uses two chained after() calls: one to apply red, one to restore.
            self._list_frame.configure(fg_color=C_DANGER)
            self.after(800, lambda: self._list_frame.configure(fg_color=C_SURFACE))
            return
        # Slider value is 10–100 (integer steps); convert to 0.0–1.0 for the Pi.
        speed = self._speed_slider.get() / 100.0
        enqueue_state(self.state, State.AUTONOMOUS,
                      path_id=self._selected_id, path_speed=speed)
        self._is_running = True
        self._run_btn.configure(state="disabled")
        self._stop_btn.configure(state="normal")

    def _stop(self):
        """Send DISABLED, re-enable the RUN button."""
        enqueue_state(self.state, State.DISABLED)
        self._is_running = False
        self._run_btn.configure(state="normal")
        self._stop_btn.configure(state="disabled")

    def _return_home(self):
        """
        Trigger the robot's return-home routine.
        path_id = -1 is the agreed convention for 'go home'.
        TODO: Confirm this convention with the Pi team before final deployment.
        """
        enqueue_state(self.state, State.AUTONOMOUS, path_id=-1, path_speed=0.5)
        self._is_running = True
        self._run_btn.configure(state="disabled")
        self._stop_btn.configure(state="normal")

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
    Sliders to adjust deadzone and update rate in-memory.
    Changes apply immediately to the running threads (global vars).

    TODO: Persist these values between sessions. Options:
          a) Write to a settings.json alongside routes.json
          b) Patch Robot/Constants.py (requires restart)
          c) Send a settings packet to the Pi if the Pi also needs them
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        card = ctk.CTkFrame(self, fg_color=C_SURFACE, corner_radius=16)
        card.place(relx=0.5, rely=0.5, anchor="center", width=640, height=500)

        ctk.CTkLabel(card, text="CONTROLLER SETTINGS",
                     font=("Arial Bold", 28),
                     text_color=C_TEXT).pack(pady=(36, 24))

        # ── Deadzone slider ───────────────────────────────────────────────
        ctk.CTkLabel(card, text="Joystick Deadzone",
                     font=("Arial", 18), text_color=C_MUTED).pack()

        self._dz_label = ctk.CTkLabel(card, text=f"{DEADZONE:.2f}",
                                       font=("Arial Bold", 26),
                                       text_color=C_TERTIARY)
        self._dz_label.pack()

        dz_slider = ctk.CTkSlider(
            card, from_=0.0, to=0.5, number_of_steps=50,
            width=440, button_color=C_TERTIARY,
            progress_color=C_SECONDARY,
            command=self._dz_changed,
        )
        dz_slider.set(DEADZONE)   # Initialize thumb to current value
        dz_slider.pack(pady=(0, 32))

        # ── Update rate slider ────────────────────────────────────────────
        ctk.CTkLabel(card, text="Update Rate (seconds per cycle)",
                     font=("Arial", 18), text_color=C_MUTED).pack()

        self._rate_label = ctk.CTkLabel(card, text=f"{UPDATE_HZ:.3f}s",
                                         font=("Arial Bold", 26),
                                         text_color=C_TERTIARY)
        self._rate_label.pack()

        rate_slider = ctk.CTkSlider(
            card, from_=0.02, to=0.2, number_of_steps=18,
            width=440, button_color=C_TERTIARY,
            progress_color=C_SECONDARY,
            command=self._rate_changed,
        )
        rate_slider.set(UPDATE_HZ)   # Initialize thumb to current value
        rate_slider.pack(pady=(0, 36))

        make_nav_button(card, "← BACK",
                        command=lambda: self.show(SettingsSubMenuScreen),
                        color=C_PRIMARY, width=220, height=65).pack()

    def _dz_changed(self, value):
        """
        Slider callback for deadzone adjustment.
        Updates the module-level DEADZONE global so JoystickThread picks
        up the new value on its next axis-read cycle without a restart.
        """
        global DEADZONE
        DEADZONE = round(float(value), 2)
        self._dz_label.configure(text=f"{DEADZONE:.2f}")

    def _rate_changed(self, value):
        """
        Slider callback for update rate adjustment.
        Updates the module-level UPDATE_HZ global so both JoystickThread
        and SerialTXThread pick up the new sleep duration on their next cycle.
        """
        global UPDATE_HZ
        UPDATE_HZ = round(float(value), 3)
        self._rate_label.configure(text=f"{UPDATE_HZ:.3f}s")


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
    Placeholder screen for boundary configuration options.

    TODO: Define boundary parameters to expose once the Pi-side API is designed.
          Candidates: field boundary coordinates, fence limits, no-go zones.
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        card = ctk.CTkFrame(self, fg_color=C_SURFACE, corner_radius=16)
        card.place(relx=0.5, rely=0.5, anchor="center", width=520, height=300)

        ctk.CTkLabel(card, text="BOUNDARY SETTINGS",
                     font=("Arial Bold", 28),
                     text_color=C_TEXT).pack(pady=(40, 16))

        ctk.CTkLabel(card, text="Boundary settings coming soon.",
                     font=("Arial", 18), text_color=C_MUTED).pack(pady=20)

        make_nav_button(card, "← BACK",
                        command=lambda: self.show(BotSettingsScreen),
                        color=C_PRIMARY, width=220, height=65).pack(pady=24)


# ============================================================
# SCREEN 10b: HOME SETTINGS
# ============================================================

class HomeSettingsScreen(BaseScreen):
    """
    Placeholder screen for home position configuration options.

    TODO: Define home settings to expose once the Pi-side API is designed.
          Candidates: home position coordinates, return speed, home trigger behavior.
    """
    def __init__(self, parent, app, app_state: AppState):
        super().__init__(parent, app, app_state)

        card = ctk.CTkFrame(self, fg_color=C_SURFACE, corner_radius=16)
        card.place(relx=0.5, rely=0.5, anchor="center", width=520, height=300)

        ctk.CTkLabel(card, text="HOME SETTINGS",
                     font=("Arial Bold", 28),
                     text_color=C_TEXT).pack(pady=(40, 16))

        ctk.CTkLabel(card, text="Home settings coming soon.",
                     font=("Arial", 18), text_color=C_MUTED).pack(pady=20)

        make_nav_button(card, "← BACK",
                        command=lambda: self.show(BotSettingsScreen),
                        color=C_PRIMARY, width=220, height=65).pack(pady=24)


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
        name = self._routes.get(str(rid), f"ID{rid}")
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
                     justify="center").pack(pady=(0, 10))

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

        # ── Route rows ────────────────────────────────────────────────────
        for route_id_str, name in self._routes.items():
            rid = int(route_id_str)
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
        self._ack_timeout_remaining = 20   # 20 × 500 ms = 10 s total timeout

        # Disable button and show sending state so the user knows it's working
        self._save_btn.configure(state="disabled", text="SENDING...")

        # Send packet – SerialTXThread will transmit on its next cycle
        payload = json.dumps(self._config)
        enqueue_packet(self.state, "kfx_config", payload)

        # Start polling for the ack
        self._poll_ack()

    def _poll_ack(self):
        """
        Poll event_queue every 500 ms for 'kfx_ack' from the Pi.
        Stops on ack (success), timeout (failure), or frame destruction.
        """
        # ── Check for ack in queue ────────────────────────────────────────
        try:
            while not self.state.event_queue.empty():
                event = self.state.event_queue.get_nowait()
                if event.get("type") == "kfx_ack":
                    # Pi confirmed save – now safe to write local config
                    with self.state.lock:
                        self.state.kfx_config = dict(self._config)
                    save_kfx_config(self._config)
                    self.show(SettingsSubMenuScreen)
                    return   # Frame destroyed – stop polling
                else:
                    # Not our event – put it back for other pollers
                    self.state.event_queue.put(event)
                    break
        except Exception:
            pass

        # ── Timeout check ─────────────────────────────────────────────────
        self._ack_timeout_remaining -= 1
        if self._ack_timeout_remaining <= 0:
            # Timed out – re-enable button so user can try again
            self._waiting_for_ack = False
            self._save_btn.configure(state="normal", text="SAVE & SEND")
            # Flash button red briefly to signal the failure
            self._save_btn.configure(fg_color=C_DANGER)
            self.after(1000, lambda: self._save_btn.configure(fg_color=C_SECONDARY))
            print("[KFX] No ack received within timeout – Pi may be disconnected")
            return

        # ── Reschedule ───────────────────────────────────────────────────
        try:
            self.after(500, self._poll_ack)
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
        enqueue_state(self._state, State.DISABLED)
        with self._state.lock:
            self._state.joystick_active = False
            self._state.e_stop_active = True
        self._app.show_frame(MainMenuScreen)


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
                                      height=44, corner_radius=0)
        self._top_bar.pack(fill="x", side="top")
        self._top_bar.pack_propagate(False)   # Keep fixed height

        # Robot battery label – bottom-left
        #self._robot_batt_lbl = ctk.CTkLabel(
        #    self._top_bar,
        #    text="🐂 --%",
        #    font=("Arial Bold", 18),
        #    text_color=C_TEXT,
        #)
        #self._robot_batt_lbl.pack(side="left", padx=16)

        # Steam Deck battery label – top-right
        #self._deck_batt_lbl = ctk.CTkLabel(
        #    self._top_bar,
        #    text="🎮 --%",
        #    font=("Arial Bold", 18),
        #    text_color=C_TEXT,
        #)
        #self._deck_batt_lbl.pack(side="right", padx=16)

        # ── Robot battery visual ──────────────────────────────────────────────── = shit i added to get the icons to show up
        robot_frame = ctk.CTkFrame(self._top_bar, fg_color="transparent")
        robot_frame.pack(side="left", padx=16)

        self._robot_batt_lbl = ctk.CTkLabel(
            robot_frame, text="🐂 --%", font=("Segoe UI Emoji", 18),
            text_color=C_TEXT
        )
        self._robot_batt_lbl.pack(side="left", padx=(0, 8))

        self._robot_batt_canvas = tk.Canvas(
         robot_frame, width=40, height=18,
         bg=C_BG, highlightthickness=1, highlightbackground=C_TEXT
        )
        
        
        self._robot_batt_canvas.pack(side="left")

# ── Steam Deck battery visual ───────────────────────────────────────────
        deck_frame = ctk.CTkFrame(self._top_bar, fg_color="transparent")
        deck_frame.pack(side="right", padx=16)

        self._deck_batt_lbl = ctk.CTkLabel(
            deck_frame, text="🎮 --%", font=("Arial Bold", 18),
           text_color=C_TEXT
        )
        self._deck_batt_lbl.pack(side="right", padx=(8, 0))

        self._deck_batt_canvas = tk.Canvas(
            deck_frame, width=40, height=18,
            bg=C_BG, highlightthickness=1, highlightbackground=C_TEXT
        )
        self._deck_batt_canvas.pack(side="right")


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

        # ── Navigate to startup screen ────────────────────────────────────
        self.show_frame(StartupScreen)

        # ── Global event polling (path_created etc.) ──────────────────────
        self._poll_global_events()

        # ── Handle window close button ────────────────────────────────────
        self.protocol("WM_DELETE_WINDOW", self._on_close)


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
            # ── Robot battery (from Pi over XBee) ────────────────────────
            #with self.app_state.lock:
            #    robot_soc = self.app_state.battery.state_of_charge
            #self._robot_batt_lbl.configure(text=f"🤖 {robot_soc:.0f}%")

            # ── Steam Deck battery (Linux sysfs) ─────────────────────────
            #deck_soc = get_steamdeck_battery()
            #deck_text = f"🎮 {deck_soc}%" if deck_soc is not None else "🎮 --%"
            #self._deck_batt_lbl.configure(text=deck_text)

            with self.app_state.lock:
             robot_soc = self.app_state.battery.state_of_charge
            self._robot_batt_lbl.configure(text=f" 🐂 {robot_soc:.0f}%")
            self._draw_battery(self._robot_batt_canvas, robot_soc)

            deck_soc = get_steamdeck_battery()
            if deck_soc is None:
                deck_soc = 0
            self._deck_batt_lbl.configure(text=f"🎮 {deck_soc}%")
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

    def _poll_global_events(self):
        """
        Check the event queue every 500 ms for cross-screen events.

        path_created: only kept alive while RecordRouteScreen is active.
        If the user e-stopped or navigated away mid-save, discard it so it
        cannot trigger NameRouteScreen on a future recording.

        All other events are put back for the active screen to consume.
        """
        try:
            while not self.app_state.event_queue.empty():
                event = self.app_state.event_queue.get_nowait()
                if event.get("type") == "path_created":
                    # Only re-queue if RecordRouteScreen is still waiting for it
                    if isinstance(self._current_frame, RecordRouteScreen):
                        self.app_state.event_queue.put(event)
                    # else: stale event — silently discard
                else:
                    self.app_state.event_queue.put(event)
                break   # Avoid infinite re-queue loop in a single poll cycle
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
