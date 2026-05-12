#!/usr/bin/env python3
"""
Drone Controller - PX4 Pixhawk 6C Terminal Controller
Uses MAVLink Mission Protocol (waypoints) for all movement - same as QGroundControl
Proper ACK handling, mission upload protocol, reliable command delivery
"""

import sys
import time
import math
import os
from pymavlink import mavutil
from threading import Thread, Event, Lock
import argparse

if sys.platform == 'win32':
    try:
        import serial.tools.list_ports
        WINDOWS_SERIAL_AVAILABLE = True
    except ImportError:
        WINDOWS_SERIAL_AVAILABLE = False
    try:
        import ctypes
        kernel32 = ctypes.windll.kernel32
        kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)
    except:
        pass
else:
    WINDOWS_SERIAL_AVAILABLE = False


class Colors:
    HEADER  = '\033[95m'
    OKBLUE  = '\033[94m'
    OKCYAN  = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL    = '\033[91m'
    ENDC    = '\033[0m'
    BOLD    = '\033[1m'

    @staticmethod
    def disable():
        Colors.HEADER = Colors.OKBLUE = Colors.OKCYAN = ''
        Colors.OKGREEN = Colors.WARNING = Colors.FAIL = ''
        Colors.ENDC = Colors.BOLD = ''


MAV_RESULT_STR = {0:'ACCEPTED', 1:'TEMP_REJECTED', 2:'DENIED',
                  3:'UNSUPPORTED', 4:'FAILED', 5:'IN_PROGRESS'}
MAV_MISSION_STR = {0:'ACCEPTED', 1:'ERROR', 2:'UNSUPPORTED_FRAME',
                   3:'UNSUPPORTED', 4:'NO_SPACE', 5:'INVALID',
                   13:'INVALID_SEQUENCE', 14:'DENIED'}
PX4_MAIN_MODES = {1:'MANUAL', 2:'ALTCTL', 3:'POSCTL', 4:'AUTO',
                  5:'ACRO', 6:'OFFBOARD', 7:'STABILIZED'}
PX4_AUTO_SUB   = {1:'READY', 2:'TAKEOFF', 3:'LOITER',
                  4:'MISSION', 5:'RTL', 6:'LAND'}


# ===========================================================================
# DroneController
# ===========================================================================
class DroneController:
    """
    PX4 drone controller using proper MAVLink mission protocol.
    All movement commands upload a mission and execute it - exactly like QGroundControl.
    """

    # Timeouts (seconds) - match QGroundControl values
    ACK_TIMEOUT         = 5.0    # COMMAND_ACK timeout
    MISSION_ACK_TIMEOUT = 10.0   # MISSION_ACK timeout (upload/download)
    MISSION_ITEM_TIMEOUT = 3.0   # per-item request timeout
    MAX_RETRIES         = 5      # max retries before giving up

    def __init__(self, connection_string, baud=57600):
        self.connection_string = connection_string
        self.baud = baud
        self.master = None
        self.connected = False
        self.debug = True

        # Telemetry state
        self.armed             = False
        self.mode              = 'UNKNOWN'
        self.latitude          = 0.0
        self.longitude         = 0.0
        self.altitude_amsl     = 0.0
        self.altitude_relative = 0.0
        self.heading           = 0.0
        self.gps_fix           = 0
        self.num_satellites    = 0
        self.battery_voltage   = 0.0
        self.battery_remaining = 0

        # Thread-safe ACK storage for COMMAND_ACK
        self._cmd_ack_lock  = Lock()
        self._cmd_acks      = {}   # {command_id: {result, component, ts}}

        # Thread-safe storage for MISSION protocol messages
        self._mission_lock       = Lock()
        self._mission_request_q  = []   # incoming MISSION_REQUEST seq numbers
        self._mission_ack        = None  # last MISSION_ACK received
        self._mission_count_recv = None  # MISSION_COUNT received during download

        # Background monitor
        self._stop_event    = Event()
        self._monitor_thread = None

        if sys.platform == 'win32':
            os.system('cls')
        else:
            os.system('clear')

    # -----------------------------------------------------------------------
    # Connection
    # -----------------------------------------------------------------------
    def connect(self):
        C = Colors
        print(f"{C.OKCYAN}[INFO] Connecting to {self.connection_string}...{C.ENDC}")
        try:
            self.master = mavutil.mavlink_connection(
                self.connection_string, baud=self.baud,
                source_system=255, source_component=190)

            print(f"{C.OKCYAN}[INFO] Waiting for heartbeat...{C.ENDC}")
            hb = self.master.wait_heartbeat(timeout=15)
            if not hb:
                print(f"{C.FAIL}[ERROR] No heartbeat received{C.ENDC}")
                return False

            # If we landed on the telemetry radio (sysid=0), hunt for Pixhawk
            if self.master.target_system == 0:
                print(f"{C.WARNING}[WARNING] Got telemetry radio (sysid=0), waiting for Pixhawk...{C.ENDC}")
                deadline = time.time() + 15
                while time.time() < deadline:
                    msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                    if msg and msg.get_srcSystem() == 1:
                        self.master.target_system    = 1
                        self.master.target_component = 1
                        print(f"{C.OKGREEN}[SUCCESS] Found Pixhawk (sysid=1){C.ENDC}")
                        break
                else:
                    print(f"{C.FAIL}[ERROR] Pixhawk not found. Check TELEM1 config.{C.ENDC}")
                    return False

            self.connected = True
            print(f"{C.OKGREEN}[SUCCESS] Connected! sysid={self.master.target_system}{C.ENDC}")

            # Start background telemetry thread
            self._stop_event.clear()
            self._monitor_thread = Thread(target=self._monitor_loop, daemon=True)
            self._monitor_thread.start()

            # Request all streams at 4 Hz
            self._request_streams()
            time.sleep(1)
            return True

        except Exception as e:
            print(f"{Colors.FAIL}[ERROR] Connection failed: {e}{Colors.ENDC}")
            return False

    def _request_streams(self):
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4, 1)

    def close(self):
        print("[INFO] Closing connection...")
        self._stop_event.set()
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2)
        if self.master:
            self.master.close()
        self.connected = False

    # -----------------------------------------------------------------------
    # Background telemetry monitor
    # -----------------------------------------------------------------------
    def _monitor_loop(self):
        while not self._stop_event.is_set():
            try:
                msg = self.master.recv_match(blocking=True, timeout=0.05)
                if msg:
                    self._handle_msg(msg)
            except Exception as e:
                if self.debug:
                    print(f"[DEBUG] Monitor error: {e}")

    def _handle_msg(self, msg):
        t = msg.get_type()

        if t == 'HEARTBEAT':
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.mode  = self._decode_mode(msg.custom_mode)

        elif t == 'GLOBAL_POSITION_INT':
            self.latitude          = msg.lat / 1e7
            self.longitude         = msg.lon / 1e7
            self.altitude_amsl     = msg.alt / 1000.0
            self.altitude_relative = msg.relative_alt / 1000.0
            self.heading           = msg.hdg / 100.0

        elif t == 'GPS_RAW_INT':
            self.gps_fix        = msg.fix_type
            self.num_satellites = msg.satellites_visible

        elif t == 'SYS_STATUS':
            self.battery_voltage   = msg.voltage_battery / 1000.0
            self.battery_remaining = msg.battery_remaining

        elif t == 'COMMAND_ACK':
            with self._cmd_ack_lock:
                self._cmd_acks[msg.command] = {
                    'result':    msg.result,
                    'component': msg.get_srcComponent(),
                    'ts':        time.time(),
                }
            if self.debug:
                rs = MAV_RESULT_STR.get(msg.result, str(msg.result))
                print(f"[DEBUG] COMMAND_ACK cmd={msg.command} result={rs} comp={msg.get_srcComponent()}")

        elif t in ('MISSION_REQUEST', 'MISSION_REQUEST_INT'):
            with self._mission_lock:
                self._mission_request_q.append(msg.seq)
            if self.debug:
                print(f"[DEBUG] MISSION_REQUEST seq={msg.seq}")

        elif t == 'MISSION_ACK':
            with self._mission_lock:
                self._mission_ack = {'result': msg.type, 'ts': time.time()}
            if self.debug:
                rs = MAV_MISSION_STR.get(msg.type, str(msg.type))
                print(f"[DEBUG] MISSION_ACK result={rs}")

        elif t == 'MISSION_COUNT':
            with self._mission_lock:
                self._mission_count_recv = msg.count
            if self.debug:
                print(f"[DEBUG] MISSION_COUNT count={msg.count}")

    def _decode_mode(self, custom_mode):
        main = (custom_mode >> 16) & 0xFF
        sub  = (custom_mode >> 24) & 0xFF
        name = PX4_MAIN_MODES.get(main, f'UNKNOWN({main})')
        if main == 4:
            sub_name = PX4_AUTO_SUB.get(sub, '')
            if sub_name:
                name = f'AUTO.{sub_name}'
        return name

    # -----------------------------------------------------------------------
    # COMMAND_LONG with proper retry + ACK (like QGroundControl MavCommandQueue)
    # -----------------------------------------------------------------------
    def _send_command_long(self, command, p1=0, p2=0, p3=0, p4=0,
                           p5=0, p6=0, p7=0, retries=3):
        """
        Send COMMAND_LONG and wait for COMMAND_ACK with retries.
        Returns True on ACCEPTED, False otherwise.
        """
        C = Colors
        tgt_sys  = self.master.target_system
        tgt_comp = 1  # always autopilot component

        if self.debug:
            print(f"[DEBUG] COMMAND_LONG cmd={command} p1={p1} p2={p2} p3={p3} "
                  f"p4={p4} p5={p5} p6={p6} p7={p7}")

        for attempt in range(retries):
            # Clear any stale ACK for this command
            with self._cmd_ack_lock:
                self._cmd_acks.pop(command, None)

            # Send with confirmation = attempt number (QGC does the same)
            self.master.mav.command_long_send(
                tgt_sys, tgt_comp, command, attempt,
                p1, p2, p3, p4, p5, p6, p7)

            # Wait for ACK
            deadline = time.time() + self.ACK_TIMEOUT
            while time.time() < deadline:
                with self._cmd_ack_lock:
                    ack = self._cmd_acks.get(command)
                if ack:
                    with self._cmd_ack_lock:
                        self._cmd_acks.pop(command, None)
                    result = ack['result']
                    if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print(f"{C.OKGREEN}[ACK] Command {command} ACCEPTED{C.ENDC}")
                        return True
                    elif result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                        if self.debug:
                            print(f"[DEBUG] Command {command} IN_PROGRESS, waiting...")
                        # Keep waiting - IN_PROGRESS means it's executing
                        deadline = time.time() + self.ACK_TIMEOUT
                        with self._cmd_ack_lock:
                            self._cmd_acks.pop(command, None)
                        continue
                    else:
                        rs = MAV_RESULT_STR.get(result, str(result))
                        print(f"{C.FAIL}[ACK] Command {command} {rs}{C.ENDC}")
                        return False
                time.sleep(0.02)

            if attempt < retries - 1:
                print(f"{C.WARNING}[RETRY] Command {command} timeout, retry {attempt+1}/{retries-1}...{C.ENDC}")

        print(f"{Colors.FAIL}[ERROR] Command {command} failed after {retries} attempts{Colors.ENDC}")
        return False

    # -----------------------------------------------------------------------
    # MISSION PROTOCOL - Upload (exactly like QGroundControl PlanManager)
    # -----------------------------------------------------------------------
    def _upload_mission(self, items):
        """
        Upload a list of MissionItem dicts to the vehicle using MAVLink mission protocol.
        Follows the exact same state machine as QGroundControl PlanManager::writeMissionItems()

        Each item dict:
          seq          - sequence number (0-based)
          command      - MAV_CMD_*
          frame        - MAV_FRAME_* (default GLOBAL_RELATIVE_ALT)
          param1..7    - command parameters
          lat, lon     - coordinates (degrees)
          alt          - altitude (meters, relative)
          autocontinue - True/False
          current      - True for first item

        PX4 note: home position (seq=0) is NOT sent - PX4 manages its own home.
        """
        C = Colors
        count = len(items)
        if count == 0:
            print(f"{C.FAIL}[ERROR] No mission items to upload{C.ENDC}")
            return False

        print(f"{C.OKCYAN}[MISSION] Uploading {count} items...{C.ENDC}")

        tgt_sys  = self.master.target_system
        tgt_comp = 1

        # Clear stale mission protocol state
        with self._mission_lock:
            self._mission_request_q.clear()
            self._mission_ack = None

        # Step 1: Send MISSION_COUNT
        if self.debug:
            print(f"[DEBUG] Sending MISSION_COUNT={count}")

        for attempt in range(self.MAX_RETRIES):
            with self._mission_lock:
                self._mission_request_q.clear()
                self._mission_ack = None

            self.master.mav.mission_count_send(
                tgt_sys, tgt_comp, count,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

            # Step 2: Respond to each MISSION_REQUEST from vehicle
            items_sent = set()
            deadline   = time.time() + self.MISSION_ACK_TIMEOUT
            success    = False

            while time.time() < deadline:
                # Check for MISSION_ACK (final confirmation)
                with self._mission_lock:
                    ack = self._mission_ack

                if ack:
                    if ack['result'] == 0:  # MAV_MISSION_ACCEPTED
                        print(f"{C.OKGREEN}[MISSION] Upload complete! All {count} items accepted.{C.ENDC}")
                        success = True
                    else:
                        rs = MAV_MISSION_STR.get(ack['result'], str(ack['result']))
                        print(f"{C.FAIL}[MISSION] Upload failed: {rs}{C.ENDC}")
                    break

                # Check for MISSION_REQUEST
                with self._mission_lock:
                    reqs = list(self._mission_request_q)
                    self._mission_request_q.clear()

                for seq in reqs:
                    if seq >= count:
                        print(f"{C.FAIL}[MISSION] Vehicle requested seq={seq} but we only have {count} items{C.ENDC}")
                        return False

                    item = items[seq]
                    lat_int = int(item.get('lat', 0) * 1e7)
                    lon_int = int(item.get('lon', 0) * 1e7)
                    alt     = float(item.get('alt', 0))
                    frame   = item.get('frame', mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)

                    if self.debug:
                        print(f"[DEBUG] Sending MISSION_ITEM_INT seq={seq} "
                              f"cmd={item['command']} lat={lat_int} lon={lon_int} alt={alt}")

                    self.master.mav.mission_item_int_send(
                        tgt_sys, tgt_comp,
                        seq,
                        frame,
                        item['command'],
                        1 if item.get('current', seq == 0) else 0,
                        1 if item.get('autocontinue', True) else 0,
                        float(item.get('param1', 0)),
                        float(item.get('param2', 0)),
                        float(item.get('param3', 0)),
                        float(item.get('param4', 0)),
                        lat_int,
                        lon_int,
                        alt,
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

                    items_sent.add(seq)
                    # Reset deadline after each item sent
                    deadline = time.time() + self.MISSION_ITEM_TIMEOUT

                time.sleep(0.02)

            if success:
                return True

            if attempt < self.MAX_RETRIES - 1:
                print(f"{C.WARNING}[MISSION] Retry {attempt+1}/{self.MAX_RETRIES-1}...{C.ENDC}")

        print(f"{Colors.FAIL}[MISSION] Upload failed after {self.MAX_RETRIES} attempts{Colors.ENDC}")
        return False

    def _start_mission(self):
        """Switch to AUTO.MISSION mode and start executing uploaded mission"""
        C = Colors
        print(f"{C.OKCYAN}[MISSION] Starting mission execution...{C.ENDC}")

        # Set AUTO mode (PX4 custom mode 4<<16 with sub-mode MISSION = 4<<24)
        AUTO_MISSION = (4 << 16) | (4 << 24)
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            AUTO_MISSION)

        time.sleep(0.5)

        # Also send MAV_CMD_MISSION_START
        result = self._send_command_long(
            mavutil.mavlink.MAV_CMD_MISSION_START,
            p1=0, p2=0)

        if result:
            print(f"{C.OKGREEN}[MISSION] Mission started!{C.ENDC}")
        return result

    # -----------------------------------------------------------------------
    # Helper: position calculation
    # -----------------------------------------------------------------------
    def _offset_position(self, lat, lon, bearing_deg, distance_m):
        """
        Calculate new lat/lon from a starting point, bearing and distance.
        Uses proper spherical Earth formula.
        """
        R = 6371000.0
        b = math.radians(bearing_deg)
        lat1 = math.radians(lat)
        lon1 = math.radians(lon)
        lat2 = math.asin(math.sin(lat1) * math.cos(distance_m / R) +
                         math.cos(lat1) * math.sin(distance_m / R) * math.cos(b))
        lon2 = lon1 + math.atan2(
            math.sin(b) * math.sin(distance_m / R) * math.cos(lat1),
            math.cos(distance_m / R) - math.sin(lat1) * math.sin(lat2))
        return math.degrees(lat2), math.degrees(lon2)

    def _make_waypoint(self, seq, lat, lon, alt, current=False,
                       hold=0, accept_radius=2, pass_radius=0, yaw=float('nan')):
        """Create a standard NAV_WAYPOINT mission item dict"""
        return {
            'seq':         seq,
            'command':     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'frame':       mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'param1':      hold,           # hold time (s)
            'param2':      accept_radius,  # acceptance radius (m)
            'param3':      pass_radius,    # pass-through radius
            'param4':      yaw,            # yaw (NaN = unchanged)
            'lat':         lat,
            'lon':         lon,
            'alt':         alt,
            'autocontinue': True,
            'current':     current,
        }

    def _wait_arrival(self, target_lat, target_lon, tolerance_m=3.0, timeout=60):
        """
        Wait until drone is within tolerance_m of target position.
        Returns True when arrived, False on timeout.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            dlat = (self.latitude  - target_lat)  * 111320
            dlon = (self.longitude - target_lon)  * 111320 * math.cos(math.radians(self.latitude))
            dist = math.sqrt(dlat**2 + dlon**2)
            if self.debug:
                print(f"[DEBUG] Distance to waypoint: {dist:.1f}m", end='\r')
            if dist <= tolerance_m:
                print()
                return True
            time.sleep(0.5)
        print()
        return False

    # -----------------------------------------------------------------------
    # STATUS
    # -----------------------------------------------------------------------
    def get_status(self):
        C = Colors
        print(f"\n{C.BOLD}{'='*60}")
        print("DRONE STATUS")
        print(f"{'='*60}{C.ENDC}")
        cc = C.OKGREEN if self.connected else C.FAIL
        ac = C.WARNING if self.armed else C.OKGREEN
        gc = C.OKGREEN if self.gps_fix >= 3 else C.WARNING
        bc = C.OKGREEN if self.battery_remaining > 30 else (C.FAIL if self.battery_remaining < 20 else C.WARNING)
        print(f"Connected:  {cc}{'YES' if self.connected else 'NO'}{C.ENDC}")
        print(f"Armed:      {ac}{'YES' if self.armed else 'NO'}{C.ENDC}")
        print(f"Mode:       {C.OKCYAN}{self.mode}{C.ENDC}")
        print(f"GPS Fix:    {gc}{self.gps_fix} ({self.num_satellites} sats){C.ENDC}")
        print(f"Position:   {self.latitude:.6f}, {self.longitude:.6f}")
        print(f"Altitude:   {self.altitude_relative:.2f}m (rel)  {self.altitude_amsl:.2f}m (AMSL)")
        print(f"Heading:    {self.heading:.1f}°")
        print(f"Battery:    {bc}{self.battery_voltage:.2f}V ({self.battery_remaining}%){C.ENDC}")
        print(f"{C.BOLD}{'='*60}{C.ENDC}\n")

    # -----------------------------------------------------------------------
    # BASIC COMMANDS (arm/disarm/takeoff/land/rtl use COMMAND_LONG - correct)
    # -----------------------------------------------------------------------
    def arm(self):
        C = Colors
        print(f"{C.OKCYAN}[CMD] Arming...{C.ENDC}")
        if self.gps_fix < 3:
            print(f"{C.WARNING}[WARNING] GPS fix={self.gps_fix} (need >=3){C.ENDC}")
        ok = self._send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            p1=1, p2=0)
        if ok:
            time.sleep(1)
            print(f"{C.OKGREEN}[SUCCESS] Armed!{C.ENDC}" if self.armed
                  else f"{C.WARNING}[WARNING] ACK received but not armed yet - check prearm{C.ENDC}")
        return ok

    def disarm(self):
        C = Colors
        print(f"{C.OKCYAN}[CMD] Disarming...{C.ENDC}")
        ok = self._send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            p1=0, p2=0)
        if ok:
            time.sleep(0.5)
            print(f"{C.OKGREEN}[SUCCESS] Disarmed!{C.ENDC}")
        return ok

    def takeoff(self, altitude_m=10.0):
        C = Colors
        print(f"{C.OKCYAN}[CMD] Takeoff to {altitude_m}m...{C.ENDC}")
        if self.altitude_amsl == 0.0:
            print(f"{C.FAIL}[ERROR] Position unknown, cannot takeoff{C.ENDC}")
            return False
        # PX4 uses AMSL for takeoff altitude
        target_amsl = self.altitude_amsl + altitude_m
        if self.debug:
            print(f"[DEBUG] Current AMSL={self.altitude_amsl:.2f}m  Target AMSL={target_amsl:.2f}m")
        ok = self._send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            p1=-1, p2=0, p3=0, p4=float('nan'),
            p5=float('nan'), p6=float('nan'), p7=target_amsl)
        if ok:
            print(f"{C.OKGREEN}[SUCCESS] Takeoff command sent! Climbing to {altitude_m}m...{C.ENDC}")
        return ok

    def land(self):
        C = Colors
        print(f"{C.OKCYAN}[CMD] Landing...{C.ENDC}")
        ok = self._send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            p1=0, p2=0, p3=0, p4=float('nan'),
            p5=float('nan'), p6=float('nan'), p7=0)
        if ok:
            print(f"{C.OKGREEN}[SUCCESS] Land command sent!{C.ENDC}")
        return ok

    def rtl(self):
        C = Colors
        print(f"{C.OKCYAN}[CMD] Return to launch...{C.ENDC}")
        ok = self._send_command_long(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
        if ok:
            print(f"{C.OKGREEN}[SUCCESS] RTL command sent!{C.ENDC}")
        return ok

    def set_mode(self, mode_name):
        C = Colors
        mode_map = {
            'MANUAL':     1 << 16,
            'ALTCTL':     2 << 16,
            'POSCTL':     3 << 16,
            'AUTO':       4 << 16,
            'ACRO':       5 << 16,
            'OFFBOARD':   6 << 16,
            'STABILIZED': 7 << 16,
        }
        m = mode_name.upper()
        if m not in mode_map:
            print(f"{C.FAIL}[ERROR] Unknown mode: {mode_name}. Options: {list(mode_map)}{C.ENDC}")
            return False
        print(f"{C.OKCYAN}[CMD] Setting mode {m}...{C.ENDC}")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_map[m])
        time.sleep(0.5)
        print(f"{C.OKGREEN}[SUCCESS] Mode change sent{C.ENDC}")
        return True

    def emergency_stop(self):
        C = Colors
        print(f"{C.FAIL}[WARNING] EMERGENCY STOP - MOTORS WILL CUT!{C.ENDC}")
        resp = input("Type YES to confirm: ")
        if resp.strip() != 'YES':
            print("[INFO] Cancelled")
            return False
        return self._send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            p1=0, p2=21196)  # 21196 = force disarm magic

    # -----------------------------------------------------------------------
    # MOVEMENT COMMANDS - all use waypoint mission upload (like QGroundControl)
    # -----------------------------------------------------------------------
    def goto_location(self, lat, lon, alt_relative):
        """
        Go to a specific GPS location using mission upload.
        This is how QGroundControl does guided goto - uploads a 1-item mission.
        """
        C = Colors
        print(f"{C.OKCYAN}[CMD] Goto {lat:.6f}, {lon:.6f} at {alt_relative}m...{C.ENDC}")

        items = [self._make_waypoint(0, lat, lon, alt_relative, current=True)]
        if not self._upload_mission(items):
            return False
        return self._start_mission()

    def forward(self, distance_m):
        """Move forward by distance_m in current heading direction"""
        C = Colors
        print(f"{C.OKCYAN}[CMD] Forward {distance_m}m (heading={self.heading:.1f}°)...{C.ENDC}")

        if self.latitude == 0.0 and self.longitude == 0.0:
            print(f"{C.FAIL}[ERROR] Position unknown{C.ENDC}")
            return False

        new_lat, new_lon = self._offset_position(
            self.latitude, self.longitude, self.heading, distance_m)

        if self.debug:
            print(f"[DEBUG] From: {self.latitude:.6f},{self.longitude:.6f}")
            print(f"[DEBUG] To:   {new_lat:.6f},{new_lon:.6f}")

        return self.goto_location(new_lat, new_lon, self.altitude_relative)

    def circle(self, radius_m):
        """
        Orbit in a circle using MAV_CMD_DO_ORBIT via COMMAND_INT.
        Uses relative altitude to prevent helix/spiral.
        """
        C = Colors
        print(f"{C.OKCYAN}[CMD] Circle radius={radius_m}m at {self.altitude_relative:.1f}m...{C.ENDC}")

        if self.latitude == 0.0 and self.longitude == 0.0:
            print(f"{C.FAIL}[ERROR] Position unknown{C.ENDC}")
            return False

        MAV_CMD_DO_ORBIT = 34
        alt_rel = self.altitude_relative

        if self.debug:
            print(f"[DEBUG] COMMAND_INT DO_ORBIT radius={radius_m} "
                  f"frame=GLOBAL_RELATIVE_ALT alt={alt_rel:.2f}m yaw=1(face_center)")

        # Clear stale ACK
        with self._cmd_ack_lock:
            self._cmd_acks.pop(MAV_CMD_DO_ORBIT, None)

        self.master.mav.command_int_send(
            self.master.target_system,
            1,  # autopilot component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            MAV_CMD_DO_ORBIT,
            0, 0,
            radius_m,        # param1: radius
            float('nan'),    # param2: velocity (NaN = default)
            1,               # param3: yaw behavior = 1 (face center, prevents helix)
            float('nan'),    # param4: orbits (NaN = infinite)
            int(self.latitude  * 1e7),
            int(self.longitude * 1e7),
            alt_rel)         # RELATIVE altitude - prevents helix!

        # Wait for ACK
        deadline = time.time() + self.ACK_TIMEOUT
        while time.time() < deadline:
            with self._cmd_ack_lock:
                ack = self._cmd_acks.get(MAV_CMD_DO_ORBIT)
            if ack:
                with self._cmd_ack_lock:
                    self._cmd_acks.pop(MAV_CMD_DO_ORBIT, None)
                if ack['result'] == 0:
                    print(f"{C.OKGREEN}[SUCCESS] Circle started at {radius_m}m radius, {alt_rel:.1f}m altitude{C.ENDC}")
                    print(f"{C.WARNING}[TIP] Use 'land' or 'mode POSCTL' to stop{C.ENDC}")
                    return True
                else:
                    rs = MAV_RESULT_STR.get(ack['result'], str(ack['result']))
                    print(f"{C.FAIL}[ERROR] Circle command {rs}{C.ENDC}")
                    return False
            time.sleep(0.02)

        print(f"{C.FAIL}[ERROR] Circle command timeout{C.ENDC}")
        return False

    # -----------------------------------------------------------------------
    # GEOMETRIC PATTERNS - all use waypoint mission upload
    # -----------------------------------------------------------------------
    def _fly_pattern(self, waypoints_latlon, alt, label):
        """
        Upload and execute a list of (lat, lon) waypoints as a mission.
        Waits for each waypoint to be reached before reporting progress.
        """
        C = Colors
        n = len(waypoints_latlon)
        print(f"{C.OKCYAN}[PATTERN] {label}: {n} waypoints at {alt:.1f}m{C.ENDC}")

        items = []
        for i, (lat, lon) in enumerate(waypoints_latlon):
            items.append(self._make_waypoint(
                seq=i, lat=lat, lon=lon, alt=alt,
                current=(i == 0), hold=1, accept_radius=2))

        if not self._upload_mission(items):
            print(f"{C.FAIL}[ERROR] Mission upload failed{C.ENDC}")
            return False

        if not self._start_mission():
            print(f"{C.FAIL}[ERROR] Mission start failed{C.ENDC}")
            return False

        # Wait for each waypoint
        for i, (lat, lon) in enumerate(waypoints_latlon):
            print(f"{C.OKCYAN}[PATTERN] Flying to waypoint {i+1}/{n}...{C.ENDC}")
            arrived = self._wait_arrival(lat, lon, tolerance_m=3.0, timeout=120)
            if arrived:
                print(f"{C.OKGREEN}[PATTERN] Waypoint {i+1}/{n} reached{C.ENDC}")
            else:
                print(f"{C.WARNING}[PATTERN] Waypoint {i+1}/{n} timeout - continuing{C.ENDC}")

        print(f"{C.OKGREEN}[PATTERN] {label} complete!{C.ENDC}")
        return True

    def square(self, side_m=10.0):
        """Fly a square pattern using waypoints"""
        C = Colors
        print(f"{C.OKCYAN}[CMD] Square {side_m}m sides...{C.ENDC}")
        if self.latitude == 0.0:
            print(f"{C.FAIL}[ERROR] Position unknown{C.ENDC}")
            return False

        alt = self.altitude_relative
        h   = self.heading
        lat, lon = self.latitude, self.longitude

        # 4 corners: forward, right 90, forward, right 90, forward, right 90, forward (back)
        corners = []
        cur_lat, cur_lon = lat, lon
        cur_heading = h
        for _ in range(4):
            cur_lat, cur_lon = self._offset_position(cur_lat, cur_lon, cur_heading, side_m)
            corners.append((cur_lat, cur_lon))
            cur_heading = (cur_heading + 90) % 360

        if self.debug:
            for i, (la, lo) in enumerate(corners):
                print(f"[DEBUG] Corner {i+1}: {la:.6f},{lo:.6f}")

        return self._fly_pattern(corners, alt, f"Square({side_m}m)")

    def rectangle(self, length_m=15.0, width_m=10.0):
        """Fly a rectangle pattern using waypoints"""
        C = Colors
        print(f"{C.OKCYAN}[CMD] Rectangle {length_m}m x {width_m}m...{C.ENDC}")
        if self.latitude == 0.0:
            print(f"{C.FAIL}[ERROR] Position unknown{C.ENDC}")
            return False

        alt = self.altitude_relative
        h   = self.heading
        cur_lat, cur_lon = self.latitude, self.longitude
        corners = []

        sides = [length_m, width_m, length_m, width_m]
        cur_heading = h
        for side in sides:
            cur_lat, cur_lon = self._offset_position(cur_lat, cur_lon, cur_heading, side)
            corners.append((cur_lat, cur_lon))
            cur_heading = (cur_heading + 90) % 360

        return self._fly_pattern(corners, alt, f"Rectangle({length_m}x{width_m}m)")

    def triangle(self, side_m=10.0):
        """Fly an equilateral triangle using waypoints"""
        C = Colors
        print(f"{C.OKCYAN}[CMD] Equilateral triangle {side_m}m sides...{C.ENDC}")
        if self.latitude == 0.0:
            print(f"{C.FAIL}[ERROR] Position unknown{C.ENDC}")
            return False

        alt = self.altitude_relative
        cur_lat, cur_lon = self.latitude, self.longitude
        cur_heading = self.heading
        corners = []

        for _ in range(3):
            cur_lat, cur_lon = self._offset_position(cur_lat, cur_lon, cur_heading, side_m)
            corners.append((cur_lat, cur_lon))
            cur_heading = (cur_heading + 120) % 360  # exterior angle = 120°

        return self._fly_pattern(corners, alt, f"Triangle({side_m}m)")

    def right_triangle(self, base_m=10.0, height_m=10.0):
        """Fly a right-angled triangle using waypoints"""
        C = Colors
        print(f"{C.OKCYAN}[CMD] Right triangle base={base_m}m height={height_m}m...{C.ENDC}")
        if self.latitude == 0.0:
            print(f"{C.FAIL}[ERROR] Position unknown{C.ENDC}")
            return False

        alt = self.altitude_relative
        h   = self.heading
        hyp = math.sqrt(base_m**2 + height_m**2)
        # angle at start corner (between base and hypotenuse)
        alpha = math.degrees(math.atan2(height_m, base_m))

        if self.debug:
            print(f"[DEBUG] Hypotenuse={hyp:.2f}m  alpha={alpha:.1f}°")

        # Point A = start (current position)
        # Point B = after flying base forward
        # Point C = after turning 90° left and flying height
        # Back to A via hypotenuse
        lat_a, lon_a = self.latitude, self.longitude
        lat_b, lon_b = self._offset_position(lat_a, lon_a, h, base_m)
        lat_c, lon_c = self._offset_position(lat_b, lon_b, (h - 90) % 360, height_m)

        corners = [(lat_b, lon_b), (lat_c, lon_c), (lat_a, lon_a)]
        return self._fly_pattern(corners, alt, f"RightTriangle({base_m}x{height_m}m)")


# ===========================================================================
# Help text
# ===========================================================================
def print_help():
    C = Colors
    print(f"\n{C.BOLD}{'='*62}")
    print("  DRONE CONTROLLER - COMMANDS")
    print(f"{'='*62}{C.ENDC}")
    print(f"{C.OKGREEN}Basic:{C.ENDC}")
    print("  status                     - Show drone status")
    print("  arm                        - Arm motors")
    print("  disarm                     - Disarm motors")
    print("  takeoff [alt]              - Takeoff (default 10m)")
    print("  land                       - Land now")
    print("  rtl                        - Return to launch")
    print()
    print(f"{C.OKGREEN}Movement (waypoint-based, reliable):{C.ENDC}")
    print("  forward [dist]             - Move forward Xm (default 5m)")
    print("  f[n]                       - Forward shortcut e.g. f5 f10 f3")
    print("  goto [lat] [lon] [alt]     - Fly to GPS coordinate")
    print("  circle [radius]            - Orbit circle (default 5m)")
    print()
    print(f"{C.OKGREEN}Patterns (full waypoint mission upload):{C.ENDC}")
    print("  square [side]              - Square pattern (default 10m)")
    print("  rectangle [len] [width]    - Rectangle (default 15x10m)")
    print("  triangle [side]            - Equilateral triangle (default 10m)")
    print("  righttriangle [base] [h]   - Right triangle (default 10x10m)")
    print()
    print(f"{C.OKGREEN}Advanced:{C.ENDC}")
    print("  mode [MODE]                - Set mode (POSCTL/AUTO/MANUAL etc)")
    print("  emergency                  - Kill motors NOW (drone will crash!)")
    print("  clear                      - Clear screen")
    print("  help                       - This help")
    print("  exit / quit                - Exit")
    print(f"{C.BOLD}{'='*62}{C.ENDC}\n")


# ===========================================================================
# COM port listing
# ===========================================================================
def list_com_ports():
    if not WINDOWS_SERIAL_AVAILABLE:
        print("[ERROR] pyserial not installed: pip install pyserial")
        return
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("[WARNING] No COM ports found")
        return
    print("\nAvailable COM ports:")
    for p in ports:
        print(f"  {p.device}  -  {p.description}")
    print(f"\nUsage: python drone_controller.py {ports[0].device}\n")


# ===========================================================================
# Main
# ===========================================================================
def main():
    parser = argparse.ArgumentParser(description='PX4 Drone Controller')
    parser.add_argument('connection', nargs='?',
                        help='Connection string e.g. COM3 or udp:127.0.0.1:14550')
    parser.add_argument('--baud', type=int, default=57600)
    parser.add_argument('--no-debug', action='store_true')
    parser.add_argument('--no-color', action='store_true')
    parser.add_argument('--list-ports', action='store_true')
    args = parser.parse_args()

    if args.no_color:
        Colors.disable()

    if args.list_ports:
        list_com_ports()
        return 0

    if not args.connection:
        print("[ERROR] No connection string. Example: python drone_controller.py COM3")
        parser.print_help()
        return 1

    C = Colors
    print(f"{C.BOLD}{C.OKCYAN}")
    print("=" * 62)
    print("  DRONE CONTROLLER  -  PX4 Pixhawk 6C")
    print("  Waypoint-based mission protocol (like QGroundControl)")
    print("=" * 62)
    print(C.ENDC)

    ctrl = DroneController(args.connection, args.baud)
    ctrl.debug = not args.no_debug

    if not ctrl.connect():
        print(f"{C.FAIL}[ERROR] Connection failed{C.ENDC}")
        return 1

    print(f"{C.OKCYAN}[INFO] Waiting for telemetry...{C.ENDC}")
    time.sleep(2)
    ctrl.get_status()
    print_help()
    print(f"{C.OKGREEN}[INFO] Ready. Type 'help' for commands.{C.ENDC}\n")

    try:
        while True:
            try:
                raw = input("drone> ").strip()
                if not raw:
                    continue

                parts = raw.split()
                cmd   = parts[0].lower()
                cargs = parts[1:]

                if cmd in ('exit', 'quit'):
                    break

                elif cmd == 'help':
                    print_help()

                elif cmd == 'status':
                    ctrl.get_status()

                elif cmd == 'arm':
                    ctrl.arm()

                elif cmd == 'disarm':
                    ctrl.disarm()

                elif cmd == 'takeoff':
                    alt = float(cargs[0]) if cargs else 10.0
                    ctrl.takeoff(alt)

                elif cmd == 'land':
                    ctrl.land()

                elif cmd == 'rtl':
                    ctrl.rtl()

                elif cmd == 'forward':
                    d = float(cargs[0]) if cargs else 5.0
                    ctrl.forward(d)

                # shortcut: f5, f10, f3 etc.
                elif cmd.startswith('f') and cmd[1:].replace('.','',1).isdigit():
                    ctrl.forward(float(cmd[1:]))

                elif cmd == 'goto':
                    if len(cargs) < 3:
                        print("[ERROR] Usage: goto <lat> <lon> <alt>")
                    else:
                        ctrl.goto_location(float(cargs[0]), float(cargs[1]), float(cargs[2]))

                elif cmd == 'circle':
                    r = float(cargs[0]) if cargs else 5.0
                    ctrl.circle(r)

                elif cmd == 'square':
                    s = float(cargs[0]) if cargs else 10.0
                    ctrl.square(s)

                elif cmd == 'rectangle':
                    l = float(cargs[0]) if cargs else 15.0
                    w = float(cargs[1]) if len(cargs) > 1 else 10.0
                    ctrl.rectangle(l, w)

                elif cmd == 'triangle':
                    s = float(cargs[0]) if cargs else 10.0
                    ctrl.triangle(s)

                elif cmd in ('righttriangle', 'rt'):
                    b = float(cargs[0]) if cargs else 10.0
                    h = float(cargs[1]) if len(cargs) > 1 else 10.0
                    ctrl.right_triangle(b, h)

                elif cmd == 'mode':
                    if not cargs:
                        print("[ERROR] Usage: mode <POSCTL|AUTO|MANUAL|ALTCTL|STABILIZED>")
                    else:
                        ctrl.set_mode(cargs[0])

                elif cmd == 'emergency':
                    ctrl.emergency_stop()

                elif cmd in ('clear', 'cls'):
                    os.system('cls' if sys.platform == 'win32' else 'clear')
                    ctrl.get_status()

                else:
                    print(f"{Colors.FAIL}[ERROR] Unknown command: {cmd}{Colors.ENDC}")
                    print("[INFO] Type 'help' for available commands")

            except KeyboardInterrupt:
                print(f"\n{Colors.WARNING}[INFO] Ctrl+C - type 'exit' to quit or keep flying{Colors.ENDC}")
            except EOFError:
                break
            except Exception as e:
                print(f"{Colors.FAIL}[ERROR] {e}{Colors.ENDC}")

    finally:
        ctrl.close()

    print(f"{Colors.OKGREEN}[INFO] Goodbye!{Colors.ENDC}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
