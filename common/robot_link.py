"""Link to the Arduino (arduino/car/car.ino): the USB cable, or WiFi through the ESP32-CAM."""
import socket
import threading
from collections import deque
import time
import urllib.parse
import serial
from serial.tools import list_ports


# The sonar "radar" directions (car.ino), and their angles in degrees from straight ahead (+ = left).
DIRS = ("LL", "L", "C", "R", "RR")
DEFAULT_ANGLES = {"LL": 70, "L": 35, "C": 0, "R": -35, "RR": -70}


def scan_angles():
    import config
    a = dict(DEFAULT_ANGLES)
    a.update(getattr(config, "SCAN_ANGLES", {}))
    return a


def parse(text):
    """One line from the Arduino -> a tuple, or None if it isn't a known message."""
    parts = text.strip().split(",")
    try:
        if parts[0] == "READY":
            return ("READY",)
        if parts[0] == "H":                       # heartbeat; new car.ino adds the lowest 5 V in mV
            return ("H", int(parts[1])) if len(parts) == 2 and parts[1] else ("H", 0)
        if parts[0] == "STUCK":
            return ("STUCK",)
        if parts[0] == "TRAP" and len(parts) == 2:
            return ("TRAP", parts[1])
        if parts[0] == "MODE" and len(parts) == 2:
            return ("MODE", int(parts[1]))
        if parts[0] == "P" and len(parts) == 4 and parts[1] in DIRS:
            return ("P", parts[1], int(parts[2]), int(parts[3]))
        if parts[0] == "W" and len(parts) == 3:
            return ("W", int(parts[1]), int(parts[2]))
    except ValueError:
        return None
    return None


def _score(p, kind):
    text = f"{p.device} {p.description} {p.manufacturer or ''}".lower()
    if "bluetooth" in text or "debug-console" in text:
        return 0
    arduino = p.vid in (0x2341, 0x2A03) or "arduino" in text or "usbmodem" in p.device
    usb_serial = p.vid in (0x1A86, 0x10C4, 0x0403) or "usbserial" in p.device or "wchusbserial" in p.device
    if kind == "uno":
        return 3 if arduino else (1 if usb_serial else 0)
    return 3 if usb_serial and not arduino else 0      # kind == "esp32"


def find_ports(kind="uno"):
    ports = [p for p in list_ports.comports() if _score(p, kind) > 0]
    ports.sort(key=lambda p: _score(p, kind), reverse=True)
    return [p.device for p in ports]


class WifiPort:
    """A TCP connection to the ESP32-CAM bridge (port 82). Behaves like the USB serial port."""
    def __init__(self, host, port=82):
        self.s = socket.create_connection((host, port), timeout=2)
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.s.setblocking(False)

    def read_all(self):
        data = b""
        while True:
            try:
                chunk = self.s.recv(4096)
            except (BlockingIOError, InterruptedError):
                return data
            if not chunk:
                raise ConnectionError("the car closed the WiFi link")
            data += chunk

    def write(self, data):
        self.s.sendall(data)

    def close(self):
        self.s.close()


def wifi_host():
    """The ESP32-CAM's address: the host of config.CAMERA, or its own WiFi (192.168.4.1)."""
    import config
    cam = str(config.CAMERA)
    if cam.startswith("http"):
        return urllib.parse.urlparse(cam).hostname
    return "192.168.4.1"


class RobotLink:
    def __init__(self, port="auto"):
        self.wifi = (port == "wifi")
        if self.wifi:
            self.host = wifi_host()
            port = f"WiFi {self.host}:82"
        elif not port or port == "auto":
            found = find_ports("uno")
            if not found:
                raise RuntimeError("No Arduino found. Plug in the USB cable. Close the Arduino IDE / Serial Monitor.\n"
                                   "(Driving without the cable? Set PORT = \"wifi\" in config.py)")
            port = found[0]
        self.port = port
        self.buf = b""
        self.ready = False
        self.lost = False
        self.why = ""                     # why the link is down (shown in the window)
        self.closing = False
        self.lock = threading.Lock()
        self.events = deque()             # "LINK LOST", "LINK BACK", "ARDUINO RESTARTED" (shown + logged)
        self.last_rx = time.time()
        self.has_beat = False             # True once the Arduino sends heartbeats (new car.ino)
        self.vcc_mv = 0                   # lowest 5 V rail voltage in the last half second (0 = unknown)
        self.current_mode = 0
        if self.wifi:
            # the camera needs ~10 s after power-on to join the WiFi: keep trying for 30 s
            start, err = time.time(), None
            while True:
                try:
                    self.ser = WifiPort(self.host)
                    break
                except OSError as e:
                    err = e
                    if time.time() - start > 30:
                        raise RuntimeError(f"Cannot reach the car at {self.host}:82 ({err}).\n"
                                           "1) Safari: open http://" + self.host + " . No picture = the camera is off or "
                                           "not on the WiFi (press its RST button, check its 5V).\n"
                                           "2) Picture but still this error: in the terminal run   nc -vz " + self.host +
                                           " 82   and send the result.\n"
                                           "3) Is the Mac on the same WiFi as the camera?")
                    print(f"waiting for the car at {self.host}:82 ... ({int(time.time() - start)} s)", flush=True)
                    time.sleep(1.5)
        else:
            self.ser = serial.Serial(port, 115200, timeout=0)
        start, asked = time.time(), 0.0
        while time.time() - start < 5 and not self.ready:
            if self.wifi and time.time() - asked > 0.5:     # over WiFi the Uno does not restart: ask it
                self.send("MODE,0")
                asked = time.time()
            self.ready = any(m[0] in ("READY", "MODE") for m in self.poll())
            time.sleep(0.05)

    # ---- low level: USB or WiFi ----
    def _read(self):
        if self.wifi:
            return self.ser.read_all()
        n = self.ser.in_waiting
        return self.ser.read(n) if n else b""

    def _reconnect_loop(self):
        """Background thread: the link dropped. Try again every second without freezing the window.
        Meanwhile the Arduino has stopped the motors by itself (its watchdog)."""
        while self.lost and not self.closing:
            try:
                port = WifiPort(self.host)
            except OSError:
                self.why = "the car's WiFi is not reachable (camera off? Mac on another WiFi?)"
                time.sleep(1.0)
                continue
            data = b""
            try:                                  # the link only counts as back when the Arduino answers
                port.write(f"MODE,{self.current_mode}\n".encode())
                t0 = time.time()
                while not data and time.time() - t0 < 2.0 and not self.closing:
                    time.sleep(0.05)
                    data = port.read_all()
            except OSError:
                data = b""
            if not data:
                self.why = "WiFi OK, but the Arduino is silent (Arduino power? the D1 -> IO13 wire?)"
                port.close()
                time.sleep(1.0)
                continue
            with self.lock:
                self.ser, self.buf, self.last_rx, self.lost = port, data, time.time(), False
            self.events.append("LINK BACK")
            return

    def _link_lost(self):
        if self.lost:
            return
        self.lost = True
        self.why = "reconnecting..."
        self.events.append("LINK LOST")
        try:
            self.ser.close()
        except Exception:
            pass
        threading.Thread(target=self._reconnect_loop, daemon=True).start()

    def poll(self):
        if self.lost:
            return []
        try:
            with self.lock:
                data = self._read()
        except (OSError, serial.SerialException):
            if not self.wifi:
                raise
            self._link_lost()
            return []
        now = time.time()
        if data:
            self.last_rx = now
            self.buf += data
        elif self.wifi and self.has_beat and now - self.last_rx > 2.5:
            self._link_lost()                 # no heartbeat for 2.5 s: the connection is dead
            return []
        out = []
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            msg = parse(raw.decode(errors="ignore"))
            if not msg:
                continue
            if msg[0] == "READY" and self.ready:
                # the Arduino restarted in the middle of a run (power dip): put it back in its stage
                self.events.append("ARDUINO RESTARTED")
                self.send(f"MODE,{self.current_mode}")
            if msg[0] == "H":
                self.has_beat = True
                self.vcc_mv = msg[1]
            else:
                out.append(msg)
        if len(self.buf) > 4096:
            self.buf = b""
        return out

    def send(self, text):
        if self.lost:
            return
        try:
            with self.lock:
                self.ser.write((text + "\n").encode())
        except (OSError, serial.SerialException):
            if not self.wifi:
                raise
            self._link_lost()

    def take_events(self):
        ev = []
        while self.events:
            ev.append(self.events.popleft())
        return ev

    def mode(self, n):
        self.current_mode = int(n)
        self.send(f"MODE,{int(n)}")

    def wheels(self, left, right):
        self.send(f"M,{int(left)},{int(right)}")

    def look(self, direction):
        self.send(f"LOOK,{direction}")

    def close(self):
        self.closing = True
        try:
            for _ in range(3):
                self.send("MODE,0")
                time.sleep(0.02)
            self.ser.close()
        except Exception:
            pass
