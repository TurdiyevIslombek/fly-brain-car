"""Camera reader for stage 3. A background thread keeps only the NEWEST picture.
Sources: the ESP32-CAM stream (MJPEG over WiFi), the Mac's webcam, or "test" (a made-up video)."""
import collections
import threading
import time
import urllib.request
import numpy as np
import cv2
import config

ESP32_ADDRESSES = ["http://flycar-cam.local:81/stream", "http://192.168.4.1:81/stream"]


class Camera:
    def __init__(self, source="auto", flip=False, mirror=False):
        self.source, self.flip, self.mirror = source, flip, mirror
        self.lock = threading.Lock()
        self.frame, self.count = None, 0
        self.arrivals = collections.deque()      # arrival times of the frames in the last second
        self._t_last = time.time()
        self._cache = (-1, None)                  # (frame number, converted pictures)
        self.status = "connecting..."
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    @property
    def fps(self):
        """Frames that really arrived in the last 1 second (no averaging tricks)."""
        now = time.time()
        with self.lock:
            while self.arrivals and now - self.arrivals[0] > 1.0:
                self.arrivals.popleft()
            return float(len(self.arrivals))

    def age(self):
        """Seconds since the newest frame arrived. Big = the stream froze."""
        return time.time() - self._t_last

    def latest(self):
        """(rgb picture, small gray picture, frame number, arrival time) of the newest frame, or None."""
        with self.lock:
            if self.frame is None:
                return None
            n, t = self.count, self._t_last
            if self._cache[0] == n:                    # same frame as last tick: no work again
                return self._cache[1]
            f = self.frame.copy()
        if self.flip:
            f = cv2.rotate(f, cv2.ROTATE_180)
        if self.mirror:
            f = cv2.flip(f, 1)
        rgb = cv2.cvtColor(f, cv2.COLOR_BGR2RGB)
        gray = cv2.cvtColor(cv2.resize(f, (config.CAM_W, config.CAM_H)), cv2.COLOR_BGR2GRAY)
        self._cache = (n, (rgb, gray, n, t))
        return rgb, gray, n, t

    def close(self):
        self.running = False
        self.thread.join(timeout=2)

    # ---- background thread ----
    def _store(self, img):
        now = time.time()
        with self.lock:
            self.arrivals.append(now)
            self.frame, self.count, self._t_last = img, self.count + 1, now

    def _run(self):
        self._t_last = time.time()
        src = str(self.source)
        while self.running:
            try:
                if src == "test":
                    self._run_test()
                elif src == "webcam" or src.isdigit():
                    self._run_webcam(0 if src == "webcam" else int(src))
                else:
                    urls = ESP32_ADDRESSES if src == "auto" else [src]
                    for url in urls:
                        if not self.running:
                            break
                        self.status = f"connecting to {url}"
                        try:
                            self._run_mjpeg(url)
                        except Exception as e:
                            self.status = f"no camera at {url} ({type(e).__name__})"
                            time.sleep(0.5)
            except Exception as e:
                self.status = f"camera error: {e}"
            time.sleep(1.0)

    def _run_mjpeg(self, url):
        with urllib.request.urlopen(url, timeout=2) as stream:     # 2 s of silence = reconnect
            self.status = f"camera: {url}"
            buf = b""
            while self.running:
                chunk = stream.read1(16384)        # whatever has arrived (does not wait for more)
                if not chunk:
                    raise ConnectionError("stream ended")
                buf += chunk
                jpgs = []
                while True:                                   # take every complete JPEG out of the buffer
                    start = buf.find(b"\xff\xd8")
                    end = buf.find(b"\xff\xd9", start + 2) if start != -1 else -1
                    if start == -1 or end == -1:
                        break
                    jpgs.append(buf[start:end + 2])
                    buf = buf[end + 2:]
                if jpgs:                     # after a WiFi hiccup several arrive at once:
                    with self.lock:          # count them all, but decode only the newest
                        self.arrivals.extend([time.time()] * (len(jpgs) - 1))
                    img = cv2.imdecode(np.frombuffer(jpgs[-1], np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        self._store(img)
                if len(buf) > 2_000_000:
                    buf = b""

    def _run_webcam(self, index):
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            self.status = "webcam not available (System Settings > Privacy > Camera: allow Terminal / VS Code)"
            return
        self.status = f"camera: webcam {index}"
        try:
            while self.running:
                ok, img = cap.read()
                if ok:
                    self._store(img)
                else:
                    time.sleep(0.01)
        finally:
            cap.release()

    def _run_test(self):
        """A made-up video: a textured box approaches on the left, then on the right, again and again."""
        rng = np.random.default_rng(0)
        blobs = lambda h, w: cv2.normalize(cv2.GaussianBlur(rng.random((h, w)), (9, 9), 0), None, 0, 255,
                                           cv2.NORM_MINMAX).astype(np.uint8)
        H, W = 240, 320
        wall, box = blobs(H, W), blobs(128, 128)
        self.status = "camera: test video (box comes at the left, then the right)"
        i = 0
        while self.running:
            phase, k = divmod(i % 60, 30)
            img = wall.copy()
            if k < 20:
                cx = W // 4 if phase == 0 else 3 * W // 4
                s = 24 + 4 * k
                patch = cv2.resize(box, (2 * s, 2 * s))
                y0, x0 = H // 2 - s, cx - s
                ys, xs = slice(max(y0, 0), min(y0 + 2 * s, H)), slice(max(x0, 0), min(x0 + 2 * s, W))
                img[ys, xs] = patch[ys.start - y0:ys.stop - y0, xs.start - x0:xs.stop - x0]
            self._store(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
            i += 1
            time.sleep(1 / 15)
