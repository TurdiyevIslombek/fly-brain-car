// ESP32-CAM (AI-Thinker): the car's eye AND its radio.
//   http://IP              test page (open it in Safari)
//   http://IP:81/stream    the video the laptop app reads (stage 3)
//   IP:82                  WiFi <-> Arduino bridge: the laptop talks to the Arduino
//                          exactly like over the USB cable (no cable needed)
// WiFi settings: wifi_settings.h (upload_camera.sh writes it).

#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include "wifi_settings.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ---------- WiFi <-> Arduino bridge ----------
#define LINK_RX 13        // <- Arduino D1 (TX), through the resistors (5 V -> 3 V). IO15 also works (auto-detected).
#define LINK_TX 14        // -> Arduino D0 (RX)
#define LINK_PORT 82
#define STREAM_GAP_MS 35   // pause between video frames (about 20 fps max), so the link never waits
HardwareSerial arduino(1);
WiFiServer bridge(LINK_PORT);
WiFiClient laptop;
int linkRx = LINK_RX;                    // the pin the Arduino's wire is really on (13 or 15)
unsigned long fromArduino = 0, toArduino = 0;   // bytes passed through the link (see /status)

int pickRxPin() {
  // The Arduino's TX line rests at about 3 V. A pin with nothing on it reads LOW with a pull-down.
  for (int p : {13, 15}) {
    pinMode(p, INPUT_PULLDOWN);
    delay(5);
    if (digitalRead(p) == HIGH) return p;
  }
  return LINK_RX;                        // nothing found (Arduino off?): use the default
}

// AI-Thinker ESP32-CAM camera pins
#define CAM_PWDN 32
#define CAM_RESET -1
#define CAM_XCLK 0
#define CAM_SIOD 26
#define CAM_SIOC 27
#define CAM_Y9 35
#define CAM_Y8 34
#define CAM_Y7 39
#define CAM_Y6 36
#define CAM_Y5 21
#define CAM_Y4 19
#define CAM_Y3 18
#define CAM_Y2 5
#define CAM_VSYNC 25
#define CAM_HREF 23
#define CAM_PCLK 22
#define FLASH_LED 4

#define BOUNDARY "flycarframe"
static const char *STREAM_TYPE = "multipart/x-mixed-replace;boundary=" BOUNDARY;
static const char *STREAM_SEP = "\r\n--" BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

String address = "";

bool startCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = CAM_Y2;
  c.pin_d1 = CAM_Y3;
  c.pin_d2 = CAM_Y4;
  c.pin_d3 = CAM_Y5;
  c.pin_d4 = CAM_Y6;
  c.pin_d5 = CAM_Y7;
  c.pin_d6 = CAM_Y8;
  c.pin_d7 = CAM_Y9;
  c.pin_xclk = CAM_XCLK;
  c.pin_pclk = CAM_PCLK;
  c.pin_vsync = CAM_VSYNC;
  c.pin_href = CAM_HREF;
  c.pin_sccb_sda = CAM_SIOD;
  c.pin_sccb_scl = CAM_SIOC;
  c.pin_pwdn = CAM_PWDN;
  c.pin_reset = CAM_RESET;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_QVGA;          // 320 x 240
  c.jpeg_quality = 15;                    // a bit smaller pictures = less WiFi traffic (12 = sharper, 20 = smaller)
  c.fb_count = psramFound() ? 2 : 1;
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }
  // Picture tuned for motion vision: more contrast (blank cardboard has almost no detail; the fly
  // brain's looming comes from edges moving apart), a bit darker (white walls in sun burn out),
  // lens correction on (the corners of this cheap lens are dark).
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_contrast(s, 2);        // -2..2
    s->set_ae_level(s, -1);       // -2..2: exposure a bit lower
    s->set_lenc(s, 1);
  }
  return true;
}

static esp_err_t indexHandler(httpd_req_t *req) {
  static const char page[] =
      "<html><head><meta name='viewport' content='width=device-width'></head>"
      "<body style='background:#111;color:#eee;font-family:sans-serif;text-align:center'>"
      "<h3>FlyCar camera</h3><img id='v' style='width:100%;max-width:640px'>"
      "<p>The laptop app reads: <span id='u'></span></p>"
      "<script>var u='http://'+location.hostname+':81/stream';"
      "document.getElementById('v').src=u;document.getElementById('u').textContent=u;</script>"
      "</body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t statusHandler(httpd_req_t *req) {
  char t[300];
  snprintf(t, sizeof(t),
           "Arduino wire found on: IO%d\nbytes FROM the Arduino: %lu\nbytes TO the Arduino: %lu\nlaptop connected: %s\n"
           "\n(refresh this page while the car app runs stage 1 or 2: 'FROM' must keep growing)\n",
           linkRx, fromArduino, toArduino, (laptop && laptop.connected()) ? "yes" : "no");
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, t, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t captureHandler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

static esp_err_t streamHandler(httpd_req_t *req) {
  char part[64];
  esp_err_t res = httpd_resp_set_type(req, STREAM_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
      break;
    }
    size_t n = snprintf(part, sizeof(part), STREAM_PART, (unsigned)fb->len);
    res = httpd_resp_send_chunk(req, STREAM_SEP, strlen(STREAM_SEP));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part, n);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;          // the laptop disconnected
    vTaskDelay(pdMS_TO_TICKS(STREAM_GAP_MS));   // leave WiFi time for the Arduino link
  }
  return res;
}

void addPage(httpd_handle_t server, const char *path, esp_err_t (*handler)(httpd_req_t *)) {
  httpd_uri_t u = {};
  u.uri = path;
  u.method = HTTP_GET;
  u.handler = handler;
  u.user_ctx = NULL;
  httpd_register_uri_handler(server, &u);
}

void startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  httpd_handle_t web = NULL;
  if (httpd_start(&web, &cfg) == ESP_OK) {
    addPage(web, "/", indexHandler);
    addPage(web, "/capture", captureHandler);
    addPage(web, "/status", statusHandler);
  }
  cfg.server_port = 81;
  cfg.ctrl_port += 1;
  httpd_handle_t video = NULL;
  if (httpd_start(&video, &cfg) == ESP_OK) {
    addPage(video, "/stream", streamHandler);
  }
}

void startWifi() {
  bool joined = false;
  if (strlen(WIFI_NAME) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_NAME, WIFI_PASSWORD);
    WiFi.setSleep(false);
    Serial.printf("Joining WiFi \"%s\"", WIFI_NAME);
    for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
      delay(500);
      Serial.print('.');
    }
    Serial.println();
    joined = (WiFi.status() == WL_CONNECTED);
  }
  if (joined) {
    address = WiFi.localIP().toString();
  } else {
    if (strlen(WIFI_NAME) > 0) Serial.println("Could not join that WiFi (wrong name/password, or 5 GHz only?).");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_NAME, AP_PASSWORD);
    WiFi.setSleep(false);
    address = WiFi.softAPIP().toString();
    Serial.printf("Made its own WiFi: \"%s\", password \"%s\". Connect the Mac to it.\n", AP_NAME, AP_PASSWORD);
  }
  if (MDNS.begin("flycar-cam")) MDNS.addService("http", "tcp", 80);
}

void setup() {
  // The ESP32-CAM resets itself when its 5 V dips for a moment (WiFi sending pulls big current spikes).
  // This turns that self-reset off, so a short dip doesn't kill the stream. Still give it good power:
  // short thick wires straight from the LM2596, plus a 100-470 uF capacitor across 5V and GND if you have one.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, LOW);
  Serial.println();
  if (!startCamera()) {
    Serial.println("Check the camera ribbon cable and the 5 V power. Restarting in 3 s.");
    delay(3000);
    ESP.restart();
  }
  startWifi();
  startServers();
  linkRx = pickRxPin();
  arduino.begin(115200, SERIAL_8N1, linkRx, LINK_TX);
  bridge.begin();
  bridge.setNoDelay(true);
}

void pumpBridge() {
  if (bridge.hasClient()) {                      // the laptop app connected (a new one replaces the old)
    WiFiClient c = bridge.accept();
    if (laptop) laptop.stop();
    laptop = c;
    laptop.setNoDelay(true);
  }
  uint8_t buf[256];
  while (laptop && laptop.available()) {         // laptop -> Arduino
    int n = laptop.read(buf, sizeof(buf));
    if (n <= 0) break;
    arduino.write(buf, n);
    toArduino += n;
  }
  int n = arduino.available();                   // Arduino -> laptop
  if (n > 0) {
    n = arduino.readBytes(buf, n < (int)sizeof(buf) ? n : (int)sizeof(buf));
    fromArduino += n;
    if (laptop && laptop.connected()) laptop.write(buf, n);
  }
}

unsigned long lastHello = 0;

void loop() {
  pumpBridge();
  if (millis() - lastHello > 5000) {
    lastHello = millis();
    Serial.printf("CAMERA READY: http://%s:81/stream    ARDUINO LINK: %s:%d  (Arduino wire on IO%d, from Arduino %lu bytes) %s\n",
                  address.c_str(), address.c_str(), LINK_PORT, linkRx, fromArduino,
                  (laptop && laptop.connected()) ? "(laptop connected)" : "");
  }
  delay(1);
}
