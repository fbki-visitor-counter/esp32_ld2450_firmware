
#include <FS.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Update.h>
#include <time.h>

WebServer server(80);

const char *host = "esp32-webupdate";
const char* SSID = nullptr;
const char* PASS = nullptr;

enum {
  STARTUP,
  DEVICE_SETUP,
  NORMAL_OPERATION
} system_state = NORMAL_OPERATION;

// ====================================================================================================================
// Non-volatile memory
// ====================================================================================================================

const String file_credentials = R"(/credentials.txt)";

void littlefs_init() {
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS initialization error, programmer flash configured?");
    ESP.restart();
  }
}

// https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WebServer/examples/HttpHashCredAuth/HttpHashCredAuth.ino
int read_credentials() {
  File f;
  f=LittleFS.open(file_credentials,"r");
  if(f){
    Serial.println("Found SSID, PASS.");

    String mod = f.readString();

    int i1 = mod.indexOf('\n',0);
    int i2 = mod.indexOf('\n', i1+1);

    SSID = strdup( mod.substring(0, i1-1).c_str() );
    PASS = strdup( mod.substring(i1+1,i2-1).c_str() );

    Serial.print("[");
    Serial.print(SSID);
    Serial.println("]");

    Serial.print("[");
    Serial.print(PASS);
    Serial.println("]");

    f.close();

    return 1;
  } else {
    return 0;
  }
}

void save_credentials(String ssid_, String pass_) {
  File f = LittleFS.open(file_credentials, "w");  // open as a brand new file, discard old contents
  if (f) {
    f.println(ssid_);
    f.println(pass_);
    f.close();

    Serial.println("Credentials saved.");
  } else {
    Serial.println("LittleFS error");

    while (1) {}
  }
}

// ====================================================================================================================
// WiFi
// ====================================================================================================================
int wifi_reconnect(int grace_period) {
  while( WiFi.status() != WL_CONNECTED ) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);

    grace_period--;

    if (grace_period == 0)
     return 0;
  }

  return 1;
}

int wifi_init() {
  WiFi.begin(SSID, PASS);
  return wifi_reconnect(200);
}

// ====================================================================================================================
// Real time
// ====================================================================================================================
char* timestamp() {
  char* buf = (char*)malloc(32);
  time_t now;
  tm tm;  

  time(&now);                       // read the current time
  localtime_r(&now, &tm);           // update the structure tm with the current time

  strftime(buf, 32, "%Y-%m-%d %H:%M:%S", &tm);

  return buf;
}

void print_time() {
  char* ts = timestamp();

  Serial.println(ts);

  free(ts);
}

double frac_time() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);

  return tv.tv_sec + tv.tv_usec / 1000000.0;
}

void ntp_init() {
  Serial.print("NTP Startup\n");
  configTime(0, 0, "time.google.com", "time.windows.com", "pool.ntp.org"); // API differs from ESP8266 core slightly

  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    now = time(nullptr);
    delay(100);
  }

  Serial.print("Current time: ");
  print_time();
}

// ====================================================================================================================
// Device setup
// ====================================================================================================================

//
// SmartConfig (unerliable):
// https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/examples/WiFiSmartConfig/WiFiSmartConfig.ino
// https://www.youtube.com/watch?v=a6-GlYtFu4c
//
// WiFiProv (too large???):
// https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFiProv/examples/WiFiProv/WiFiProv.ino
//
// Captive portal:
// https://github.com/espressif/arduino-esp32/blob/master/libraries/DNSServer/examples/CaptivePortal/CaptivePortal.ino
//
// mDNS:
// https://github.com/espressif/arduino-esp32/blob/master/libraries/HTTPUpdateServer/examples/WebUpdater/WebUpdater.ino
//

static const char root_page[] = R"===(
<!DOCTYPE html>
<html>
  <head>
    <title>ESP32-LD2450 Visitor counter</title>
    <style>
		body {
			font-family: system-ui, sans-serif;
			font-size: 20px;
			margin: 0;

			background-color: #E88F00;
			color: #00305B;
		}

		.container {
			max-width: 48rem;
			margin: 0px auto;
		}

		.content {
			background-color: #202020;
			color: #D0D0D0;

			font-size: 18px;
			padding: 1rem;

			min-height: 80vh;
		}
    </style>
  </head>
  <body>
    <div class="header">
      <div class="container">
        <h1>ESP32-LD2450 Visitor counter</h1>
      </div>
    </div>
    <div class="content">
      <div class="container">
        <h2>Device setup</h2>
        <p>Please configure a Wi-Fi network for the visitor counter to use.</p>
        <form method="POST" action="/api/setup">
          <input type="text" name="ssid" placeholder="SSID..."><br>
          <input type="password" name="pass" placeholder="Password..."><br>
          <input type="submit" value="Save">
        </form>
        <h2>Firmware upgrade</h2>
        <p>Attach a .bin file for OTA firmware upgrade.</p>
        <form method="POST" action="/api/update" enctype="multipart/form-data">
          <input type="file" name="update">
          <input type="submit" value="Update">
        </form><br>
      </div>
    </div>
    <div class="footer">
      <div class="container">
        <p>Firmware version: )===" __DATE__ " " __TIME__ R"===(</p>
      </div>
    </div>
    </main>
  </body>
</html>
)===";

void handle_root() {
  server.send(200, "text/html", root_page);
}

void handleNotFound() {
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "redirect to root");
}

void handle_setup() {
  if (!server.hasArg("ssid") or !server.hasArg("pass")) {
    server.send(500, "text/plain", "BAD ARGS");
    return;
  }

  String ssid_ = server.arg("ssid");
  String pass_ = server.arg("pass");

  save_credentials(ssid_, pass_);

  ESP.restart();
}

void handle_update() {
  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
  ESP.restart();
}

void handle_update_upload() {
  HTTPUpload &upload = server.upload();
  
  if (upload.status == UPLOAD_FILE_START) {
    Serial.setDebugOutput(true);
    Serial.printf("Update: %s\n", upload.filename.c_str());
    if (!Update.begin()) {  //start with max available size
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {  //true to set the size to the current progress
      Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
    Serial.setDebugOutput(false);
  } else {
    Serial.printf("Update Failed Unexpectedly (likely broken connection): status=%d\n", upload.status);
  }
}

void http_server_init() {
  MDNS.begin(host);

  server.on("/", handle_root);
  server.on("/api/setup", handle_setup);
  server.on("/api/update", HTTP_POST, handle_update, handle_update_upload);

  server.onNotFound(handleNotFound);
  server.begin();

  // $ avahi-browse -a
  MDNS.addService("http", "tcp", 80);
}

void device_setup() {
  WiFi.softAP("ESP32-LD2450");
}

// ====================================================================================================================
// Startup + main loop
// ====================================================================================================================

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(921600);

  radar_init();
  littlefs_init();

  if (!read_credentials()) {
    device_setup();
    http_server_init();
    return;
  }

  if (!wifi_init()) {
    device_setup();
    http_server_init();
    return;
  }

  http_server_init();
  ntp_init();
}

// ====================================================================================================================
// LD2450
// ====================================================================================================================

HardwareSerial radar_uart(1);

// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Serial/OnReceive_Demo/OnReceive_Demo.ino
void radar_init() {
  pinMode(16, INPUT);
  pinMode(18, OUTPUT);

  radar_uart.begin(256000, SERIAL_8N1, 16, 18); // RX, TX
  radar_uart.setRxFIFOFull(127);
  radar_uart.onReceive(handle_radar_uart, false);
}

void handle_radar_uart(void) {
  if (radar_uart.available() == 127) {
    // Overrun... do nothing
  }

  // Unknown or truncated frame - skip
  if (radar_uart.available() != 30) {
    while (radar_uart.available())
      radar_uart.read();

    return;
  }

  read_ld2450_frame();
}

typedef struct target {
    int16_t x;
    int16_t y;
    int16_t speed;
    uint16_t resolution;
} target_t;

typedef struct frame {
  target_t targets[3];
  double ts;
} frame_t;

frame_t latest_frame = {0}; // Singular frame used for immediate serial output to a PC
uint32_t dropped_frames = 0; // LD2450 frames dropped because of main loop not keeping up

void read_ld2450_frame() {
  char frame[30] = {0};

  radar_uart.read(frame, 30);

  bool head_valid = frame[0] == 0xAA and frame[1] == 0xFF and frame[2] == 0x03 and frame[3] == 0x00;
  bool tail_valid = frame[28] == 0x55 and frame[29] == 0xCC;

  if (not head_valid or not tail_valid)
    return;

  if (latest_frame.ts) {
    dropped_frames++;
    return;
  }

  latest_frame.ts = frac_time();

  for (int i = 0; i < 3; i++) {
    uint16_t x_ = frame[4 + 8*i + 0] + frame[4 + 8*i + 1]*256;
    uint16_t y_ = frame[4 + 8*i + 2] + frame[4 + 8*i + 3]*256;
    uint16_t speed_ = frame[4 + 8*i + 4] + frame[4 + 8*i + 5]*256;
    uint16_t d_resolution_ = frame[4 + 8*i + 6] + frame[4 + 8*i + 7]*256;

    // LD2450 uses a custom signed number format
    latest_frame.targets[i].x = x_ & 0x8000 ? x_ - 0x8000 : -x_;
    latest_frame.targets[i].y = y_ & 0x8000 ? y_ - 0x8000 : -y_;
    latest_frame.targets[i].speed = speed_ & 0x8000 ? speed_ - 0x8000 : -speed_;
    latest_frame.targets[i].resolution = d_resolution_;
  }
}

class CounterStateMachine {
public:
  enum {
    IDLE,
    ALERT
  } state = IDLE;

  uint32_t state_ttl = 0;

  uint32_t count_r = 0;
  uint32_t count_l = 0;
  
  // x(t - 1)
  // v(t - 1)
  int16_t x_ = 0;
  int16_t v_ = 0;

  void transition_to_idle() {
    state = IDLE;
    state_ttl = 0;
  }

  void transition_to_alert() {
    state = ALERT;
    state_ttl = 10;
  }

  void r_event() {
    transition_to_idle();
    count_r++;
  }

  void l_event() {
    transition_to_idle();
    count_l++;
  }

  void advance(int16_t x, int16_t v) {
    bool vz = v_ < 0 and v >= 0;
    bool xz_r = x_ < 0 and x >= 0;
    bool xz_l = x < 0 and x_ >= 0;

    if (vz) transition_to_alert();

    if (state == ALERT) {
      if (xz_r) r_event();
      if (xz_l) l_event();

      if (state_ttl == 0) {
        transition_to_idle();
      } else {
        state_ttl--;
      }
    }

    x_ = x;
    v_ = v;
  }
};

CounterStateMachine sm1;
CounterStateMachine sm2;
CounterStateMachine sm3;

CounterStateMachine machines[] = {sm1, sm2, sm3};

void print_state_machines() {
  for (int i = 0; i < 3; i++) {
    Serial.printf(" ==== Machine %d ==== \r\n", i);
    Serial.printf(" Left:  %d\r\n", machines[i].count_l);
    Serial.printf(" Right: %d\r\n", machines[i].count_r);
  }
}

void bent_pipe(Stream& a, Stream& b) {
  if (radar_uart.available())
   Serial.println(radar_uart.read(), HEX);

  if (Serial.available())
   radar_uart.write(Serial.read());
}

void handle_latest_frame() {
  if (latest_frame.ts) { 
    Serial.print(latest_frame.ts, 4);
    Serial.print(",");

    for (int i = 0; i < 3; i++) {
      Serial.print(latest_frame.targets[i].x);
      Serial.print(",");

      Serial.print(latest_frame.targets[i].y);
      Serial.print(",");

      Serial.print(latest_frame.targets[i].speed);
      Serial.print(",");

      Serial.print(latest_frame.targets[i].resolution);
      Serial.print(i < 2 ? "," : "\r\n");

      machines[i].advance(latest_frame.targets[i].x, latest_frame.targets[i].speed);
    }

    latest_frame.ts = 0;

    //print_state_machines();
  }
}

void handle_led() {
  switch (system_state) {
    case STARTUP:
      digitalWrite(LED_BUILTIN, millis() % 20 < 10);
      break;
    case DEVICE_SETUP:
      digitalWrite(LED_BUILTIN, millis() % 2000 < 1000);
      break;
    case NORMAL_OPERATION:
      digitalWrite(LED_BUILTIN, millis() % 5000 < 100);
      break;
  }
}

void loop() {
  handle_led();
  handle_latest_frame();
  server.handleClient();
}
