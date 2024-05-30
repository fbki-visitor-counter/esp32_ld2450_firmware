
#include <FS.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Update.h>
#include <time.h>

// https://github.com/richgel999/miniz/blob/master/examples/example5.c
#define MINIZ_NO_STDIO
#define MINIZ_NO_ARCHIVE_APIS
#define MINIZ_NO_TIME
#define MINIZ_NO_ZLIB_APIS
#define MINIZ_NO_MALLOC
#define TDEFL_LESS_MEMORY 1
#include "miniz.h"

WebServer server(80);

const char *host = nullptr; // To be filled - "esp32_ld2450_MMMM"
const char* SSID = nullptr;
const char* PASS = nullptr;

const int mqtt_port = 8883;
const char* mqtt_broker = "broker.emqx.io";

const char* sensors_topic = nullptr; // To be filled
const char* command_topic = nullptr; // To be filled
const char* rawdata_topic = nullptr; // To be filled
const char* telemetry_topic = nullptr; // To be filled

WiFiClientSecure tls_client;
PubSubClient mqtt_client(tls_client);

enum {
  STARTUP,
  DEVICE_SETUP,
  NORMAL_OPERATION
} system_state = NORMAL_OPERATION;

String device_id() {
  uint8_t mac[6];

  WiFi.softAPmacAddress(mac);

  String mac_id = String(mac[4], HEX) +
                  String(mac[5], HEX);

  return String("esp32_ld2450_") + mac_id;
}

uint32_t runtime_random_id;

// https://github.com/espressif/arduino-esp32/blob/master/libraries/Insights/examples/DiagnosticsSmokeTest/DiagnosticsSmokeTest.ino
RTC_NOINIT_ATTR static uint32_t warm_reset_count;

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

typedef struct target_t {
    int16_t x;
    int16_t y;
    int16_t speed;
    uint16_t resolution;
} target_t;

typedef struct frame_t {
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

  uint32_t count_u = 0;
  uint32_t count_d = 0;
  uint32_t count_l = 0;
  uint32_t count_r = 0;

  void advance(int16_t x_, int16_t y_) {
    double x = double(x_) / 1000.0;
    double y = double(y_) / 1000.0;

    switch (state) {
      case IDLE:
        if (x >= -.5 and x <= .5 and y >= .2 and y <= 2.0) {
          state = ALERT;
        }
        break;
      case ALERT:
        if (y < .2) {
          count_u++;
          state = IDLE;
        } else if (y > 2.0) {
          count_d++;
          state = IDLE;
        } else if (x < -.5) {
          count_l++;
          state = IDLE;
        } else if (x > .5) {
          count_r++;
          state = IDLE;
        }
        break;
    }
  }
};

CounterStateMachine machines[] = {
  CounterStateMachine(),
  CounterStateMachine(),
  CounterStateMachine()
};

void print_state_machines() {
  for (int i = 0; i < 3; i++) {
    Serial.printf(" ==== Machine %d ==== \r\n", i);
    Serial.printf(" Up:    %d\r\n", machines[i].count_u);
    Serial.printf(" Down:  %d\r\n", machines[i].count_d);
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
    nc_tcp_enqueue_frame(latest_frame);
    miniz_enqueue_frame(latest_frame);

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

      machines[i].advance(latest_frame.targets[i].x, latest_frame.targets[i].y);
    }

    latest_frame.ts = 0;

    //print_state_machines();
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
    delay(0);
  }

  Serial.print("Current time: ");
  print_time();
}

// ====================================================================================================================
// MiniZ
// ====================================================================================================================

#define FRAMES_IN_BLOCK 30
#define BLOCK_SIZE_LIMIT 1024

tdefl_compressor* p_deflator = nullptr;

void miniz_init() {
  p_deflator = (tdefl_compressor*)ps_malloc(sizeof(tdefl_compressor));
  miniz_reset();
}

void miniz_reset() {
  static mz_uint s_tdefl_num_probes[11] = { 0, 1, 6, 32, 16, 32, 128, 256, 512, 768, 1500 };

  int level = 3;

  mz_uint comp_flags = TDEFL_WRITE_ZLIB_HEADER | s_tdefl_num_probes[MZ_MIN(10, level)] | ((level <= 3) ? TDEFL_GREEDY_PARSING_FLAG : 0);

  tdefl_init(p_deflator, NULL, NULL, comp_flags);
}

void miniz_enqueue_frame(struct frame_t frame) {
  static int frames_placed = 0;
  static size_t avail_out;
  static char* dst;
  static char* next_out;

  if (!p_deflator)
    return;

  if (!dst) {
    dst = (char*)malloc(BLOCK_SIZE_LIMIT);
    avail_out = BLOCK_SIZE_LIMIT;
    next_out = dst;
  }

  tdefl_status status;
  char* next_in = (char*)&frame;
  size_t avail_in = sizeof(frame_t);
  size_t in_bytes;
  size_t out_bytes = avail_out;

  while (avail_in) {
    in_bytes = avail_in;

    status = tdefl_compress(p_deflator, next_in, &in_bytes, next_out, &out_bytes, frames_placed < (FRAMES_IN_BLOCK - 1) ? TDEFL_NO_FLUSH : TDEFL_FINISH);

    next_in += in_bytes;
    avail_in -= in_bytes;

    next_out += out_bytes;
    avail_out -= out_bytes;

    if (status != TDEFL_STATUS_OKAY and status != TDEFL_STATUS_DONE) {
      mqtt_client.publish(telemetry_topic, (String("compression error ") + int(status)).c_str());
      break;
    }

    if (avail_out == 0) {
      mqtt_client.publish(telemetry_topic, "compression ran out of space");
      break;
    }
  }

  frames_placed++;

  if (frames_placed == FRAMES_IN_BLOCK) {
    if (status == TDEFL_STATUS_DONE) {
      size_t size = BLOCK_SIZE_LIMIT - avail_out;
      mqtt_client.publish(rawdata_topic, (uint8_t*)dst, size);
    } else {
      mqtt_client.publish(telemetry_topic, "compression fin error");
    }

    free(dst);

    dst = nullptr;

    miniz_reset();
    frames_placed = 0;
  }
}

// ====================================================================================================================
// TLS
// ====================================================================================================================
static const char ca_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI
2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx
1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ
q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz
tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ
vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP
BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV
5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY
1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4
NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG
Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91
8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe
pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl
MrY=
-----END CERTIFICATE-----
)EOF";

// https://github.com/knolleary/pubsubclient/pull/851/files
void tls_init() {
  tls_client.setCACert(ca_cert); // TODO: Should one seed random first?
}

// ====================================================================================================================
// MQTT
// ====================================================================================================================
void handle_message_fn(char* topic, byte* payload, unsigned int len) {
  Serial.print("MQTT: message on topic ");
  Serial.print(topic);
  Serial.print(": ");
}

// Подключение или переподключение MQTT
static void mqtt_reconnect() {
  String client_id = device_id();

  if (mqtt_client.connect(client_id.c_str())) {
    // ...
  } else {
    //Serial.println("MQTT Fail");
  }

  mqtt_client.subscribe(command_topic);
}

// Первичный запуск MQTT
void mqtt_init() {
  Serial.print("MQTT startup\n");

  String a = "axkuhta/" + device_id() + "/visitors4";
  String b = "axkuhta/" + device_id() + "/command";
  String c = "axkuhta/" + device_id() + "/rawdata";
  String d = "axkuhta/" + device_id() + "/telemetry";

  sensors_topic = strdup( a.c_str() );
  command_topic = strdup( b.c_str() );
  rawdata_topic = strdup( c.c_str() );
  telemetry_topic = strdup( d.c_str() );

  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setCallback(handle_message_fn);
  mqtt_client.setBufferSize(BLOCK_SIZE_LIMIT); // To fit larger gzip messages

  mqtt_reconnect();

  Serial.print("Command channel: ");
  Serial.println(command_topic);
  Serial.print("Sensors channel: ");
  Serial.println(sensors_topic);
  Serial.print("Rawdata channel: ");
  Serial.println(rawdata_topic);
  Serial.print("Telemetry channel: ");
  Serial.println(telemetry_topic);

  String telemetry_hello = String() + "hello\nwarm_reset_count=" + warm_reset_count + "\nfirmware=" __DATE__ " " __TIME__ "\n";

  mqtt_client.publish(telemetry_topic, telemetry_hello.c_str());
}

// No delay required
// https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_esp8266/mqtt_esp8266.ino
void mqtt_handle() {
  if (!mqtt_client.connected()) {
    mqtt_reconnect();
  }

  mqtt_client.loop();
}

// ====================================================================================================================
// Netcat radar data server
// ====================================================================================================================

#define MAX_NC_CLIENTS 4

WiFiServer nc_tcp_server(8021, MAX_NC_CLIENTS);

struct nc_client_t {
  WiFiClient socket;
  frame_t pending_frame;
} nc_clients[MAX_NC_CLIENTS];

void nc_tcp_init() {
  nc_tcp_server.begin();
  //nc_tcp_server.setTimeout(0);
}

void nc_tcp_accept_all() {
  WiFiClient client = nc_tcp_server.accept();

  if (client.connected()) {
    for (int i = 0; i < MAX_NC_CLIENTS; i++) {
      if (!nc_clients[i].socket.connected()) {
        client.printf("time,x1,y1,v1,r1,x2,y2,v2,r2,x3,y3,v3,r3\n");
        nc_clients[i].socket = client;
        return;
      }
    }

    client.printf("No slots available, sorry\n");
    client.stop();
  }
}

void nc_tcp_handle() {
  nc_tcp_accept_all();

  for (int i = 0; i < MAX_NC_CLIENTS; i++) {
    WiFiClient client = nc_clients[i].socket;

    if (nc_clients[i].pending_frame.ts == 0.0)
      continue;

    frame_t frame = nc_clients[i].pending_frame;

    if (client.connected()) {
      client.printf("%lf,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",  frame.ts,
                                                                  frame.targets[0].x, frame.targets[0].y, frame.targets[0].speed, frame.targets[0].resolution,
                                                                  frame.targets[1].x, frame.targets[1].y, frame.targets[1].speed, frame.targets[1].resolution,
                                                                  frame.targets[2].x, frame.targets[2].y, frame.targets[2].speed, frame.targets[2].resolution );
    } else {
      client.stop();
    }

    nc_clients[i].pending_frame.ts = 0.0;
  }
}

void nc_tcp_enqueue_frame(struct frame_t frame) {
  for (int i = 0; i < MAX_NC_CLIENTS; i++) {
    if (nc_clients[i].pending_frame.ts)
      continue;

    nc_clients[i].pending_frame = frame;
  }
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

    input {
      font-size: inherit;
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
  WiFi.setSleep(false); // Faster upload
  
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
    Serial.printf("%u bytes\n", upload.totalSize);
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
  server.on("/", handle_root);
  server.on("/api/setup", handle_setup);
  server.on("/api/update", HTTP_POST, handle_update, handle_update_upload);

  server.onNotFound(handleNotFound);

  server.begin();
  server.enableDelay(false); // Prevent handleClient() from locking main loop to 1000 Hz caused by delay(1) in handleClient()

  host = strdup( device_id().c_str() );

  // $ avahi-browse -a
  MDNS.begin(host);
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

  if (esp_reset_reason() == ESP_RST_POWERON) {
    warm_reset_count = 0;
  } else {
    warm_reset_count++;
  }

  Serial.begin(921600);

  radar_init();
  littlefs_init();

  if (!read_credentials()) {
    transition_to_setup();
    http_server_init();
    nc_tcp_init();
    return;
  }

  WiFi.begin(SSID, PASS);
  WiFi.setSleep(true); // Wi-Fi ping latency 10ms -> 500ms
  system_state = STARTUP;
}

void publish_visitors() {
  char* buf = (char*)malloc(512);
  char* ts = timestamp();

  int visitors_u = machines[0].count_u + machines[1].count_u + machines[2].count_u;
  int visitors_d = machines[0].count_d + machines[1].count_d + machines[2].count_d;
  int visitors_l = machines[0].count_l + machines[1].count_l + machines[2].count_l;
  int visitors_r = machines[0].count_r + machines[1].count_r + machines[2].count_r;

  const char* fmt_str = 
R"===({
  "runtime_random_id": %u,
  "device_timestamp": "%s",
  "u": %d,
  "d": %d,
  "l": %d,
  "r": %d
})===";

  snprintf(buf, 512, fmt_str, runtime_random_id, ts, visitors_u, visitors_d, visitors_l, visitors_r);

  Serial.println(buf);

  // True means "retain"
  // A topic can have one sticky retained message
  bool success = mqtt_client.publish(sensors_topic, buf, true);

  free(ts);
  free(buf);
}

void visitors_handle() {
  static uint32_t publish_at;

  if (millis() >= publish_at) {
    publish_visitors();
    publish_at += 10000;
  }
}

void handle_led() {
  switch (system_state) {
    case STARTUP:
      digitalWrite(LED_BUILTIN, millis() % 200 < 100);
      break;
    case DEVICE_SETUP:
      digitalWrite(LED_BUILTIN, millis() % 2000 < 1000);
      break;
    case NORMAL_OPERATION:
      digitalWrite(LED_BUILTIN, millis() % 5000 < 100);
      break;
  }
}

#define WIFI_GRACE_PERIOD 20000
uint32_t wifi_last_ok = 0;

void transition_to_setup() {
  // WiFi.disconnect(true); -- Combined AP+STA mode runs like garbage (ping 1000ms, PL 66%) when STA is not associated and scanning channels, but oh well
  WiFi.softAP("ESP32-LD2450");
  system_state = DEVICE_SETUP;
}

void measure_loop_rate() {
  static uint64_t wakeups;
  static uint32_t measure_at;

  if (millis() >= measure_at) {
    measure_at = millis() + 1000;
    Serial.println(wakeups);
    wakeups = 0;
  }

  wakeups++;
}

void loop() {
  if (WiFi.status() == WL_CONNECTED)
    wifi_last_ok = millis();

  bool long_time_no_wifi = millis() > wifi_last_ok + WIFI_GRACE_PERIOD;

  switch (system_state) {
    case NORMAL_OPERATION:
      if (long_time_no_wifi) {
        transition_to_setup();
      } else {
        server.handleClient();
        nc_tcp_handle();
        visitors_handle();
        mqtt_handle();
      }
      break;
    case DEVICE_SETUP:
      if (WiFi.status() == WL_CONNECTED) {
        ESP.restart();
      } else {
        server.handleClient();
        nc_tcp_handle();
      }
      break;
    case STARTUP:
      if (WiFi.status() == WL_CONNECTED) {
        http_server_init();
        nc_tcp_init();
        ntp_init();
        tls_init();
        mqtt_init();
        miniz_init();
        runtime_random_id = esp_random();
        system_state = NORMAL_OPERATION;
      } else {
        if (long_time_no_wifi) {
          transition_to_setup();
          http_server_init();
          nc_tcp_init();
        }
      }
      break;
  }

  handle_led();
  handle_latest_frame();

  //
  // Using cpu_ll_waiti() to halt the CPU and slow the main loop down to 1000 Hz (systick) when the system is otherwise idle
  //
  // ESP-IDF implements this but only when all FreeRTOS tasks are blocked (which is never?), see ESP-IDF v4.4.7 source code
  // components/freertos/port/xtensa/include/freertos/portmacro.h:#define vApplicationIdleHook esp_vApplicationIdleHook
  // components/esp_system/freertos_hooks.c: void esp_vApplicationIdleHook(void)
  //
  cpu_ll_waiti();
  delay(0);
}
