
HardwareSerial radar_uart(1);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(921600);

  pinMode(16, INPUT);
  pinMode(18, OUTPUT);

  // https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Serial/OnReceive_Demo/OnReceive_Demo.ino
  radar_uart.begin(256000, SERIAL_8N1, 16, 18); // RX, TX
  radar_uart.setRxFIFOFull(127);
  radar_uart.onReceive(handle_radar_uart, false);
}

double frac_time() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);

  return tv.tv_sec + tv.tv_usec / 1000000.0;
}

void panic(const char* message) {
  Serial.print(message);
  Serial.flush();
  ESP.restart();
}

void handle_radar_uart(void) {
  if (radar_uart.available() == 127)
    panic("UART overrun");

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

// Singular frame used for immediate serial output to a PC
frame_t latest_frame = {0};

void read_ld2450_frame() {
  char frame[30] = {0};

  radar_uart.read(frame, 30);

  bool head_valid = frame[0] == 0xAA and frame[1] == 0xFF and frame[2] == 0x03 and frame[3] == 0x00;
  bool tail_valid = frame[28] == 0x55 and frame[29] == 0xCC;

  if (not head_valid or not tail_valid)
    return;

  if (latest_frame.ts)
    panic("Main loop not keeping up");

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

    print_state_machines();
  }
}

void loop() {
  handle_latest_frame();

  digitalWrite(LED_BUILTIN, millis() % 5000 < 100);
}
