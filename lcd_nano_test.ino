#include <SPI.h>
#include <LiquidCrystal.h>
#include <RotaryEncoder.h>
#include <Printers.h>

//#define DEBUG

XBeeWithCallbacks xbee;

#define LCD_PINS_RS 5
#define LCD_PINS_ENABLE 4
#define LCD_PINS_B1 14
#define LCD_PINS_B2 10
#define LCD_PINS_B3 9
#define LCD_PINS_B4 2
LiquidCrystal lcd(LCD_PINS_RS, LCD_PINS_ENABLE, LCD_PINS_B1, LCD_PINS_B2, LCD_PINS_B3, LCD_PINS_B4);
#define screenX 20
#define screenY 4

#define LCD_PINS_ENC_A 7
#define LCD_PINS_ENC_B 8
#define LCD_PINS_ENC_BTN 3
RotaryEncoder *lcd_encoder = nullptr;
void check_lcd_encoder() { lcd_encoder->tick(); }
int lcd_encoder_position = 0;
bool lcd_enc_btn = false;

#define WHEEL_PINS_A 6
#define WHEEL_PINS_B 18
const int wheel_debounce = 100;
volatile unsigned long wheel_debounce_check_microsec = 0;
volatile long wheel_encoder_position = 0;
long wheel_encoder_position_prev = 0;
long wheel_tty_position = 0;
const int wheel_tty_debounce = 100;
unsigned long wheel_tty_time = 0;

#define BTN_X 17
#define BTN_Y 12
#define BTN_Z 11
bool xyz_btn_state[] = {false,false,false};
bool selected_xyz[] = {true,false,false};

char sd_menu_list[screenX*screenY];
char recieve_buffer[256];
byte recieve_buffer_end = 0;
byte recieve_buffer_begin = 0;

const char* xyz_chars = "XYZ";

struct ToSendState {
  long xyz_ticks[3];
};
ToSendState send_state;
static void send_serial_state() {
  Serial1.print("W:");
  for(byte i = 0; i < 3; ++i) {
    Serial1.print(xyz_chars[i]);
    Serial1.print(send_state.xyz_ticks[i]);
    Serial1.print(' ');
  }
  Serial1.println();
}

struct ReceivedState {
  float current_xyz[3];
};
ReceivedState received_state;
ReceivedState printed_lcd_state;

static void write_lcd() {
  for(byte i = 0; i < 3; ++i) {
    if(printed_lcd_state.current_xyz[i] == received_state.current_xyz[i]) {
      continue;
    }
    lcd.setCursor(0, i);
    char selected = ' ';
    if(selected_xyz[i]) {
      selected = '*';
    }
    lcd.print(selected);
    lcd.print(xyz_chars[i]);
    if(received_state.current_xyz[i] >= 0) {
      lcd.print(" ");
    }
    lcd.print(String(received_state.current_xyz[i], 2));
  }
//  lcd.setCursor(0, 3);
//  lcd.print("                    ");
//  lcd.setCursor(0, 3);
//  lcd.print((int)lcd_enc_btn);
//  lcd.print(" ");
//  lcd.print(lcd_encoder_position);
}

void wheel_encoder_A_interrupt() {
  unsigned long _time = micros();
  if(_time - wheel_debounce_check_microsec >= wheel_debounce) {
    wheel_debounce_check_microsec = _time;
    if(digitalRead(WHEEL_PINS_A) && (digitalRead(WHEEL_PINS_B))) {
      wheel_encoder_position--;
    }
  }
}
void wheel_encoder_B_interrupt() {
  unsigned long _time = micros();
  if(_time - wheel_debounce_check_microsec >= wheel_debounce) {
    wheel_debounce_check_microsec = _time;
    if(digitalRead(WHEEL_PINS_A) && (digitalRead(WHEEL_PINS_B))) {
      wheel_encoder_position++;
    }
  }
}

unsigned long last_xbee_time = 0;
unsigned long last_button_time = 0;

void setup() {
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  Serial1.begin(9600);
  delay(100);

  pinMode(BTN_X, INPUT);
  pinMode(BTN_Y, INPUT);
  pinMode(BTN_Z, INPUT);
  digitalWrite(BTN_X, LOW);
  digitalWrite(BTN_Y, LOW);
  digitalWrite(BTN_Z, LOW);
  pinMode(LCD_PINS_ENC_BTN, INPUT);

  lcd_encoder = new RotaryEncoder(LCD_PINS_ENC_A, LCD_PINS_ENC_B, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(LCD_PINS_ENC_A), check_lcd_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LCD_PINS_ENC_B), check_lcd_encoder, CHANGE);

  pinMode(WHEEL_PINS_A, INPUT);
  pinMode(WHEEL_PINS_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(WHEEL_PINS_A), wheel_encoder_A_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(WHEEL_PINS_B), wheel_encoder_B_interrupt, RISING);

  lcd.begin(screenX, screenX);
  lcd.clear();
  last_xbee_time = millis();
  printed_lcd_state.current_xyz[0] = -1;
  printed_lcd_state.current_xyz[1] = -1;
  printed_lcd_state.current_xyz[2] = -1;

  delay(100);
}

int find_char_in_buffer(byte needle) {
  for(byte i = recieve_buffer_begin; i != recieve_buffer_end; ++i) {
    if(recieve_buffer[i] == needle) {
      return i;
    }
  }
  return -1;
}

char event_buffer[256];
void parse_events() {
  int nl_i = find_char_in_buffer('\n');
  if (nl_i == -1) { return; }
  byte read_size = nl_i - recieve_buffer_begin;
  byte e = 0;
  for(byte i = recieve_buffer_begin; i != recieve_buffer_end; ++i) {
    event_buffer[e++] = recieve_buffer[i];
    #ifdef DEBUG
    Serial.print(recieve_buffer[i]);
    #endif
  }
  for(byte i = recieve_buffer_begin; i != recieve_buffer_end; ++i) {
    recieve_buffer[i] = 0;
  }
  event_buffer[read_size] = 0;
  recieve_buffer_begin = recieve_buffer_end;
  if(read_size < 3) { return; }
  if(event_buffer[0] == 'P' && event_buffer[1] == ':') {
    char* pch = strtok (event_buffer + 2, " ");
    while (pch != NULL)
    {
      char* num_str = pch + 1;
      float num = atof(num_str);
      switch(pch[0]) {
        case 'X':
          received_state.current_xyz[0] = num;
          break;
        case 'Y':
          received_state.current_xyz[1] = num;
          break;
        case 'Z':
          received_state.current_xyz[2] = num;
          break;
        default:
          Serial.println("error parsing input");
          break;
      }
      pch = strtok (NULL, " ");
    }
  }
}

#define SPLIT_LCD 100
#define SPLIT_LCD_ENCODER 10
#define SPLIT_BUTTON 1
#define SPLIT_XBEE 10

void loop() {
  for(unsigned long loop_counter;;++loop_counter) {
    unsigned long t = millis();
    if(loop_counter%SPLIT_LCD == 0) {
      write_lcd();
    }
    if(loop_counter%SPLIT_LCD_ENCODER == 0) {
      lcd_encoder_position = lcd_encoder->getPosition();
    }
    if(loop_counter%SPLIT_XBEE == 0) {
      if(wheel_encoder_position != wheel_tty_position && t - wheel_tty_time >= wheel_tty_debounce) {
        wheel_tty_position = wheel_encoder_position;
        wheel_tty_time = t;
        send_serial_state();
      }
    }
    if(loop_counter%SPLIT_BUTTON == 0 && t - last_button_time > 10) {
      last_button_time = t;
      bool b_xyz[] = {digitalRead(BTN_X), digitalRead(BTN_Y), digitalRead(BTN_Z)};
      int rising_count = 0;
      int old_xyz_count = 0;
      for(byte i = 0; i < 3; ++i) {
        rising_count += (int)(!xyz_btn_state[i] && b_xyz[i]);
        old_xyz_count += (int)xyz_btn_state[i];
      }
      if(old_xyz_count == 0 && rising_count > 0) {
        for(byte i = 0; i < 3; ++i) {
          selected_xyz[i] = false;
        }
      }
      for(byte i = 0; i < 3; ++i) {
        selected_xyz[i] = selected_xyz[i] || (!xyz_btn_state[i] && b_xyz[i]);
        xyz_btn_state[i] = b_xyz[i];
      }
      int wheel_diff = wheel_encoder_position - wheel_encoder_position_prev;
      if(wheel_diff != 0) {
        for(byte i = 0; i < 3; ++i) {
          if(selected_xyz[i]) {
            send_state.xyz_ticks[i] += wheel_diff;
          }
        }
        wheel_encoder_position_prev = wheel_encoder_position;
      }
      
      lcd_enc_btn = !digitalRead(LCD_PINS_ENC_BTN);
    }

    int serial_data = -1;
    bool new_data = false;
    do {
      serial_data = Serial1.read();
      if(serial_data == -1) { break; }
      new_data = true;
      if(recieve_buffer_end == recieve_buffer_begin - 1)
        recieve_buffer_begin++; // about to overflow, overwrite oldest data.
      recieve_buffer[recieve_buffer_end++] = (byte)serial_data;

    } while(serial_data != -1);
    if(new_data) {
      parse_events();
    }
  }
}
