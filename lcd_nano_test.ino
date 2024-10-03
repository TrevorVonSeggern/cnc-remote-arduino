#include <SPI.h>
#include <LiquidCrystal.h>
#include <RotaryEncoder.h>
#include <XBee.h>
#include <Printers.h>

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
int wheel_encoder_position_prev = 0;

#define BTN_X 17
#define BTN_Y 12
#define BTN_Z 11
bool xyz_btn_state[] = {false,false,false};
bool selected_xyz[] = {true,false,false};

struct ToSendState {
  float xyz_ticks[3];
};
ToSendState send_state;
struct ReceivedState {
  float current_xyz[3];
};
ReceivedState received_state;

const char* xyz_chars = "XYZ";

static void write_lcd() {
  for(byte i = 0; i < 3; ++i) {
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
  lcd.setCursor(0, 3);
  lcd.print("                    ");
  lcd.setCursor(0, 3);
  lcd.print((int)lcd_enc_btn);
  lcd.print(" ");
  lcd.print(lcd_encoder_position);
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

void receive16(Rx16Response& rx, uintptr_t) {
  Tx16Request tx;
  tx.setAddress16(rx.getRemoteAddress16());
  tx.setPayload(rx.getFrameData() + rx.getDataOffset(), rx.getDataLength());
  xbee.send(tx);
  Serial.println(F("Sending Tx16Request"));
}
void zbReceive(ZBRxResponse& rx, uintptr_t) {
  ZBTxRequest tx;
  tx.setAddress64(rx.getRemoteAddress64());
  tx.setAddress16(rx.getRemoteAddress16());
  tx.setPayload(rx.getFrameData() + rx.getDataOffset(), rx.getDataLength());
  xbee.send(tx);
  Serial.println(F("Sending ZBTxRequest"));
}
void receive64(Rx64Response& rx, uintptr_t) {
  Tx64Request tx;
  tx.setAddress64(rx.getRemoteAddress64());
  tx.setPayload(rx.getFrameData() + rx.getDataOffset(), rx.getDataLength());
  xbee.send(tx);
  Serial.println(F("Sending Tx64Request"));
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(100);
  xbee.setSerial(Serial1);
  xbee.onPacketError(printErrorCb, (uintptr_t)(Print*)&Serial);
  xbee.onTxStatusResponse(printErrorCb, (uintptr_t)(Print*)&Serial);
  xbee.onZBTxStatusResponse(printErrorCb, (uintptr_t)(Print*)&Serial);
  xbee.onRx16Response(receive16);
  xbee.onZBRxResponse(zbReceive);
  xbee.onRx64Response(receive64);
  xbee.onOtherResponse(printResponseCb, (uintptr_t)(Print*)&Serial);

  uint8_t value = 0;
  AtCommandRequest req((uint8_t*)"AO", &value, sizeof(value));
  req.setFrameId(xbee.getNextFrameId());
  xbee.send(req);

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

  delay(100);
}

#define SPLIT_LCD 100
#define SPLIT_LCD_ENCODER 10
#define SPLIT_BUTTON 1
#define SPLIT_XBEE 10

void loop() {
  for(unsigned long loop_counter;;++loop_counter) {
    if(loop_counter%SPLIT_LCD == 0) {
      write_lcd();
    }
    if(loop_counter%SPLIT_LCD_ENCODER == 0) {
      lcd_encoder_position = lcd_encoder->getPosition();
    }
    if(loop_counter%SPLIT_XBEE == 0) {
      xbee.loop();
    }
    long t = millis();
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

          // TODO - remove once we get xbee communication to set xyz position.
          received_state.current_xyz[i] = (float)send_state.xyz_ticks[i] / 10.0f;
        }
        wheel_encoder_position_prev = wheel_encoder_position;
      }
      
      lcd_enc_btn = !digitalRead(LCD_PINS_ENC_BTN);
    }
    if(t - last_xbee_time > 1000) {
      Serial.println(wheel_encoder_position);
      last_xbee_time = t;
      Tx16Request tx;
      tx.setAddress16(0);
      tx.setPayload((uint8_t*)&send_state, sizeof(ToSendState));
      xbee.send(tx);
    }
  }
}
