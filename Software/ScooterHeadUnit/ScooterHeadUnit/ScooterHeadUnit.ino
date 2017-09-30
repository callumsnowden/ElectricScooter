#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

//defines for pins
#define RS485_TX 7
#define RS485_RX 8
#define RS485_CONTROL 6

#define LCD_RS 12
#define LCD_RW 11
#define LCD_E 10
#define LCD_D4 5
#define LCD_D5 4
#define LCD_D6 3
#define LCD_D7 2

LiquidCrystal lcd(12, 11, 10, 5, 4, 3, 2);

SoftwareSerial RS485(RS485_RX, RS485_TX);

void setup() {
  lcd.begin(16, 2);
  lcd.println("Init.");

  pinMode(RS485_CONTROL, OUTPUT);
  digitalWrite(RS485_CONTROL, LOW); //put into receive mode
  RS485.begin(57600);
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  lcd.clear();
}

void loop() {
  uint8_t rxBuffer[9];

  if (RS485.available()) {
    if (RS485.read() == 0xAA) {
      digitalWrite(13, HIGH);
      for (uint8_t i = 0; i < 9; i++) {
        rxBuffer[i] = RS485.read();
      }

      //if (rxBuffer[8] == 0x55) {
        Serial.println("Pkt!");
        lcd.clear();
        lcd.print("Thr%: ");
        uint8_t throttlePercent = rxBuffer[0];
        lcd.print(throttlePercent);

        lcd.setCursor(0, 1);
        
        lcd.print("CurA: ");
        lcd.print(rxBuffer[1]);
      //}
    }
    digitalWrite(13, LOW);
  }
}
