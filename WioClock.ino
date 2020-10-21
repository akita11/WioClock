#include "RTC_SAMD51.h"
#include "DateTime.h"
#include <AtWiFi.h>
#include <SPI.h>
#include <TFT_eSPI.h> // Hardware-specific library
//#include <EnergySaving.h>

// Use Seeed's official RTC library whose version is after 2020/10/21.

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0;    // Saved H, M, S x & y multipliers
float sdeg = 0, mdeg = 0, hdeg = 0;
uint16_t osx = 120, osy = 120, omx = 120, omy = 120, ohx = 120, ohy = 120; // Saved H, M, S x & y coords
uint16_t x0 = 0, x1 = 0, yy0 = 0, yy1 = 0;
uint32_t targetTime = 0;                    // for next 1 second timeout
//static uint8_t conv2d(const char* p); // Forward declaration needed for IDE 1.6.x
//uint8_t hh = conv2d(__TIME__), mm = conv2d(__TIME__ + 3), ss = conv2d(__TIME__ + 6); // Get H, M, S from compile time
boolean initial = 1;

#define CX 120
#define CY 120
#define RX 115
#define RY 115
#define WE 10
#define WT 20
#define WH 70
#define WM 40
#define WS 50
#define WZ1 85
#define WZ2 110
#define RH 5
#define RM 3
#define COLOR_H TFT_WHITE
#define COLOR_M TFT_CYAN
#define COLOR_S TFT_RED

#define PI_180 0.0174532925199433 // PI/180

#define THETA_OFFSET 90

int time_slot = 0;
#define HMconv(h,m) (h*60+m)
#define N_TIME_SLOT 6
int time_slot_start[] = {HMconv(8, 45), HMconv(10, 30), HMconv(13, 00), HMconv(14, 45), HMconv(16, 30), HMconv(18, 15)};
// TFT_BROWN = 0x9A60
// ref: https://github.com/Bodmer/TFT_eSPI/blob/master/TFT_eSPI.h
int time_slot_color[] = {0x9A60, TFT_RED, TFT_ORANGE, TFT_YELLOW, TFT_GREEN, TFT_BLUE};
#define TIME_SLOT_LENGTH 90

//EnergySaving nrgSave;

// Note: need before use:
// - install some libraries related to WiFi:
//   https://qiita.com/jksoft/items/cb11eb171002c0ed1f25
// - Wio Termial update:
// - https://qiita.com/jksoft/items/cb11eb171002c0ed1f25

// Reference:
// NTP client: https://beta-notes.way-nifty.com/blog/2020/08/post-3484c0.html

#define WIFI_SSID     "YOUR_SSID"
#define WIFI_PASSWORD "YOUR_PWD"

#define NTP_SERVER "210.173.160.27" // ntp1.jst.mfeed.ad.jp 
const int NTP_PACKET_SIZE = 48;
boolean fNTPadjusted = 0;

DateTime now;
RTC_SAMD51 rtc;

#define PIR_PIN 0 // D0

float cylinderX(int cx, int rx, float theta)
{
  return (rx * cos((theta - THETA_OFFSET) * PI_180) + cx);
}
float cylinderY(int cy, int ry, float theta)
{
  return (ry * sin((theta - THETA_OFFSET) * PI_180) + cy);
}


uint32_t get_ntp_time()
{
  char ntp_buf[ NTP_PACKET_SIZE];
  uint32_t epoch;
  WiFiUDP udp;
  udp.begin(8000);
  udp.beginPacket(NTP_SERVER, 123);
  for (int i = 0; i < NTP_PACKET_SIZE; i++) ntp_buf[i] = 0;
  ntp_buf[0] = 0xe3; //0b11100011;
  ntp_buf[1] = 0; ntp_buf[2] = 6; ntp_buf[3] = 0xEC;
  ntp_buf[12] = 49; ntp_buf[13] = 0x4E; ntp_buf[14] = 49; ntp_buf[15] = 52;

  udp.write((const uint8_t*)ntp_buf, NTP_PACKET_SIZE);
  udp.endPacket();

  delay(1000); // wait for NTP server's response
  int packetSize = udp.parsePacket();
  int len = udp.read(ntp_buf, packetSize);
  uint32_t highWord = (ntp_buf[40] << 8) | ntp_buf[41];
  uint32_t lowWord = (ntp_buf[42] << 8) | ntp_buf[43];
  uint32_t secsSince1900 = highWord << 16 | lowWord;  // seconds since 1900/1/1
  const uint32_t seventyYears = 2208988800UL;
  epoch = secsSince1900 - seventyYears;
  epoch += (9 * 60 * 60); // convert to JST (GMT+9)
  return (epoch);
}

void DrawClockBase()
{
  tft.fillScreen(TFT_BLACK);

  // Draw clock face
  tft.fillEllipse(CX, CY, RX, RY, TFT_WHITE);
  tft.fillEllipse(CX, CY, RX - WE, RY - WE, TFT_BLACK);
  // Draw 12 lines
  for (int i = 0; i < 360; i += 30) {
    x0 = cylinderX(CX, (RX - WE), i); yy0 = cylinderY(CY, (RY - WE), i);
    x1 = cylinderX(CX, (RX - WT), i); yy1 = cylinderY(CY, (RY - WT), i);
    tft.drawLine(x0, yy0, x1, yy1, TFT_WHITE);
  }

  // Draw 60 dots
  for (int i = 0; i < 360; i += 6) {
    x0 = cylinderX(CX, (RX - WT), i); yy0 = cylinderY(CY, (RY - WT), i);
    // Draw minute markers
    tft.drawPixel(x0, yy0, TFT_WHITE);

    // Draw main quadrant dots
    if (i == 0 || i == 180) {
      tft.fillCircle(x0, yy0, 2, TFT_WHITE);
    }
    if (i == 90 || i == 270) {
      tft.fillCircle(x0, yy0, 2, TFT_WHITE);
    }
  }
}

void ISRext()
{
}

void NTPadjust()
{
  Serial.print("connecting WiFi:"); Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA); // turn WiFi ON
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.println();
  Serial.printf("Connected, IP address: ");
  Serial.println(WiFi.localIP());
  //    configTzTime("JST-9", "ntp.nict.jp", "ntp.jst.mfeed.ad.jp");   // currently not available in Wio Terminal

  DateTime now_ntp = DateTime(get_ntp_time());
  rtc.adjust(now_ntp);

  WiFi.mode(WIFI_OFF); // turn WiFi OFF

  targetTime = millis();
  initial = 1;
}

void setup()
{
  rtc.begin();
  Serial.begin(115200);

//  while (!Serial) ;

  // Wio Temrinal Grove:
  //https://wiki.seeedstudio.com/Wio-Terminal-Getting-Started/#pinout-diagram

  tft.init();
  tft.setRotation(3);
  tft.setTextSize(1);

  DrawClockBase();

  targetTime = millis() + 1000;
  //  nrgSave.begin(WAKE_EXT_INTERRUPT, 13, ISRext);  //standby setup for external interrupts
  //  nrgSave.standby();  //now mcu go to standby
  pinMode(WIO_KEY_A, INPUT_PULLUP);
  pinMode(WIO_KEY_B, INPUT_PULLUP);

  pinMode(PIR_PIN, INPUT);
  //  attachInterrupt(PIR_PIN, ISRext, HIGH);         // Set up an interrupt on analog pin A1 on a logic HIGH level
  //  PM->SLEEPCFG.bit.SLEEPMODE = 0x4;             // Set up SAMD51 to enter low power STANDBY mode
  //  while(PM->SLEEPCFG.bit.SLEEPMODE != 0x4);     // Wait for it to take
  //  USB->DEVICE.CTRLA.bit.ENABLE = 0;                   // Shutdown the USB peripheral
  //  while(USB->DEVICE.SYNCBUSY.bit.ENABLE);             // Wait for synchronization
}

boolean fDisplayBackLight = true;

int Ton = 0;
boolean fPIR = false;
int NTPadjustH, NTPadjustM;

void loop()
{
  //  __DSB();
  //  __WFI();
  //  delay(100);
  if (digitalRead(PIR_PIN) == 1) {
    digitalWrite(LCD_BACKLIGHT, HIGH); fDisplayBackLight = true;
    fPIR = true;
  }
  if (digitalRead(WIO_KEY_A) == LOW) {
    tft.fillScreen(TFT_GREEN);
    NTPadjust();
    targetTime = millis();
    DrawClockBase();
  }
#define LCD_BACKLIGHT (72Ul) // Control Pin of LCD
  // https://webshop.pupu.jp/grape/WioTerminal/BasicsOfLCD.html#6

  if (digitalRead(WIO_KEY_B) == LOW) {
    if (fDisplayBackLight == true) {
      digitalWrite(LCD_BACKLIGHT, LOW); fDisplayBackLight = false;
    }
    else {
      digitalWrite(LCD_BACKLIGHT, HIGH); fDisplayBackLight = true;
    }
    while (digitalRead(WIO_KEY_B) == LOW) delay(10);
  }
#define DRAW_STEP_SECOND 1
  if (targetTime < millis()) {
    DateTime now;
    now = rtc.now();
    if (fPIR == true) {
      Ton = 0;
    }
    else {
      if (Ton == DRAW_STEP_SECOND * 300) { // 300sec (5min)
        digitalWrite(LCD_BACKLIGHT, LOW); fDisplayBackLight = false;
      }
      else Ton++;
    }
    fPIR = false;

    targetTime += DRAW_STEP_SECOND * 1000;

    // Pre-compute hand degrees, x & y coords for a fast screen update
    //    sdeg = now.second() * 6;                // 0-59 -> 0-354
    sdeg = 0;
    mdeg = now.minute() * 6 + sdeg / 60.0; // 0-59 -> 0-360 - includes seconds
    hdeg = now.hour() * 30 + mdeg / 12.0; // 0-11 -> 0-360 - includes minutes and seconds
    if (now.second() < DRAW_STEP_SECOND || initial) {
      Serial.println("----");
      Serial.println(now.hour());
      Serial.println(now.minute());
      initial = 0;
      // Erase hour and minute hand positions every minute
      tft.drawLine(ohx, ohy, CX, CY, TFT_BLACK);
      tft.fillCircle(ohx, ohy, RH, TFT_BLACK);
      tft.drawLine(omx, omy, CX, CY, TFT_BLACK);
      tft.fillCircle(omx, omy, RM, TFT_BLACK);
      ohx = cylinderX(CX, (RX - WH), hdeg); ohy = cylinderY(CY, (RY - WH), hdeg);
      omx = cylinderX(CX, (RX - WM), mdeg); omy = cylinderY(CY, (RY - WM), mdeg);

      boolean fAM;
      if (HMconv(now.hour(), now.minute()) <= HMconv(12, 0)) fAM = true; else fAM = false;
      for (int i = 0; i < N_TIME_SLOT; i++) {
        int tm = time_slot_start[i];
        if (fAM == false) tm -= HMconv(12, 0);
        float mdeg_s = (tm % 60) * 6;
        float hdeg_s = (tm / 60) * 30 + mdeg_s * (1.0 / 12.0);
        float mdeg_e = ((tm + TIME_SLOT_LENGTH) % 60) * 6;
        float hdeg_e = ((tm + TIME_SLOT_LENGTH) / 60) * 30 + mdeg_e * (1.0 / 12.0);
#define PIE_STEP 1
        for (float th = hdeg_s; th < hdeg_e; th += PIE_STEP) {
          tft.drawLine(cylinderX(CX, (RX - WZ1), th), cylinderY(CY, (RY - WZ1), th), cylinderX(CX, (RX - WZ2), th), cylinderY(CY, (RY - WZ2), th), time_slot_color[i]);
          tft.fillTriangle(cylinderX(CX, (RX - WZ1), th), cylinderY(CY, (RY - WZ1), th),
                           cylinderX(CX, (RX - WZ2), th), cylinderY(CY, (RY - WZ2), th),
                           cylinderX(CX, (RX - WZ2), th + PIE_STEP), cylinderY(CY, (RY - WZ2), th + PIE_STEP),
                           time_slot_color[i]);
          tft.fillTriangle(cylinderX(CX, (RX - WZ1), th), cylinderY(CY, (RY - WZ1), th),
                           cylinderX(CX, (RX - WZ2), th + PIE_STEP), cylinderY(CY, (RY - WZ2), th + PIE_STEP),
                           cylinderX(CX, (RX - WZ2), th + PIE_STEP), cylinderY(CY, (RY - WZ2), th + PIE_STEP),
                           time_slot_color[i]);

        }
      }

      // Redraw new hand positions, hour and minute hands not erased here to avoid flicker
      tft.drawLine(ohx, ohy, CX, CY, COLOR_H);
      tft.drawCircle(ohx, ohy, RH, COLOR_H);
      tft.drawLine(omx, omy, CX, CY, COLOR_M);
      tft.drawCircle(omx, omy, RM, COLOR_M);
      tft.fillCircle(CX, CY, 3, TFT_RED);
    }

    /*
        // for rotation(2), 90deg
        int xpos = 20;
        int ypos = 250;
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        xpos += tft.drawNumber(now.month(), xpos, ypos, 6);
        xpos += tft.drawChar('-', xpos, ypos, 6);
        xpos += tft.drawNumber(now.day(), xpos, ypos, 6);
        xpos += 10;
        switch (now.dayOfTheWeek()) {
          case 0 : tft.setTextColor(TFT_RED, TFT_BLACK); tft.drawString("Sun", xpos, ypos, 4); break;
          case 1 : tft.drawString("Mon", xpos, ypos, 4); break;
          case 2 : tft.drawString("Tue", xpos, ypos, 4); break;
          case 3 : tft.drawString("Wed", xpos, ypos, 4); break;
          case 4 : tft.drawString("Thu", xpos, ypos, 4); break;
          case 5 : tft.drawString("Fri", xpos, ypos, 4); break;
          case 6 : tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.drawString("Sat", xpos, ypos, 4); break;
        }
    */
    // for ration(0), 0deg
    int xpos = 250;
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawNumber(now.month(), xpos, 50, 6);
    tft.drawNumber(now.day(), xpos, 100, 6);
    int ypos = 160;
    switch (now.dayOfTheWeek()) {
      case 0 : tft.setTextColor(TFT_RED, TFT_BLACK); tft.drawString("Sun", xpos, ypos, 4); break;
      case 1 : tft.drawString("Mon", xpos, ypos, 4); break;
      case 2 : tft.drawString("Tue", xpos, ypos, 4); break;
      case 3 : tft.drawString("Wed", xpos, ypos, 4); break;
      case 4 : tft.drawString("Thu", xpos, ypos, 4); break;
      case 5 : tft.drawString("Fri", xpos, ypos, 4); break;
      case 6 : tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.drawString("Sat", xpos, ypos, 4); break;
    }
    /*
        Serial.print(now.year(), DEC);
        Serial.print('/');
        Serial.print(now.month(), DEC);
        Serial.print('/');
        Serial.print(now.day(), DEC);
        Serial.print(" ");
        Serial.print(now.hour(), DEC);
        Serial.print(':');
        Serial.print(now.minute(), DEC);
        Serial.print(':');
        Serial.print(now.second(), DEC);
        Serial.println();
    */

    // NTP adjust at 00:00 every day
//    Serial.print(fNTPadjusted); Serial.print(' '); Serial.println(now.minute());
    if (now.hour() == 0 && now.minute() == 0 && fNTPadjusted == false) {
      NTPadjustH = now.hour(); NTPadjustM = now.minute();
      NTPadjust();
      targetTime = millis();
      DrawClockBase();
      fNTPadjusted = true;
    }
    else if (fNTPadjusted == true && now.hour() != NTPadjustH && now.minute() != NTPadjustM) {
      fNTPadjusted = false;
    }
  }
}
