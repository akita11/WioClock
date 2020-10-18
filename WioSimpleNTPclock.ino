#include "RTC_SAMD51.h"
#include "DateTime.h"
#include <AtWiFi.h>
#include <time.h>

// Note: need before use:
// - install some libraries related to WiFi:
//   https://qiita.com/jksoft/items/cb11eb171002c0ed1f25
// - Wio Termial update: 
// - https://qiita.com/jksoft/items/cb11eb171002c0ed1f25

// Reference:
// NTP client: https://beta-notes.way-nifty.com/blog/2020/08/post-3484c0.html

#define WIFI_SSID     "YOUR_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

#define NTP_SERVER "210.173.160.27" // ntp1.jst.mfeed.ad.jp 

const int NTP_PACKET_SIZE = 48;

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
  return(epoch);
}

RTC_SAMD51 rtc;
void setup()
{
  rtc.begin();
  Serial.begin(115200);

  while (!Serial)
  {
    ;
  }

  Serial.print("connecting WiFi:"); Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { Serial.print('.'); delay(500); } 
  Serial.println();
  Serial.printf("Connected, IP address: ");
  Serial.println(WiFi.localIP());

  //    configTzTime("JST-9", "ntp.nict.jp", "ntp.jst.mfeed.ad.jp");   // currently not available in Wio Terminal

  DateTime now = DateTime(get_ntp_time());
  rtc.adjust(now);
}

void loop()
{
  DateTime now;
  now = rtc.now();
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
  delay(1000);
}
