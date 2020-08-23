/*
 * WiFi configurable ESP32 Smart RV
 * 
 * See https://github.com/oh2mp/esp32_smart_rv
 *
 */

#include <FreeRTOS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <time.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include "TFT_eSPI.h"
TFT_eSPI tft = TFT_eSPI();
#define TFTW 240 // tft width
#define TFTH 320 // tft height

#include "strutils.h"
#include "tftfonts.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define BUTTON 12                // push button for lightness and long press starts portal
#define APTIMEOUT 180000         // Portal timeout. Reboot after ms if no activity.

#define MAX_TAGS 8
#define BLLED 19                 // backlight led

char tagdata[MAX_TAGS][25];      // space for raw tag data unparsed
char tagname[MAX_TAGS][24];      // tag names
char tagmac[MAX_TAGS][18];       // tag macs
int tagrssi[MAX_TAGS];           // RSSI for each tag
uint32_t tagtime[MAX_TAGS];      // time when a tag was heard last time

int screentag = 0;
TaskHandle_t screentask = NULL;

int tank_volume = 100;
int flame_threshold = 100;
char miscread[8];
uint8_t brightness = 64;

WebServer server(80);
IPAddress apIP(192,168,4,1);             // portal ip address
const char my_ip[] = "192.168.4.1";      // text to show for ip address
const char my_ssid[] = "ESP32 Smart RV"; // AP SSID
uint32_t portal_timer = 0;
uint32_t request_timer = 0;
uint32_t last_bri = 0;
uint8_t last_button = 1;
char heardtags[MAX_TAGS][18];

File file;
BLEScan* blescan;

// RGB565 map for showing temperatures with colors like in weather maps.
const uint16_t tempcolors[] = {
    0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x009F, 0x011F, 0x019F, 0x021F, 0x029F, 0x031F, 0x039F, 0x041F, 0x049F, 0x051F,
    0x059F, 0x061F, 0x069F, 0x071F, 0x07FD, 0x07F9, 0x07F4, 0x07EF, 0x07EB, 0x07E6, 0x07E1, 0x17E0, 0x3FE0, 0x67E0, 0x87E0, 0xAFE0,
    0xD7E0, 0xF7E0, 0xFFA0, 0xFF60, 0xFF00, 0xFEC0, 0xFE60, 0xFE20, 0xFDC0, 0xFCA0, 0xFC40, 0xFC00, 0xFB80, 0xFB00, 0xFAC0, 0xFA60,
    0xFA20, 0xF9C0, 0xF980, 0xF920, 0xF8E0, 0xF880, 0xF840, 0xF800
  };

/* ------------------------------------------------------------------------------- */
/* Get known tag index from MAC address. Format: 12:34:56:78:9a:bc */
uint8_t getTagIndex(const char *mac) {
    for (uint8_t i = 0; i < MAX_TAGS; i++) {
        if (strcmp(tagmac[i],mac) == 0) {
            return i;
        }
    }
    return 0xFF; // no tag with this mac found
}
/* ------------------------------------------------------------------------------- */

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advDev) {
        uint8_t taginx = getTagIndex(advDev.getAddress().toString().c_str());
        
        // we are interested about known and saved BLE devices only
        if (taginx == 0xFF) return;
        if (tagname[taginx][0] == 0) return;
        tagtime[taginx] = millis();
        tagrssi[taginx] = advDev.getRSSI();
        memset(tagdata[taginx],0,sizeof(tagdata[taginx]));
 
        // Ruuvi tags (Manufacturer ID 0x0499) with data format V5 only
        if (advDev.getManufacturerData()[0] == 0x99 && advDev.getManufacturerData()[1] == 4
            && advDev.getManufacturerData()[2] == 5) {

            for (uint8_t i = 0; i < sizeof(advDev.getManufacturerData()); i++) {
                tagdata[taginx][i] = advDev.getManufacturerData()[i];
            }
        }
        /* Homemade tags (Manufacturer ID 0x02E5 Espressif Inc)
         *   The sketches are identified by next two bytes after MFID.
         *   0xE948 water tank gauge https://github.com/oh2mp/esp32_watersensor/
         *   0x1A13 thermocouple sensor for gas flame https://github.com/oh2mp/esp32_max6675_beacon/
         *   0xACDC energy meter pulse counter 
         */
        uint16_t sid = advDev.getManufacturerData()[2] + advDev.getManufacturerData()[3] * 256;
        if (advDev.getManufacturerData()[0] == 0xE5 && advDev.getManufacturerData()[1] == 2) {
            if (sid == 0xE948 || sid == 0x1A13 || sid == 0xACDC) {
                memset(tagdata[taginx],0,sizeof(tagdata[taginx]));
                for (uint8_t i = 0; i < sizeof(advDev.getManufacturerData()); i++) {
                    tagdata[taginx][i] = advDev.getManufacturerData()[i];
                }
            }
        }
        Serial.printf("BLE callback: %d %s ",int(taginx),advDev.getAddress().toString().c_str());
        for (uint8_t i = 0; i < sizeof(tagdata[taginx]); i++) {
             Serial.printf("%02x",tagdata[taginx][i]);
        }
        Serial.print("\n");
    }
};

/* ------------------------------------------------------------------------------- */
class ScannedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advDev) {
        uint8_t taginx = getTagIndex(advDev.getAddress().toString().c_str());

        // skip known tags, we are trying to find new
        if (taginx != 0xFF) return;
        
        // we are interested only about Ruuvi tags (Manufacturer ID 0x0499)
        // and self made tags that have Espressif ID 0x02E5
        if ((advDev.getManufacturerData()[0] == 0x99 && advDev.getManufacturerData()[1] == 4)
            || (advDev.getManufacturerData()[0] == 0xE5 && advDev.getManufacturerData()[1] == 2)) {
            for (uint8_t i = 0; i < MAX_TAGS; i++) {
                 if (strlen(heardtags[i]) == 0) {
                     strcpy(heardtags[i],advDev.getAddress().toString().c_str());
                     Serial.printf("Heard new tag: %s\n",heardtags[i]);
                     break;
                 }
            }
        }
    }
};

/* ------------------------------------------------------------------------------- */
void loadSavedTags() {
    char sname[25];
    char smac[18];
    for (int i = 0; i < MAX_TAGS; i++) {
        tagtime[i] = 0;
        memset(tagname[i], 0, sizeof(tagname[i]));
        memset(tagdata[i], 0, sizeof(tagdata[i]));
    }

    if (SPIFFS.exists("/known_tags.txt")) {
        uint8_t foo = 0;
        file = SPIFFS.open("/known_tags.txt", "r");
        while (file.available()) {
            memset(sname, '\0', sizeof(sname));
            memset(smac, '\0', sizeof(smac));
            
            file.readBytesUntil('\t', smac, 18);
            file.readBytesUntil('\n', sname, 25);
            trimr(smac);
            trimr(sname);
            strcpy(tagmac[foo],smac);
            strcpy(tagname[foo],sname);
            foo++;
            if (foo >= MAX_TAGS) break;
        }
        file.close();
    }
}

/* ------------------------------------------------------------------------------- */
void setup() {
    tft.init();
    tft.fillScreen(TFT_BLACK);
    tft.setRotation(2);         // Rotate 180°. In my installation the TFT is "upside down"

    ledcSetup(0, 5000, 8);
    ledcAttachPin(BLLED, 0);
    ledcWrite(0, 128);
    
    pinMode(BUTTON, INPUT_PULLUP);

    Serial.begin(115200);
    Serial.println("\n\nESP32 Smart RV by OH2MP 2020");

    fce2();
    SPIFFS.begin();
    loadSavedTags();
    
    if (SPIFFS.exists("/misc.txt")) {
        file = SPIFFS.open("/misc.txt", "r");
        memset(miscread, '\0', sizeof(miscread)); 
        file.readBytesUntil('\n', miscread, 8);
        flame_threshold = atoi(miscread);        
        Serial.printf("Flame threshold %d\n",flame_threshold);
        memset(miscread, '\0', sizeof(miscread));
        file.readBytesUntil('\n', miscread, 8);
        tank_volume = atoi(miscread);        
        Serial.printf("Tank volume %d\n",tank_volume);       
        file.close();
    } else {
        BLEDevice::init("");
        blescan = BLEDevice::getScan();
        startPortal(); // no settings were found, so start the portal without button
    }

    BLEDevice::init("");
    blescan = BLEDevice::getScan();
    blescan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    blescan->setActiveScan(true);
    blescan->setInterval(100);
    blescan->setWindow(99);
    
    screentaskstart();

    // Reset real time clock
    timeval epoch = {0, 0};
    const timeval *tv = &epoch;
    settimeofday(tv, NULL);

    if (digitalRead(BUTTON) == LOW) {
        startPortal();   
    }
}

/* ------------------------------------------------------------------------------- */
// https://www.freertos.org/a00125.html

void screentaskstart() {
    xTaskCreate(next_tag_to_screen, "screen", 4096, NULL, 2, &screentask); 
}

/* ------------------------------------------------------------------------------- */
void loop() {
    struct tm* tm;
    time_t lt;
    time(&lt);
    tm = localtime(&lt);

    if (tm->tm_hour*60+tm->tm_min == 1 || tm->tm_hour + tm->tm_min + tm->tm_sec < 5) {

        if (tm->tm_sec == 0  && portal_timer == 0) {
            timeval epoch = {0, 0};      // A little "misuse" of realtime clock but we don't need it
            const timeval *tv = &epoch;
            settimeofday(tv, NULL);

            /* Current official Ruuvi firmware (v 2.5.9) https://lab.ruuvi.com/dfu/ broadcasts 
             * in every 6425ms in RAWv2 mode, so 9 seconds should be enough to hear all tags 
             * unless you have really many.
             */
            BLEScanResults foundDevices = blescan->start(9, false);
            blescan->clearResults();
            delay(250);
        }
    }
    
    if (portal_timer > 0) {     // are we in portal mode?
        server.handleClient();
    }

    if (digitalRead(BUTTON) == LOW) {
        if (last_button == HIGH) {
            char txt[8];
            brightness += 12;
            if (brightness > 120) brightness = 0;
            Serial.printf("bri: %d\n",brightness);
            ledcWrite(0, brightness+8);
            last_bri = millis();
            tft.fillRect(0,3,TFTW,TFTH/3-3,TFT_WHITE);
            tft.setTextDatum(TC_DATUM);
            tft.setTextColor(TFT_BLACK,TFT_WHITE);
            tft.loadFont(bigfont);
            sprintf(txt,"\x5C %d", brightness/12);
            tft.drawString(txt,int(TFTW/2),36);
            tft.unloadFont();
            last_button = LOW;           
        }
    
        if (millis() - portal_timer > 30000 && portal_timer > 0) ESP.restart();
        if (request_timer == 0) {
            request_timer = millis();
        }
    } else {
        request_timer = 0;
        last_button = HIGH;
    }
    if (request_timer > 0 && millis() - request_timer > 5000) {
        startPortal();
    }
    if (millis() - portal_timer > APTIMEOUT && portal_timer > 0) {
        Serial.println("Portal timeout. Booting.");
        delay(1000);
        ESP.restart();
    }
}

/* ------------------------------------------------------------------------------- */
void next_tag_to_screen(void * param) {
   
    char displaytxt[24];
    char mfdata[25];
    short temperature = 0;
    int16_t x1, y1;
    uint16_t w, h;
    unsigned short humidity;
    unsigned short foo;
    float pressure;
    float voltage;
    uint32_t wh;
    int basey = 0;
    
    while(portal_timer == 0) {
        if (strlen(tagname[screentag]) > 0) {
            memset(displaytxt,0,sizeof(displaytxt));
            strcat(displaytxt,tagname[screentag]);
            memcpy(mfdata,tagdata[screentag],sizeof(mfdata));
        
            displaytxt[15] = 0; // shorten name to fit (hopefully)
            basey = (TFTH / 3) * (screentag % 3) +3;
            tft.fillRect(0,basey,TFTW,TFTH/3-3,TFT_BLACK);

            tft.loadFont(midfont);
            tft.setTextDatum(TC_DATUM);
            tft.setTextColor(TFT_WHITE,TFT_BLACK);
            tft.drawString(displaytxt,120,basey);
            tft.unloadFont();
            
            // No data? Draw NaN-symbol like Ø
            if (mfdata[0] == 0 && mfdata[1] == 0) {
                tft.loadFont(bigfont);
                if (tagtime[screentag] == 0) {
                    tft.setTextColor(TFT_YELLOW,TFT_BLACK);
                    tft.drawString("\x23 \x24",int(TFTW/2),basey+36); // print hourglass and ear
                } else {
                    tft.setTextColor(TFT_RED,TFT_BLACK);
                    tft.drawString("\x27",int(TFTW/2),basey+38); // print Ø
                }
                tft.unloadFont(); 
            }
            
            // Ruuvi tags
            if (mfdata[0] == 0x99 && mfdata[1] == 4) {
                tft.loadFont(bigfont);
                
                if (tagtime[screentag] == 0 || millis() - tagtime[screentag] > 300000) {
                    sprintf(displaytxt,"\x26"); // 0x26 = warning triangle sign in the bigfont
                    tft.setTextColor(TFT_RED,TFT_BLACK);
                    tft.drawString(displaytxt,int(TFTW/2),basey+32);
                    tft.unloadFont();    
                } else {
                    // This is Celsius. Only 7 of 195 countries in the World use Fahrenheit,
                    // so if you live in one of those developing countries, you must implement the conversion yourself.
                    temperature = ((short)mfdata[3]<<8) | (unsigned short)mfdata[4];
                                  
                    sprintf(displaytxt,"%.1f\x29",temperature*0.005); // 0x29 = degree sign in the bigfont

                    int colinx = int(temperature*0.005+20);
                    if (colinx < 0) {colinx = 0;}
                    if (colinx > 55) {colinx = 55;}
                    tft.setTextColor(tempcolors[colinx],TFT_BLACK);
                
                    tft.loadFont(bigfont);
                    tft.drawString(displaytxt,int(TFTW/2),basey+32);
                    tft.unloadFont();
                    tft.setTextColor(TFT_WHITE,TFT_BLACK);
                
                    humidity = ((unsigned short)mfdata[5]<<8) | (unsigned short)mfdata[6];
                    foo = ((unsigned short)mfdata[15] << 8) + (unsigned short)mfdata[16];
                    voltage = ((double)foo / 32  + 1600)/1000;
                    pressure    = ((unsigned short)mfdata[7]<<8)  + (unsigned short)mfdata[8] + 50000;
                    sprintf(displaytxt,"RH %.0f%%  %.2f V  %.0f hPa",(float)humidity*.0025, voltage, pressure/100);
                
                    tft.loadFont(tinyfont);
                    tft.drawString(displaytxt,int(TFTW/2),basey+80);
                    tft.unloadFont();
                }
            }
            // Other tags which have manufacturer id 0x02E5 (Espressif)
            if (mfdata[0] == 0xE5 && mfdata[1] == 2) {
                // water gauge
                if (mfdata[2] == 0x48 && mfdata[3] == 0xE9) {
                    tft.loadFont(bigfont);
                    if (tank_volume > 0) {
                        // This is liters. If you are retarded and use gallons or buckets (pronounced bouquet) for measuring liquid volume, make your own glyph.
                        if (tagtime[screentag] == 0 || millis() - tagtime[screentag] > 300000) {
                            tft.drawString("??\x28",int(TFTW/2),basey+32); // 0x28 = liter symbol "script small L" in the bigfont
                        } else {
                            sprintf(displaytxt,"%d\x28",(unsigned int)mfdata[5]);
                            tft.drawString(displaytxt,int(TFTW/2),basey+32);
                        }
                        int baz = TFTW * (unsigned int)mfdata[5] / tank_volume -2;
                        tft.drawRect(0,basey+80,TFTW,20,TFT_WHITE);
                        tft.fillRect(1,basey+81,baz,18,TFT_BLUE);
                    } else {
                        tft.setTextColor(TFT_RED,TFT_BLACK);
                        tft.drawString("\x27",int(TFTW/2),basey+32); // 0x27 is big Ø symbol in the bigfont                      
                    }
                    tft.unloadFont();
                }
                // flame thermocouple
                if (mfdata[2] == 0x13 && mfdata[3] == 0x1A) {
                    // get the temperature
                    foo = (((unsigned short)mfdata[5] << 8) + (unsigned short)mfdata[4]) >> 2;
                    tft.loadFont(bigfont);
                    if (foo > flame_threshold) {
                        tft.setTextColor(TFT_SKYBLUE,TFT_BLACK);
                        tft.drawString("\x3b",int(TFTW/2),basey+32); // 0x3b = flame symbol in the bigfont
                    } else {
                        tft.setTextColor(TFT_RED,TFT_BLACK);
                        tft.drawString("\x26",int(TFTW/2),basey+32); // 0x27 is Ø symbol in the bigfont
                    }
                    tft.unloadFont();
                    tft.loadFont(tinyfont);
                    tft.setTextColor(TFT_WHITE,TFT_BLACK);
                    sprintf(displaytxt,"%d\xb0",foo);  // 0xb0 is degree sign in standard ascii
                    tft.drawString(displaytxt,int(TFTW/2),basey+80);
                    tft.unloadFont();
                }
                // energy meter pulse counter
                if (mfdata[2] == 0xDC && mfdata[3] == 0xAC) {
                    tft.loadFont(bigfont);
                    tft.setTextColor(TFT_SKYBLUE,TFT_BLACK);
                    wh = (((uint32_t)mfdata[11] << 24) + ((uint32_t)mfdata[10] << 16) + ((uint32_t)mfdata[9] << 8) + (uint32_t)mfdata[8]);

                    if (tagtime[screentag] == 0 || millis() - tagtime[screentag] > 300000) {
                        tft.setTextColor(TFT_RED,TFT_BLACK);
                        tft.drawString("\x26",int(TFTW/2),basey+36);
                    } else {
                        sprintf(displaytxt,"%.1f \x3d",wh*0.001); // 0x3D = kWh symbol
                        tft.drawString(displaytxt,int(TFTW/2), basey+36);
                    }
                    tft.unloadFont();

                    tft.loadFont(tinyfont);
                    tft.setTextColor(TFT_WHITE,TFT_BLACK);
                    wh = (((uint32_t)mfdata[7] << 24) + ((uint32_t)mfdata[6] << 16) + ((uint32_t)mfdata[5] << 8) + (uint32_t)mfdata[4]);
                    sprintf(displaytxt,"\u03a3 %07.1f",wh*0.001);      // 0x03A3 is capital sigma meaning total sum
                    tft.drawString(displaytxt,int(TFTW/2),basey+80);
                    tft.unloadFont();
                }
            }
            if (basey < TFTH/2) {
                tft.drawLine(0,basey+TFTH/3-6,TFTW-1,basey+TFTH/3-6,TFT_LIGHTGREY);
            }
            screentag++;
            if (tagname[screentag][0] == 0 || screentag >= MAX_TAGS) {screentag = 0;}
        }
        vTaskDelay(2500 / portTICK_PERIOD_MS);
    }
}
/* ------------------------------------------------------------------------------- */
/* Portal code begins here
 *  
 *   Yeah, I know that String objects are pure evil 😈, but this is meant to be
 *   rebooted immediately after saving all parameters, so it is quite likely that 
 *   the heap will not fragmentate yet. 
 */
/* ------------------------------------------------------------------------------- */

void startPortal() {
    char displaytxt[128];
    
    vTaskDelete(screentask);
    ledcWrite(0, 128);

    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(TC_DATUM);
    tft.loadFont(bigfont);
    tft.setTextColor(TFT_WHITE,TFT_BLACK);
    tft.drawString("\x3e",int(TFTW/2),int(TFTH*.33)-25); // print config symbol
    tft.drawString("\x23   \x24",int(TFTW/2),int(TFTH*.66)-25); // print hourglass and ear
    
    Serial.print("Starting portal...");
    portal_timer = millis();

    for (uint8_t i = 0; i < MAX_TAGS; i++) {
        memset(heardtags[i],0,sizeof(heardtags[i]));
    }
    Serial.print("\nListening 10 seconds for new tags...\n");
//    digitalWrite(LED, HIGH);

    // First listen 10 seconds to find new tags.
    blescan->setAdvertisedDeviceCallbacks(new ScannedDeviceCallbacks());
    blescan->setActiveScan(true);
    blescan->setInterval(100);
    blescan->setWindow(99);
    BLEScanResults foundDevices = blescan->start(10, false);
    blescan->clearResults();
    blescan->stop();
    blescan = NULL;
    BLEDevice::deinit(true);
//    digitalWrite(LED, LOW);

    WiFi.disconnect();
    delay(100);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(my_ssid);
    delay(2000);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

    tft.fillScreen(TFT_BLACK);
    tft.drawString("\x3e",int(TFTW/2),30); // print config symbol
    tft.unloadFont();
    
    tft.setTextSize(2);
    sprintf(displaytxt,"WiFi SSID\n\n%s\n\n\n\nURL\n\nhttp://%s/", my_ssid, my_ip);
    tft.setCursor(0,120);
    tft.print(displaytxt);
    
    server.on("/", httpRoot);
    server.on("/style.css", httpStyle);
    server.on("/misc.html", httpMisc);
    server.on("/savemisc", httpSaveMisc);
    server.on("/sensors.html", httpSensors);
    server.on("/savesens",httpSaveSensors);
    server.on("/boot", httpBoot);
    
    server.onNotFound([]() {
        server.sendHeader("Refresh", "1;url=/"); 
        server.send(404, "text/plain", "QSD QSY");
    });
    server.begin();
    Serial.println("Portal running.");
}
/* ------------------------------------------------------------------------------- */

void httpRoot() {
    portal_timer = millis();
    String html;
    
    file = SPIFFS.open("/index.html", "r");
    html = file.readString();
    file.close();    
    
    server.send(200, "text/html; charset=UTF-8", html);
}

/* ------------------------------------------------------------------------------- */

void httpSensors() {
    String html;
    char tablerows[2048];
    char rowbuf[256];
    int counter = 0;
    
    portal_timer = millis();
    memset(tablerows, '\0', sizeof(tablerows));
    
    file = SPIFFS.open("/sensors.html", "r");
    html = file.readString();
    file.close();

    loadSavedTags();
    
    for(int i = 0 ; i < MAX_TAGS; i++) {
        if (strlen(tagmac[i]) == 0) continue;
        
        sprintf(rowbuf,"<tr><td>%s<br /><input type=\"text\" name=\"sname%d\" maxlength=\"24\" value=\"%s\">",
                       tagmac[i],counter,tagname[i]);
        strcat(tablerows,rowbuf);                       
        sprintf(rowbuf,"<input type=\"hidden\" name=\"saddr%d\" value=\"%s\"></td></tr>",counter,tagmac[i]);
        strcat(tablerows,rowbuf);
        counter++;
    }
    if (strlen(heardtags[0]) != 0 && counter < MAX_TAGS) {
        for(int i = 0; i < MAX_TAGS; i++) {
            if (strlen(heardtags[i]) == 0) continue;
            if (getTagIndex(heardtags[i]) != 0xFF) continue;
            
            sprintf(rowbuf,"<tr><td>%s<br /><input type=\"text\" name=\"sname%d\" maxlength=\"24\">",
                    heardtags[i],counter);
            strcat(tablerows,rowbuf);
            sprintf(rowbuf,"<input type=\"hidden\" name=\"saddr%d\" value=\"%s\"></td></tr>",
                           counter,heardtags[i]);
            strcat(tablerows,rowbuf);
            counter++;
            if (counter > MAX_TAGS) break;
        }
    }

    html.replace("###TABLEROWS###", tablerows);
    html.replace("###COUNTER###", String(counter));
            
    server.send(200, "text/html; charset=UTF-8", html);
}
/* ------------------------------------------------------------------------------- */

void httpSaveSensors() {
    portal_timer = millis();
    String html;
        
    file = SPIFFS.open("/known_tags.txt", "w");
    
    for (int i = 0; i < server.arg("counter").toInt(); i++) {
         if (server.arg("sname"+String(i)).length() > 0) {
             file.print(server.arg("saddr"+String(i)));
             file.print("\t");
             file.print(server.arg("sname"+String(i)));
             file.print("\n");
         }
    }
    file.close();
    loadSavedTags(); // reread

    file = SPIFFS.open("/ok.html", "r");
    html = file.readString();
    file.close();

    server.sendHeader("Refresh", "2;url=/");
    server.send(200, "text/html; charset=UTF-8", html);
}
/* ------------------------------------------------------------------------------- */

void httpStyle() {
    portal_timer = millis();
    String css;

    file = SPIFFS.open("/style.css", "r");
    css = file.readString();
    file.close();       
    server.send(200, "text/css", css);
}
/* ------------------------------------------------------------------------------- */

void httpMisc() {
    portal_timer = millis();
    String html;
    
    file = SPIFFS.open("/misc.html", "r");
    html = file.readString();
    file.close();
    
    html.replace("###THR###", String(flame_threshold));
    html.replace("###TANKVOL###", String(tank_volume));
       
    server.send(200, "text/html; charset=UTF-8", html);
}
/* ------------------------------------------------------------------------------- */
void httpSaveMisc() {
    portal_timer = millis();
    String html;
        
    file = SPIFFS.open("/misc.txt", "w");
    file.printf("%s\n",server.arg("thr").c_str());
    file.printf("%s\n",server.arg("tankvol").c_str());
    file.close();

    // reread
    file = SPIFFS.open("/misc.txt", "r");
    memset(miscread, '\0', sizeof(miscread)); 
    file.readBytesUntil('\n', miscread, 8);
    flame_threshold = atoi(miscread);
    memset(miscread, '\0', sizeof(miscread));         
    file.readBytesUntil('\n', miscread, 8);
    tank_volume = atoi(miscread);        
    file.close();
    
    file = SPIFFS.open("/ok.html", "r");
    html = file.readString();
    file.close();

    server.sendHeader("Refresh", "2;url=/");
    server.send(200, "text/html; charset=UTF-8", html);    
}
/* ------------------------------------------------------------------------------- */
void httpBoot() {
    portal_timer = millis();
    String html;
    
    file = SPIFFS.open("/ok.html", "r");
    html = file.readString();
    file.close();
    
    server.sendHeader("Refresh", "2;url=about:blank");
    server.send(200, "text/html; charset=UTF-8", html);
    delay(1000);
    
    timeval epoch = {0, 0}; // clear clock
    const timeval *tv = &epoch;
    settimeofday(tv, NULL);

    ESP.restart();
}
/* ------------------------------------------------------------------------------- */
void fce2() {
    // Have fun :)
    const char t[6][90] = {{0xa,0x20,0x20,0x20,0x20,0x2a,0x2a,0x2a,0x2a,0x20,0x43,0x4f,0x4d,0x4d,0x4f,0x44,0x4f,0x52,0x45,0x20,0x36,0x34,0x20,0x42,
    0x41,0x53,0x49,0x43,0x20,0x56,0x32,0x20,0x2a,0x2a,0x2a,0x2a,0xa,0xa,0x20,0x36,0x34,0x4b,0x20,0x52,0x41,0x4d,0x20,0x53,0x59,0x53,0x54,0x45,0x4d,
    0x20,0x20,0x33,0x38,0x39,0x31,0x31,0x20,0x42,0x41,0x53,0x49,0x43,0x20,0x42,0x59,0x54,0x45,0x53,0x20,0x46,0x52,0x45,0x45,0xa,0xa,0x52,0x45,0x41,
    0x44,0x59,0x2e,0x0a},{0x4c,0x4f,0x41,0x44,0x20,0x22,0x4f,0x48,0x32,0x4d,0x50,0x20,0x53,0x4d,0x41,0x52,0x54,0x20,0x52,0x56,0x22,0x2c,0x38,0x2c,
    0x31,0x0a,0x0a},{0x53,0x45,0x41,0x52,0x43,0x48,0x49,0x4e,0x47,0x20,0x46,0x4f,0x52,0x20,0x4f,0x48,0x32,0x4d,0x50,0x20,0x53,0x4d,0x41,0x52,0x54,
    0x20,0x52,0x56,0xa},{0x4c,0x4f,0x41,0x44,0x49,0x4e,0x47,0xa},{0x52,0x45,0x41,0x44,0x59,0x2e,0xa},{0x53,0x59,0x53,0x36,0x34,0x37,0x33,0x38,0xa}};
    const uint16_t d020[] = {0x0000,0xffff,0x69a5,0x7536,0x69f0,0x5c68,0x314f,0xbe2d,0x6a64,0x41c0,0x9b2b,0x4228,0x6b6d,0x9e90,0x6af6,0x94b2};
   
    tft.setTextSize(1);
    tft.setTextColor(0x6af6);
    tft.fillScreen(0x6af6);
    tft.fillRect(0,20,TFTW,TFTH/2,0x314f);
    tft.setCursor(0,20);
    for (uint8_t i = 0; i < 6; i++) {
        tft.setCursor(0,tft.getCursorY());
        if (i == 1 || i == 5) {
            for (uint8_t j = 0; j < strlen(t[i]); j++) {
                 tft.fillRect(tft.getCursorX(),tft.getCursorY(),8,8,0x314f);
                 tft.printf("%c",t[i][j]);
                 if (t[i][j] != 0xa) {
                     tft.fillRect(tft.getCursorX(),tft.getCursorY(),8,8,0x6af6);
                     delay(200);
                 }
            }
        } else {
            tft.printf(t[i]);
            uint16_t c = 0x314f;
            if (t[i][strlen(t[i])-2] == 0x2E) c = 0x6af6;
            tft.fillRect(tft.getCursorX(),tft.getCursorY(),8,8,c);
            delay(2000);
        }
    }
    delay(2000);
    tft.setCursor(0,20);
    for (uint16_t i = 0; i < 800; i++) {
         tft.setTextColor(d020[random(16)],d020[random(16)]);
         tft.printf("%c",random(0x20,0x5F));
    }
    delay(1000);
    tft.fillScreen(0);
}
/* ------------------------------------------------------------------------------- */
