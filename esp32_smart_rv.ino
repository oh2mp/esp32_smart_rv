/*
    WiFi configurable ESP32 Smart RV

    See https://github.com/oh2mp/esp32_smart_rv

*/

// #include <FreeRTOS.h>
#include <WiFi.h>
#include <WebServer.h>
#include "FS.h"
#include <LITTLEFS.h>
#include <time.h>
#include "TFT_eSPI.h"
TFT_eSPI tft = TFT_eSPI();
#define TFTW 240 // tft width
#define TFTH 320 // tft height

#include "tftfonts.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define BUTTON 12                // push button for lightness and long press starts portal
#define APTIMEOUT 120000         // Portal timeout. Reboot after ms if no activity.

#define MAX_TAGS 12
#define BLLED 19                 // backlight led
#define TFT_LOGOCOLOR 0x4228     // rgb565 for #444444
//   nice converter: http://www.rinkydinkelectronics.com/calc_rgb565.php

// Tag type enumerations and names
#define TAG_RUUVI     1
#define TAG_MIJIA     2
#define TAG_ENERGY    3
#define TAG_WATER     4
#define TAG_FLAME     5
#define TAG_DS1820    6
#define TAG_DHT       7
#define TAG_WATTSON   8
#define TAG_MOPEKA    9
#define TAG_IBSTH2   10
#define TAG_ALPICOOL 11

const char type_name[12][10] PROGMEM = {"", "\u0550UUVi", "ATC_Mi", "Energy", "Water", "Flame", "DS18x20", "DHTxx", "Wattson", "Mopeka\u2713", "IBS-TH2", "Alpicool"};

// end of tag type enumerations and names

char tagdata[MAX_TAGS][32];      // space for raw tag data unparsed
char tagname[MAX_TAGS][24];      // tag names
char tagmac[MAX_TAGS][18];       // tag macs
uint32_t tagtime[MAX_TAGS];      // time when a tag was heard last time
uint8_t tagtype[MAX_TAGS];       // "cached" value for tag type
uint8_t tagcount = 0;            // total amount of known tags
int screentag = 0;
int screenslot = 0;
char gattcache[32];              // Space for caching GATT payload

TaskHandle_t screentask = NULL;
TaskHandle_t buttontask = NULL;
TaskHandle_t gattcltask = NULL;

int tank_volume = 100;
int flame_threshold = 100;
char miscread[8];
uint8_t brightness = 10;
uint8_t last_brightness = 10;

volatile uint8_t buttonstate = 1;
volatile uint8_t oldbuttonstate = 1;
volatile uint32_t debounce = 0;

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

WebServer server(80);
IPAddress apIP(192, 168, 4, 1);                  // portal ip address
const char my_ip[] PROGMEM = "192.168.4.1";      // text to show for ip address
const char my_ssid[] PROGMEM = "ESP32 Smart RV"; // AP SSID
uint32_t portal_timer = 0;
uint32_t request_timer = 0;
uint32_t last_bri = 0;
uint32_t brightness_changed = 0;
bool is_scanning = false;

char heardtags[MAX_TAGS][18];
uint8_t heardtagtype[MAX_TAGS];

File file;
BLEScan* blescan;
BLEClient *pClient;

// RGB565 map for showing temperatures with colors like in weather maps.
const uint16_t tempcolors[] PROGMEM = {
    0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x009F, 0x011F, 0x019F, 0x021F, 0x029F, 0x031F, 0x039F, 0x041F, 0x049F, 0x051F,
    0x059F, 0x061F, 0x069F, 0x071F, 0x07FD, 0x07F9, 0x07F4, 0x07EF, 0x07EB, 0x07E6, 0x07E1, 0x17E0, 0x3FE0, 0x67E0, 0x87E0, 0xAFE0,
    0xD7E0, 0xF7E0, 0xFFA0, 0xFF60, 0xFF00, 0xFEC0, 0xFE60, 0xFE20, 0xFDC0, 0xFCA0, 0xFC40, 0xFC00, 0xFB80, 0xFB00, 0xFAC0, 0xFA60,
    0xFA20, 0xF9C0, 0xF980, 0xF920, 0xF8E0, 0xF880, 0xF840, 0xF800
};

// Mopeka definitions. Default values are valid for 11 kg gas bottles in Nordic and Baltic countries

int mopeka_cm[2] = { 35, 35 };
int mopeka_kg[2] = { 11, 11 };
int mopeka_unit = 0;             // 0 = cm, 1 = kg, 3 = %

/* ------------------------------------------------------------------------------- */
/* Get known tag index from MAC address. Format: 12:34:56:78:9a:bc */
uint8_t getTagIndex(const char *mac) {
    for (uint8_t i = 0; i < MAX_TAGS; i++) {
        if (strcmp(tagmac[i], mac) == 0) {
            return i;
        }
    }
    return 0xFF; // no tag with this mac found
}

/* ------------------------------------------------------------------------------- */
/* Find Alpicool fridge from known tags                                            */
uint8_t findAlpicool() {
    for (uint8_t i = 0; i < MAX_TAGS; i++) {
        if (tagtype[i] == TAG_ALPICOOL) {
            return i;
        }
    }
    return 0xFF; // not found
}
/* ------------------------------------------------------------------------------- */
/*  Detect tag type from payload and mac

    Ruuvi tags (Manufacturer ID 0x0499) with data format V5 only

    Homemade tags (Manufacturer ID 0x02E5 Espressif Inc)
     The sketches are identified by next two bytes after MFID.
     0xE948 water tank gauge https://github.com/oh2mp/esp32_watersensor/
     0x1A13 thermocouple sensor for gas flame https://github.com/oh2mp/esp32_max6675_beacon/
     0xACDC energy meter pulse counter https://github.com/oh2mp/esp32_energymeter
     0x1820 ds1820 beacon https://github.com/oh2mp/esp32_ds1820_ble

    Xiaomi Mijia thermometer with atc1441 custom firmware.
     https://github.com/atc1441/ATC_MiThermometer

    Mopeka✓ Gas tank level beacon
     https://www.mopeka.com/product-category/sensor/


*/

uint8_t tagTypeFromPayload(const uint8_t *payload, const uint8_t *mac) {
    // Has manufacturerdata? If so, check if this is known type.
    if (memcmp(payload, "\x02\x01\x06", 3) == 0 && payload[4] == 0xFF) {
        if (memcmp(payload + 5, "\x99\x04\x05", 3) == 0)      return TAG_RUUVI;
        if (memcmp(payload + 5, "\xE5\x02\xDC\xAC", 4) == 0)  return TAG_ENERGY;
        if (memcmp(payload + 5, "\xE5\x02\x48\xE9", 4) == 0)  return TAG_WATER;
        if (memcmp(payload + 5, "\xE5\x02\x13\x1A", 4) == 0)  return TAG_FLAME;
        if (memcmp(payload + 5, "\xE5\x02\x20\x18", 4) == 0)  return TAG_DS1820;
    }
    // Alpicool fridge?
    if (memcmp(payload, "\x02\x01\x06", 3) == 0 && memcmp(payload + 9, "ZHJIELI", 7) == 0) return TAG_ALPICOOL;
    // ATC_MiThermometer? The data should contain 10161a18 in the beginning and mac at offset 4.
    if (memcmp(payload, "\x10\x16\x1A\x18", 4) == 0 && memcmp(mac, payload + 4, 6) == 0) return TAG_MIJIA;
    // Mopeka gas tank sensor?
    if (memcmp(payload, "\x1A\xFF\x0D\x00", 4) == 0 && payload[26] == mac[5]) return TAG_MOPEKA;
    Serial.printf("%X %X\n", mac[5], payload[26]);

    return 0xFF; // unknown
}

/* ------------------------------------------------------------------------------- */
/*  Known devices callback
/* ------------------------------------------------------------------------------- */

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
        void onResult(BLEAdvertisedDevice advDev) {
            uint8_t payload[32];
            uint8_t taginx = getTagIndex(advDev.getAddress().toString().c_str());

            // we are interested about known and saved BLE devices only
            if (taginx == 0xFF || tagname[taginx][0] == 0) return;

            memset(payload, 0, 32);
            memcpy(payload, advDev.getPayload(), 32);
            
            // Don't we know the type of this device yet?
            if (tagtype[taginx] == 0) {
                uint8_t mac[6];
                tagtype[taginx] = tagTypeFromPayload(payload, mac);
            }
            // Inkbird IBS-TH2?
            if (tagtype[taginx] == 0xFF) {
                if (advDev.haveServiceUUID()) {
                    if (strcmp(advDev.getServiceUUID().toString().c_str(), "0000fff0-0000-1000-8000-00805f9b34fb") == 0) {
                        tagtype[taginx] = TAG_IBSTH2;
                    }
                }
            }
            // ignore if payload doesn't contain valid data.
            if (tagtype[taginx] == TAG_IBSTH2 &&
                memcmp(payload+7, "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0", 25) == 0) {
                return;
            }

            tagtime[taginx] = millis();
            // Copy the payload to tagdata
            memset(tagdata[taginx], 0, sizeof(tagdata[taginx]));
            memcpy(tagdata[taginx], payload, 32);
            if (tagtype[taginx] == TAG_ALPICOOL) {
                memcpy(tagdata[taginx], gattcache, 32);
            }

            Serial.printf("BLE callback: payload=");
            for (uint8_t i = 0; i < 32; i++) {
                Serial.printf("%02x", payload[i]);
            }
            Serial.printf("; ID=%d; type=%d; addr=%s; name=%s\n", taginx, tagtype[taginx], tagmac[taginx], tagname[taginx]);
        }
};

/* ------------------------------------------------------------------------------- */
/*  Alpicool callback
/* ------------------------------------------------------------------------------- */
static void alpicoolCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

    memset(gattcache,0,sizeof(gattcache));
    memcpy(gattcache,pData,length);

    short temperature = 0;
    short wanted = 0;

    temperature = (short)pData[18];
    wanted = (short)pData[8];
    Serial.print("Alpicool GATT payload=");
    for (uint8_t i = 0; i < length+1; i++) {
        Serial.printf("%02x", pData[i]);
    }
    if (pClient->isConnected()) pClient->disconnect();
}

/* ------------------------------------------------------------------------------- */
/*  Find new devices when portal is started
/* ------------------------------------------------------------------------------- */
class ScannedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
        void onResult(BLEAdvertisedDevice advDev) {
            uint8_t payload[32];
            uint8_t taginx = getTagIndex(advDev.getAddress().toString().c_str());

            Serial.printf("Heard %s %s\nPayload: ", advDev.toString().c_str(), advDev.getName().c_str());

            memcpy(payload, advDev.getPayload(), 32);
            for (uint8_t i = 0; i < 32; i++) {
                Serial.printf("%02x", payload[i]);
            }
            Serial.printf("\n");

            // skip known tags, we are trying to find new
            if (taginx != 0xFF) return;

            /*  we are interested only about known device types */
            uint8_t mac[6];
            memcpy(mac, advDev.getAddress().getNative(), 6);
            uint8_t htype = tagTypeFromPayload(payload, mac);

            // Check if this is Inkbird IBS-TH2
            if (htype == 0xFF) {
                if (advDev.haveServiceUUID()) {
                    if (strcmp(advDev.getServiceUUID().toString().c_str(), "0000fff0-0000-1000-8000-00805f9b34fb") == 0) {
                        htype = TAG_IBSTH2;
                    }
                }
            }

            if (htype != 0xFF && htype != 0) {
                for (uint8_t i = 0; i < MAX_TAGS; i++) {
                    if (strlen(heardtags[i]) == 0) {
                        strcpy(heardtags[i], advDev.getAddress().toString().c_str());
                        heardtagtype[i] = htype;
                        Serial.printf("Heard new tag: %s %s\n", heardtags[i], type_name[htype]);
                        break;
                    }
                }
            } else {
                Serial.print("Ignoring unsupported device.\n");
            }
        }
};

/* ------------------------------------------------------------------------------- */
void loadSavedTags() {
    char sname[25];
    char smac[18];
    for (uint8_t i = 0; i < MAX_TAGS; i++) {
        tagtime[i] = 0;
        tagtype[i] = 0;
        memset(tagname[i], 0, sizeof(tagname[i]));
        memset(tagdata[i], 0, sizeof(tagdata[i]));
    }

    if (LITTLEFS.exists("/littlefs/known_tags.txt")) {
        uint8_t foo = 0;
        file = LITTLEFS.open("/littlefs/known_tags.txt", "r", false);
        while (file.available()) {
            memset(sname, '\0', sizeof(sname));
            memset(smac, '\0', sizeof(smac));

            file.readBytesUntil('\t', smac, 18);
            file.readBytesUntil('\n', sname, 25);
            while (isspace(smac[strlen(smac) - 1]) && strlen(smac) > 0) smac[strlen(smac) - 1] = 0;
            while (isspace(sname[strlen(sname) - 1]) && strlen(sname) > 0) sname[strlen(sname) - 1] = 0;
            if (sname[strlen(sname) - 1] == 13) sname[strlen(sname) - 1] = 0;
            strcpy(tagmac[foo], smac);
            strcpy(tagname[foo], sname);
            foo++;
            if (foo >= MAX_TAGS) break;
            tagcount++;
        }
        file.close();
    }
}
/* ------------------------------------------------------------------------------- */
void IRAM_ATTR button_isr() {
    if (xTaskGetTickCountFromISR() - debounce < 50) return;
    portENTER_CRITICAL_ISR(&mux);
    buttonstate = digitalRead(BUTTON);
    if (buttonstate != oldbuttonstate) {
        oldbuttonstate = buttonstate;
        debounce = xTaskGetTickCountFromISR();
    }
    portEXIT_CRITICAL_ISR(&mux);
}
/* ------------------------------------------------------------------------------- */
void button_task( void * parameter) {

    pinMode(BUTTON, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(BUTTON), button_isr, CHANGE);

    uint8_t savestate = 1;
    char txt[8];

    while (1) {
        yield();
        if (portal_timer == 0 && buttonstate != savestate && debounce > 0) {
            savestate = buttonstate;
            if (buttonstate == LOW) {
                // Button pressed
                request_timer = millis();
            } else {
                // Button released
                brightness++;
                request_timer = 0;

                // Brightness changed, show it on screen
                // Disable other tasks to prevent collision with screentask.
                portENTER_CRITICAL_ISR(&mux);
                if (brightness > 10) brightness = 0;
                if (screenslot == 0) screenslot = 1;
                Serial.printf("brightness: %d\n", brightness);
                ledcWrite(0, brightness * 12 + 8);
                last_bri = millis();
                brightness_changed = millis();
                tft.fillRect(0, 0, TFTW, TFTH / 3 - 6, TFT_WHITE);
                tft.setTextDatum(TC_DATUM);
                tft.setTextColor(TFT_BLACK, TFT_WHITE);
                tft.loadFont(bigfont);
                sprintf(txt, "\x5C %d", brightness);
                tft.drawString(txt, int(TFTW / 2), 36);
                tft.unloadFont();
                last_brightness = brightness;
                portEXIT_CRITICAL_ISR(&mux);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
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

    Serial.begin(115200);
    Serial.println("\n\nESP32 Smart RV by OH2MP 2020-2024");

    LITTLEFS.begin();
    loadSavedTags();

    memset(gattcache,0,sizeof(gattcache));

    if (LITTLEFS.exists("/littlefs/misc.txt")) {
        file = LITTLEFS.open("/littlefs/misc.txt", "r", false);
        memset(miscread, '\0', sizeof(miscread));
        file.readBytesUntil('\n', miscread, 8);
        flame_threshold = atoi(miscread);
        Serial.printf("Flame threshold %d\n", flame_threshold);
        memset(miscread, '\0', sizeof(miscread));
        file.readBytesUntil('\n', miscread, 8);
        tank_volume = atoi(miscread);
        Serial.printf("Tank volume %d\n", tank_volume);
        memset(miscread, '\0', sizeof(miscread));
        file.readBytesUntil('\n', miscread, 8);
        if (strlen(miscread) > 0) brightness = atoi(miscread);
        Serial.printf("Brightness %d\n", brightness);
        file.close();
    } else {
        fce2();
    }
    if (LITTLEFS.exists("/littlefs/mopeka.txt")) {
        file = LITTLEFS.open("/littlefs/mopeka.txt", "r", false);
        memset(miscread, '\0', sizeof(miscread));
        file.readBytesUntil('\n', miscread, 8);
        mopeka_cm[0] = atoi(miscread);
        memset(miscread, '\0', sizeof(miscread));
        file.readBytesUntil('\n', miscread, 8);
        mopeka_kg[0] = atoi(miscread);
        memset(miscread, '\0', sizeof(miscread));
        file.readBytesUntil('\n', miscread, 8);
        mopeka_cm[1] = atoi(miscread);
        memset(miscread, '\0', sizeof(miscread));
        file.readBytesUntil('\n', miscread, 8);
        mopeka_kg[1] = atoi(miscread);
        memset(miscread, '\0', sizeof(miscread));
        file.readBytesUntil('\n', miscread, 8);
        mopeka_unit = atoi(miscread);
        memset(miscread, '\0', sizeof(miscread));
        file.close();
        if (mopeka_unit < 0 || mopeka_unit > 2)  mopeka_unit = 0;
        if (mopeka_cm[0] < 1)  mopeka_cm[0] = 35;
        if (mopeka_kg[0] < 1)  mopeka_kg[0] = 11;
        if (mopeka_cm[1] < 1)  mopeka_cm[1] = mopeka_cm[0];
        if (mopeka_kg[1] < 1)  mopeka_kg[1] = mopeka_kg[0];
    }

    BLEDevice::init("");
    blescan = BLEDevice::getScan();
    pClient = BLEDevice::createClient();

    blescan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    blescan->setActiveScan(true);
    blescan->setInterval(100);
    blescan->setWindow(99);

    // https://www.freertos.org/a00125.html
    xTaskCreate(screen_task, "screen", 4096, NULL, 2, &screentask);
    xTaskCreate(button_task, "button", 4096, NULL, 1, &buttontask);

    // Reset real time clock
    timeval epoch = {0, 0};
    const timeval *tv = &epoch;
    settimeofday(tv, NULL);
}

/* ------------------------------------------------------------------------------- */
void loop() {
    // Do tasks if portal mode is not active
    if (portal_timer == 0) {
        // Make a 9 second scan every 15 seconds.
        if (time(NULL) % 15 == 0) {
            while (pClient->isConnected()) delay(100);
            Serial.printf("============= start scan\n");
            is_scanning = true;
            BLEScanResults foundDevices = blescan->start(9, false);
            blescan->clearResults();
            delay(250);
            /*  Something is wrong if zero known tags is heard, so then reboot.
                Possible if all of them are out of range too, but that should not happen anyway.
            */
            if (foundDevices.getCount() == 0 && tagcount > 0) ESP.restart();
            Serial.printf("============= end scan\n");
            is_scanning = false;
        }

        // For make sure if button release is not detected. It sometimes happen.
        if (digitalRead(BUTTON) == HIGH && buttonstate == LOW) {
            buttonstate = HIGH;
            request_timer = 0;
        }
        // If the button has been pressed for 5 seconds, start portal. Be sure that button is really down.
        if (request_timer > 0 && millis() - request_timer > 5000 && digitalRead(BUTTON) == LOW) {
            startPortal();
        }
        // If brightness has changed over a minute ago, save the value.
        // This is a way to reduce LITTLEFS writes when we don't save it on every button push.
        if (brightness_changed != 0) {
            if (millis() - brightness_changed > 60000) {
                brightness_changed = 0;
                file = LITTLEFS.open("/littlefs/misc.txt", "w");
                file.printf("%d\n%d\n%d\n", flame_threshold, tank_volume, brightness);
                file.close();
                Serial.println("Brightness has changed. misc.txt saved.");
            }
        }
        // If not yet started and we have an Alpicool fridge, start the GATT task.
        if (gattcltask == nullptr && findAlpicool() != 0xFF) {
            xTaskCreate(gattcl_task, "gattcl", 4096, NULL, 3, &gattcltask);
        }
    } else {
        // Else the portal mode is active
        server.handleClient();

        if (millis() - portal_timer > APTIMEOUT) {
            Serial.println("Portal timeout. Booting.");
            delay(1000);
            ESP.restart();
        }
    }
}

/* ------------------------------------------------------------------------------- */
/*
    This task handles the screen updating
*/
void screen_task(void * param) {

    char displaytxt[24];
    short temperature = 0;
    int16_t x1, y1;
    uint16_t w, h;
    unsigned short humidity;
    unsigned short foo;
    float pressure;
    float voltage;
    uint32_t wh;
    int basey = 0;
    uint8_t mopeka_id = 0;

    while (1) {
        yield();
        // If portal mode is not active, do the tasks and show info
        if (strlen(tagname[screentag]) > 0 && portal_timer == 0) {

            memset(displaytxt, 0, sizeof(displaytxt));
            strcat(displaytxt, tagname[screentag]);

            displaytxt[15] = 0; // shorten name to fit (hopefully)
            basey = (TFTH / 3) * screenslot;
            tft.fillRect(0, basey, TFTW, TFTH / 3 - 3, TFT_BLACK);

            tft.loadFont(midfont);
            tft.setTextDatum(TC_DATUM);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.drawString(displaytxt, 120, basey);
            tft.unloadFont();

            // No data? Draw NaN-symbol like Ø
            if (tagdata[screentag][0] == 0 && tagdata[screentag][1] == 0) {
                tft.loadFont(bigfont);
                if (tagtime[screentag] == 0) {
                    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                    tft.drawString("\x23 \x24", int(TFTW / 2), basey + 36); // print hourglass and ear
                } else {
                    tft.setTextColor(TFT_RED, TFT_BLACK);
                    tft.drawString("\x27", int(TFTW / 2), basey + 38); // print Ø
                }
                tft.unloadFont();
            }

            // Ruuvi tags
            if (tagtype[screentag] == TAG_RUUVI) {
                tft.loadFont(bigfont);

                if (tagtime[screentag] == 0 || millis() - tagtime[screentag] > 300000) {
                    sprintf(displaytxt, "\x26"); // 0x26 = warning triangle sign in the bigfont
                    tft.setTextColor(TFT_RED, TFT_BLACK);
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 32);
                    tft.unloadFont();
                } else {
                    // This is Celsius. Only 7 of 195 countries in the World use Fahrenheit,
                    // so if you live in one of those developing countries, you must implement the conversion yourself.
                    temperature = ((short)tagdata[screentag][8] << 8) | (unsigned short)tagdata[screentag][9];

                    sprintf(displaytxt, "%.1f\x29", temperature * 0.005); // 0x29 = degree sign in the bigfont

                    int colinx = int(temperature * 0.005 + 20);
                    if (colinx < 0) {
                        colinx = 0;
                    }
                    if (colinx > 55) {
                        colinx = 55;
                    }
                    tft.setTextColor(tempcolors[colinx], TFT_BLACK);

                    tft.loadFont(bigfont);
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 32);
                    tft.unloadFont();
                    tft.setTextColor(TFT_WHITE, TFT_BLACK);

                    humidity = ((unsigned short)tagdata[screentag][10] << 8) | (unsigned short)tagdata[screentag][11];
                    foo = ((unsigned short)tagdata[screentag][20] << 8) + (unsigned short)tagdata[screentag][21];
                    voltage = ((double)foo / 32  + 1600) / 1000;
                    pressure = ((unsigned short)tagdata[screentag][12] << 8) + (unsigned short)tagdata[screentag][13] + 50000;
                    sprintf(displaytxt, "RH %.0f%%  %.2f V  %.0f hPa", (float)humidity * .0025, voltage, pressure / 100);

                    tft.loadFont(tinyfont);
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 80);
                    tft.unloadFont();

                    // Ruuvi logo
                    tft.fillCircle(TFTW - 17, basey + 50, 12, TFT_LOGOCOLOR);
                    tft.fillCircle(TFTW - 14, basey + 50, 7, TFT_BLACK);
                    tft.fillCircle(TFTW - 20, basey + 45, 4, TFT_LOGOCOLOR);
                }
            }
            // Other tags --------------------------------------------------------------------------------------
            // water gauge
            if (tagtype[screentag] == TAG_WATER) {
                tft.loadFont(bigfont);
                if (tank_volume > 0) {
                    // This is liters. If you are retarded and use gallons or buckets (pronounced bouquet) for measuring liquid volume, make your own glyph.
                    if (tagtime[screentag] == 0 || millis() - tagtime[screentag] > 300000) {
                        tft.drawString("??\x28", int(TFTW / 2), basey + 32); // 0x28 = liter symbol "script small L" in the bigfont
                    } else {
                        sprintf(displaytxt, "%d\x28", (unsigned int)tagdata[screentag][10]);
                        tft.drawString(displaytxt, int(TFTW / 2), basey + 32);
                    }
                    int baz = TFTW * (unsigned int)tagdata[screentag][10] / tank_volume - 2;
                    tft.drawRect(0, basey + 80, TFTW, 20, TFT_WHITE);
                    tft.fillRect(1, basey + 81, baz, 18, TFT_BLUE);
                } else {
                    tft.setTextColor(TFT_RED, TFT_BLACK);
                    tft.drawString("\x27", int(TFTW / 2), basey + 32); // 0x27 is big Ø symbol in the bigfont
                }
                tft.unloadFont();
            }
            // flame thermocouple
            if (tagtype[screentag] == TAG_FLAME) {
                // get the temperature
                foo = (((unsigned short)tagdata[screentag][10] << 8) + (unsigned short)tagdata[screentag][9]) >> 2;
                tft.loadFont(bigfont);
                if (foo > flame_threshold) {
                    tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
                    tft.drawString("\x3b", int(TFTW / 2), basey + 32); // 0x3b = flame symbol in the bigfont
                } else {
                    tft.setTextColor(TFT_RED, TFT_BLACK);
                    tft.drawString("\x26", int(TFTW / 2), basey + 32); // 0x27 is Ø symbol in the bigfont
                }
                tft.unloadFont();
                tft.loadFont(tinyfont);
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
                sprintf(displaytxt, "%d\xb0", foo); // 0xb0 is degree sign in standard ascii
                tft.drawString(displaytxt, int(TFTW / 2), basey + 80);
                tft.unloadFont();
            }
            // energy meter pulse counter
            if (tagtype[screentag] == TAG_ENERGY) {
                tft.loadFont(bigfont);
                tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
                wh = (((uint32_t)tagdata[screentag][16] << 24) + ((uint32_t)tagdata[screentag][15] << 16)
                      + ((uint32_t)tagdata[screentag][14] << 8) + (uint32_t)tagdata[screentag][13]);

                if (tagtime[screentag] == 0 || millis() - tagtime[screentag] > 300000) {
                    tft.setTextColor(TFT_RED, TFT_BLACK);
                    tft.drawString("\x26", int(TFTW / 2), basey + 36);
                } else {
                    sprintf(displaytxt, "%.1f \x3d", wh * 0.001); // 0x3D = kWh symbol
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 36);
                }
                tft.unloadFont();

                tft.loadFont(tinyfont);
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
                wh = (((uint32_t)tagdata[screentag][12] << 24) + ((uint32_t)tagdata[screentag][11] << 16)
                      + ((uint32_t)tagdata[screentag][10] << 8) + (uint32_t)tagdata[screentag][9]);

                sprintf(displaytxt, "\u03a3 %07.1f", wh * 0.001);  // 0x03A3 is capital sigma meaning total sum
                tft.drawString(displaytxt, int(TFTW / 2), basey + 80);
                tft.unloadFont();
            }
            // Xiaomi Mijia thermometer with ATC_MiThermometer custom firmware by atc1441
            if (tagtype[screentag] == TAG_MIJIA) {
                tft.loadFont(bigfont);

                if (tagtime[screentag] == 0 || millis() - tagtime[screentag] > 300000) {
                    sprintf(displaytxt, "\x26"); // 0x26 = warning triangle sign in the bigfont
                    tft.setTextColor(TFT_RED, TFT_BLACK);
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 32);
                    tft.unloadFont();
                } else {
                    temperature = ((short)tagdata[screentag][10] << 8) | (unsigned short)tagdata[screentag][11];

                    sprintf(displaytxt, "%.1f\x29", temperature * 0.1); // 0x29 = degree sign in the bigfont

                    int colinx = int(temperature * 0.1 + 20);
                    if (colinx < 0) {
                        colinx = 0;
                    }
                    if (colinx > 55) {
                        colinx = 55;
                    }
                    tft.setTextColor(tempcolors[colinx], TFT_BLACK);

                    tft.loadFont(bigfont);
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 32);
                    tft.unloadFont();
                    tft.setTextColor(TFT_WHITE, TFT_BLACK);

                    humidity = (unsigned short)tagdata[screentag][12];
                    voltage = ((short)tagdata[screentag][14] << 8) | (unsigned short)tagdata[screentag][15];
                    sprintf(displaytxt, "RH %.0f%%   %.2f V", (float)humidity, voltage / 1000);

                    tft.loadFont(tinyfont);
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 80);
                    tft.unloadFont();

                    // Xiaomi logo
                    tft.fillCircle(TFTW - 14, basey + 46, 4, TFT_LOGOCOLOR);
                    tft.fillCircle(TFTW - 18, basey + 50, 4, TFT_BLACK);
                    tft.fillRect(TFTW - 29, basey + 42, 16, 4, TFT_LOGOCOLOR);
                    tft.fillRect(TFTW - 29, basey + 42, 4, 20, TFT_LOGOCOLOR);
                    tft.fillRect(TFTW - 21, basey + 50, 4, 12, TFT_LOGOCOLOR);
                    tft.fillRect(TFTW - 13, basey + 48, 4, 14, TFT_LOGOCOLOR);
                    tft.fillRect(TFTW - 5, basey + 42, 4, 20, TFT_LOGOCOLOR);
                }
            }
            // ds1820 beacon
            if (tagtype[screentag] == TAG_DS1820) {
                tft.loadFont(bigfont);
                if (tagtime[screentag] == 0 || millis() - tagtime[screentag] > 300000) {
                    sprintf(displaytxt, "\x26"); // 0x26 = warning triangle sign in the bigfont
                    tft.setTextColor(TFT_RED, TFT_BLACK);
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 32);
                    tft.unloadFont();
                } else {
                    temperature = ((short)tagdata[screentag][10] << 8) | (unsigned short)tagdata[screentag][9];
                    sprintf(displaytxt, "%.1f\x29", temperature * 0.1); // 0x29 = degree sign in the bigfont

                    int colinx = int(temperature * 0.1 + 20);
                    if (colinx < 0) {
                        colinx = 0;
                    }
                    if (colinx > 55) {
                        colinx = 55;
                    }
                    tft.setTextColor(tempcolors[colinx], TFT_BLACK);

                    tft.loadFont(bigfont);
                    // there's no extra txt line with this type of beacon, we can draw temperature 8pix lower
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 40);
                    tft.unloadFont();
                }
            }
            // Mopeka✓ gas tank sensor
            if (tagtype[screentag] == TAG_MOPEKA) {
                tft.loadFont(bigfont);
                if (tagtime[screentag] == 0 || millis() - tagtime[screentag] > 300000) {
                    sprintf(displaytxt, "\x26"); // 0x26 = warning triangle sign in the bigfont
                    tft.setTextColor(TFT_RED, TFT_BLACK);
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 32);
                    tft.unloadFont();
                } else {
                    float displevel = 0;
                    uint8_t unitpixels = 0;    // midfont "%" width is 25px, "kg" is 30px and "cm" is 38px;
                    char unitstr[3] = "\0\0";
                    // This algorithm has been got from Mopeka Products, LLC.
                    uint8_t level = 0x35;
                    for (uint8_t i = 8; i < 27; i++) {
                        level ^= tagdata[screentag][i];
                    }

                    displevel = level * .762; // convert raw level to cm
                    if (displevel < 0) displevel = 0;
                    if (displevel > mopeka_cm[mopeka_id]) displevel = mopeka_cm[mopeka_id];
                    tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
                    if (displevel / mopeka_cm[mopeka_id] < 0.2) tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                    if (displevel / mopeka_cm[mopeka_id] < 0.1) tft.setTextColor(TFT_RED, TFT_BLACK);

                    switch (mopeka_unit) {
                        case 0:
                            sprintf(displaytxt, "\x3c  %d", int(displevel));     // \x3c is gas bottle symbol
                            unitpixels = 38;
                            strcpy(unitstr, "cm");
                            break;
                        case 1:
                            displevel = displevel / mopeka_cm[mopeka_id] * mopeka_kg[mopeka_id];
                            sprintf(displaytxt, "\x3c  %.1f", displevel);
                            unitpixels = 30;
                            strcpy(unitstr, "kg");
                            break;
                        case 2:
                            displevel = displevel / mopeka_cm[mopeka_id] * 100;
                            if (displevel > 100) displevel = 100;
                            sprintf(displaytxt, "\x3c  %d", int(displevel));
                            unitpixels = 25;
                            strcpy(unitstr, "%");
                            break;
                    }

                    uint16_t txtwidth = tft.drawString(displaytxt, int(TFTW / 2), -50);
                    uint16_t txtleft = int((TFTW - txtwidth - unitpixels - 4 ) / 2);
                    tft.setTextDatum(TL_DATUM);
                    tft.drawString(displaytxt, txtleft, basey + 32);
                    tft.unloadFont();

                    // draw unit with smaller font, midfont
                    tft.loadFont(midfont);
                    tft.drawString(unitstr, txtleft + txtwidth + 4, basey + 46);
                    tft.unloadFont();

                    tft.setTextDatum(TC_DATUM);
                    voltage = (float)tagdata[screentag][6] / 256.0f * 2.0f + 1.5f; // Mopeka specification
                    sprintf(displaytxt, "%.2f V", voltage);

                    tft.loadFont(tinyfont);
                    tft.setTextColor(TFT_WHITE, TFT_BLACK);
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 80);
                    tft.unloadFont();
                }
                mopeka_id++;
                if (mopeka_id > 1) mopeka_id = 0;
            }
            // Inkbird IBS-TH2
            if (tagtype[screentag] == TAG_IBSTH2) {
                tft.loadFont(bigfont);

                if (tagtime[screentag] == 0 || millis() - tagtime[screentag] > 300000) {
                    sprintf(displaytxt, "\x26"); // 0x26 = warning triangle sign in the bigfont
                    tft.setTextColor(TFT_RED, TFT_BLACK);
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 32);
                    tft.unloadFont();
                } else {
                    temperature = ((short)tagdata[screentag][15] << 8) | (unsigned short)tagdata[screentag][14];
                    temperature = round(temperature/10);
                    voltage = (short)tagdata[screentag][21]; // in Inkbird this is percentage, not voltage.
                    
                    sprintf(displaytxt, "%.1f\x29", temperature * 0.1); // 0x29 = degree sign in the bigfont

                    int colinx = int(temperature * 0.1 + 20);
                    if (colinx < 0) {
                        colinx = 0;
                    }
                    if (colinx > 55) {
                        colinx = 55;
                    }
                    tft.setTextColor(tempcolors[colinx], TFT_BLACK);

                    tft.loadFont(bigfont);
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 32);
                    tft.unloadFont();
                    tft.setTextColor(TFT_WHITE, TFT_BLACK);

                    sprintf(displaytxt, "%.0f%%", voltage);

                    tft.loadFont(tinyfont);
                    tft.drawString(displaytxt, int(TFTW / 2 + 30), basey + 80);
                    tft.unloadFont();
                    // Battery symbol. IBS-TH2 gives battery as percent, not voltage.
                    tft.fillRect(int(TFTW / 2 -34), basey + 80, 12, 13, TFT_WHITE);
                    tft.drawRect(int(TFTW / 2 -33), basey + 86, 8, 2, TFT_BLACK);
                    tft.drawRect(int(TFTW / 2 -20), basey + 86, 8, 2, TFT_WHITE);
                    tft.drawRect(int(TFTW / 2 -17), basey + 83, 2, 8, TFT_WHITE);
                    tft.drawSmoothRoundRect(int(TFTW / 2 -36), basey + 80, 3, 2, 28, 13, TFT_WHITE);
                    tft.fillRect(int(TFTW / 2 -7), basey + 84, 3, 6, TFT_WHITE);

                    // Inkbird logo
                    static uint8_t inkbird_bits[] = {
                         0x80, 0x00, 0x00, 0xc0, 0x03, 0x00, 0xe0, 0x03, 0x00, 0xf0, 0xc7, 0x00, 0xf8, 0xe3, 0x01, 0xfc,
                         0xf1, 0x03, 0xfe, 0xf8, 0x03, 0x7f, 0xfc, 0x03, 0x3f, 0xfe, 0x03, 0x1f, 0xff, 0x03, 0x1f, 0xff,
                         0x03, 0x1f, 0xfe, 0x03, 0xff, 0xe1, 0x03, 0xff, 0xe3, 0x03, 0xff, 0xe1, 0x03, 0xff, 0xf1, 0x03,
                         0xff, 0xf8, 0x03, 0x3f, 0xfc, 0x00, 0x3f, 0xfe, 0x00, 0x0f, 0x3f, 0x00,  0x8c, 0x3f, 0x00, 0x80,
                         0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x02, 0x00 };
                    tft.drawXBitmap(TFTW - 22, basey + 40, inkbird_bits, 18, 24, TFT_LOGOCOLOR);
                }
            }
            // Alpicool fridge
            if (tagtype[screentag] == TAG_ALPICOOL) {
                tft.loadFont(bigfont);

                if (tagdata[screentag][0] != 0xFE) tagtime[screentag] = 0; // no data yet
                if (tagtime[screentag] == 0 || millis() - tagtime[screentag] > 300000) {
                    sprintf(displaytxt, "\x26"); // 0x26 = warning triangle sign in the bigfont
                    tft.setTextColor(TFT_RED, TFT_BLACK);
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 32);
                    tft.unloadFont();
                } else {
                    temperature = (short)tagdata[screentag][18];
                    short wanted = (short)tagdata[screentag][8]; // wanted temperature
                    short ecomode = (short)tagdata[screentag][6]; // mode. 0 = MAX, 1 = ECO
                    
                    sprintf(displaytxt, "%d\x29", temperature); // 0x29 = degree sign in the bigfont

                    int colinx = int(temperature * 0.1 + 20);
                    if (colinx < 0) {
                        colinx = 0;
                    }
                    if (colinx > 55) {
                        colinx = 55;
                    }
                    tft.setTextColor(tempcolors[colinx], TFT_BLACK);

                    tft.loadFont(bigfont);
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 32);
                    tft.unloadFont();
                    tft.setTextColor(TFT_WHITE, TFT_BLACK);

                    if (ecomode == 0) {
                        sprintf(displaytxt, "%d°  MAX   %.1f V", wanted, (short)tagdata[screentag][20], (short)tagdata[screentag][21]);
                    } else {
                        sprintf(displaytxt, "%d°  ECO   %d.%d V", wanted, (short)tagdata[screentag][20], (short)tagdata[screentag][21]);
                    }

                    tft.loadFont(tinyfont);
                    tft.drawString(displaytxt, int(TFTW / 2), basey + 80);
                    tft.unloadFont();

                    // Alpicool logo
                    tft.drawWideLine(TFTW - 24, basey + 58, TFTW - 14, basey + 41, 3, TFT_LOGOCOLOR, TFT_LOGOCOLOR);
                    tft.drawWideLine(TFTW -  4, basey + 58, TFTW - 14, basey + 41, 3, TFT_LOGOCOLOR, TFT_LOGOCOLOR);
                    tft.drawWideLine(TFTW -  4, basey + 58, TFTW - 18, basey + 49, 3, TFT_LOGOCOLOR, TFT_LOGOCOLOR);
                }
            }

            if (basey < TFTH / 2) {
                tft.drawLine(0, basey + TFTH / 3 - 6, TFTW - 1, basey + TFTH / 3 - 6, TFT_LIGHTGREY);
            }
            screentag++;
            if (tagname[screentag][0] == 0 || screentag >= MAX_TAGS) {
                screentag = 0;
            }
            screenslot++;
            if (screenslot > 2) {
               screenslot = 0;
            }
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
        // Portal mode active. Show timer to timeout.
        yield();
        if (portal_timer > 0) {
            tft.loadFont(midfont);
            tft.fillRect(0, TFTH - 24, TFTW, 22, TFT_BLACK);
            tft.setCursor(TFTW / 2, TFTH - 24);
            if (WiFi.getMode() == WIFI_AP) {
                int secs = int(APTIMEOUT / 1000) - int((millis() - portal_timer) / 1000);
                sprintf(displaytxt, "%d:%02d", int(secs / 60), secs % 60);
            } else {
                sprintf(displaytxt, "%d", 11 - int((millis() - portal_timer) / 1000));
            }
            tft.drawString(displaytxt, TFTW / 2, TFTH - 24);
            tft.unloadFont();
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

/* ------------------------------------------------------------------------------- */
/*
    This task handles Alpicool fridge GATT client
*/
void gattcl_task(void *parameter) {
  char amac[18];
  memset(amac, 0, sizeof(amac));
  vTaskDelay(10000 / portTICK_PERIOD_MS); // sleep 10s in the beginning and give time for scan
  // wait until we really know if we have an Alpicool fridge
  while (findAlpicool() == 0xFF) vTaskDelay(1000 / portTICK_PERIOD_MS);
  memcpy(amac, tagmac[findAlpicool()], 18);

  if (amac[0] != 0) {
      BLEAddress serverAddress(amac);
      BLERemoteCharacteristic *rCharacteristic;
      BLERemoteCharacteristic *wCharacteristic;
      Serial.println("GATT task starting");

      while (1) {
        while (is_scanning) vTaskDelay(100 / portTICK_PERIOD_MS);
        if (!pClient->isConnected()) {
            pClient->connect(serverAddress);
            pClient->setMTU(512);
        }
        BLERemoteService *pRemoteService = pClient->getService("00001234-0000-1000-8000-00805F9B34FB");
        if (pRemoteService != nullptr) {
           rCharacteristic = pRemoteService->getCharacteristic("00001236-0000-1000-8000-00805F9B34FB");
           rCharacteristic->registerForNotify(alpicoolCallback);
           wCharacteristic = pRemoteService->getCharacteristic("00001235-0000-1000-8000-00805F9B34FB");
        }
        // Send query request
        // See: https://github.com/klightspeed/BrassMonkeyFridgeMonitor
        if (wCharacteristic != nullptr) wCharacteristic->writeValue({0xfe, 0xfe, 3, 1, 2, 0}, 6);
        vTaskDelay(60000 / portTICK_PERIOD_MS); // Once per minute is enough
      }
  } else vTaskDelay(10000 / portTICK_PERIOD_MS);
}


/* ------------------------------------------------------------------------------- */
/*  Portal code begins here

     Yeah, I know that String objects are pure evil 😈, but this is meant to be
     rebooted immediately after saving all parameters, so it is quite likely that
     the heap will not fragmentate yet.
*/
/* ------------------------------------------------------------------------------- */

void startPortal() {
    char displaytxt[128];

    // disable button
    portENTER_CRITICAL_ISR(&mux);
    vTaskDelete(buttontask);
    detachInterrupt(digitalPinToInterrupt(BUTTON));
    if (gattcltask != nullptr) vTaskDelete(gattcltask);
    if (pClient->isConnected()) pClient->disconnect();
    portEXIT_CRITICAL_ISR(&mux);

    // In portal mode force the backlight to be bright.
    ledcWrite(0, 128);

    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(TC_DATUM);
    tft.loadFont(bigfont);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("\x3e", int(TFTW / 2), int(TFTH * .33) - 25); // print config symbol
    tft.drawString("\x23   \x24", int(TFTW / 2), int(TFTH * .66) - 25); // print hourglass and ear

    Serial.print("Starting portal...");
    portal_timer = millis();

    for (uint8_t i = 0; i < MAX_TAGS; i++) {
        memset(heardtags[i], 0, sizeof(heardtags[i]));
    }
    Serial.print("\nListening 11 seconds for new tags...\n");

    // First listen 11 seconds to find new tags.
    blescan->setAdvertisedDeviceCallbacks(new ScannedDeviceCallbacks());
    blescan->setActiveScan(true);
    blescan->setInterval(100);
    blescan->setWindow(99);
    BLEScanResults foundDevices = blescan->start(11, false);
    blescan->stop();
    blescan->clearResults();
    blescan = NULL;
    BLEDevice::deinit(true);

    portal_timer = millis();
    tft.loadFont(bigfont);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.fillScreen(TFT_BLACK);
    tft.drawString("\x3e", int(TFTW / 2), 30); // print config symbol
    tft.unloadFont();

    delay(100);
    WiFi.disconnect();
    delay(100);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(my_ssid);
    delay(2000);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

    tft.setTextSize(2);
    sprintf(displaytxt, "WiFi SSID\n\n%s\n\n\n\nURL\n\nhttp://%s/", my_ssid, my_ip);
    tft.setCursor(0, 110);
    tft.print(displaytxt);

    server.on("/", httpRoot);
    server.on("/style.css", httpStyle);
    server.on("/misc.html", httpMisc);
    server.on("/savemisc", httpSaveMisc);
    server.on("/mopeka.html", httpMopeka);
    server.on("/savemopeka", httpSaveMopeka);
    server.on("/sensors.html", httpSensors);
    server.on("/savesens", httpSaveSensors);
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

    file = LITTLEFS.open("/littlefs/index.html", "r", false);
    html = file.readString();
    file.close();

    server.send(200, "text/html; charset=UTF-8", html);
}

/* ------------------------------------------------------------------------------- */

void httpSensors() {
    String html;
    String tablerows; //char tablerows[16384];
    char rowbuf[1024];
    int counter = 0;

    portal_timer = millis();

    file = LITTLEFS.open("/littlefs/sensors.html", "r", false);
    html = file.readString();
    file.close();

    loadSavedTags();

    for (int i = 0 ; i < MAX_TAGS; i++) {
        if (strlen(tagmac[i]) == 0) continue;

        sprintf(rowbuf, "<tr><td colspan=\"2\" id=\"mac%d\">%s</td></tr><tr>\n",
                counter, tagmac[i]);
        tablerows += String(rowbuf);
        sprintf(rowbuf, "<tr><td><input type=\"text\" id=\"sname%d\" name=\"sname%d\" maxlength=\"24\" value=\"%s\">",
                counter, counter, tagname[i]);
        tablerows += String(rowbuf);
        sprintf(rowbuf, "<input type=\"hidden\" id=\"saddr%d\" name=\"saddr%d\" value=\"%s\">", counter, counter, tagmac[i]);
        tablerows += String(rowbuf);
        if (counter > 0) {
            sprintf(rowbuf, "<td><a onclick=\"moveup(%d)\">\u2191</a></td></tr>\n", counter);
            tablerows += String(rowbuf);
        } else {
            tablerows += "<td></td></tr>\n";
        }
        counter++;
    }
    if (strlen(heardtags[0]) != 0 && counter < MAX_TAGS) {
        for (int i = 0; i < MAX_TAGS; i++) {
            if (strlen(heardtags[i]) == 0) continue;
            if (getTagIndex(heardtags[i]) != 0xFF) continue;

            sprintf(rowbuf, "<tr><td colspan=\"2\" id=\"mac%d\">%s &nbsp; %s</td></tr>\n",
                    counter, heardtags[i], type_name[heardtagtype[i]]);
            tablerows += String(rowbuf);
            sprintf(rowbuf, "<tr><td><input type=\"text\" id=\"sname%d\" name=\"sname%d\" maxlength=\"24\">", counter, counter);
            tablerows += String(rowbuf);
            sprintf(rowbuf, "<input type=\"hidden\" id=\"saddr%d\" name=\"saddr%d\" value=\"%s\">",
                    counter, counter, heardtags[i]);
            tablerows += String(rowbuf);
            if (counter > 0) {
                sprintf(rowbuf, "<td><a onclick=\"moveup(%d)\">\u2191</a></td></tr>\n", counter);
                tablerows += String(rowbuf);
            } else {
                tablerows += "<td></td></tr>\n";
            }
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

    file = LITTLEFS.open("/littlefs/known_tags.txt", "w");

    for (int i = 0; i < server.arg("counter").toInt(); i++) {
        if (server.arg("sname" + String(i)).length() > 0) {
            file.print(server.arg("saddr" + String(i)));
            file.print("\t");
            file.print(server.arg("sname" + String(i)));
            file.print("\n");
        }
    }
    file.close();
    loadSavedTags(); // reread

    file = LITTLEFS.open("/littlefs/ok.html", "r", false);
    html = file.readString();
    file.close();

    server.sendHeader("Refresh", "2;url=/");
    server.send(200, "text/html; charset=UTF-8", html);
}
/* ------------------------------------------------------------------------------- */

void httpStyle() {
    portal_timer = millis();
    String css;

    file = LITTLEFS.open("/littlefs/style.css", "r", false);
    css = file.readString();
    file.close();
    server.send(200, "text/css", css);
}
/* ------------------------------------------------------------------------------- */

void httpMisc() {
    portal_timer = millis();
    String html;

    file = LITTLEFS.open("/littlefs/misc.html", "r", false);
    html = file.readString();
    file.close();

    html.replace("###THR###", String(flame_threshold));
    html.replace("###TANKVOL###", String(tank_volume));

    server.send(200, "text/html; charset=UTF-8", html);
}
/* ------------------------------------------------------------------------------- */

void httpMopeka() {
    portal_timer = millis();
    String html;

    file = LITTLEFS.open("/littlefs/mopeka.html", "r", false);
    html = file.readString();
    file.close();

    html.replace("###CM0###", String(mopeka_cm[0]));
    html.replace("###CM1###", String(mopeka_cm[1]));
    html.replace("###KG0###", String(mopeka_kg[0]));
    html.replace("###KG1###", String(mopeka_kg[1]));
    switch (mopeka_unit) {
        case 0:
            html.replace("###CHKCM###", "checked");
            html.replace("###CHKKG###", "");
            html.replace("###CHKPC###", "");
            break;
        case 1:
            html.replace("###CHKKG###", "checked");
            html.replace("###CHKCM###", "");
            html.replace("###CHKPC###", "");
            break;
        case 2:
            html.replace("###CHKPC###", "checked");
            html.replace("###CHKCM###", "");
            html.replace("###CHKKG###", "");
            break;
    }

    server.send(200, "text/html; charset=UTF-8", html);
}
/* ------------------------------------------------------------------------------- */
void httpSaveMisc() {
    portal_timer = millis();
    String html;

    file = LITTLEFS.open("/littlefs/misc.txt", "w");
    file.printf("%s\n", server.arg("thr").c_str());
    file.printf("%s\n", server.arg("tankvol").c_str());
    file.printf("%d\n", brightness);
    file.close();

    // reread
    file = LITTLEFS.open("/littlefs/misc.txt", "r", false);
    memset(miscread, '\0', sizeof(miscread));
    file.readBytesUntil('\n', miscread, 8);
    flame_threshold = atoi(miscread);
    memset(miscread, '\0', sizeof(miscread));
    file.readBytesUntil('\n', miscread, 8);
    tank_volume = atoi(miscread);
    memset(miscread, '\0', sizeof(miscread));
    file.readBytesUntil('\n', miscread, 8);
    brightness = atoi(miscread);
    memset(miscread, '\0', sizeof(miscread));
    file.close();

    file = LITTLEFS.open("/littlefs/ok.html", "r", false);
    html = file.readString();
    file.close();

    server.sendHeader("Refresh", "2;url=/");
    server.send(200, "text/html; charset=UTF-8", html);
}

/* ------------------------------------------------------------------------------- */
void httpSaveMopeka() {
    portal_timer = millis();
    String html;

    file = LITTLEFS.open("/littlefs/mopeka.txt", "w");
    file.printf("%s\n", server.arg("cm0").c_str());
    file.printf("%s\n", server.arg("kg0").c_str());
    file.printf("%s\n", server.arg("cm1").c_str());
    file.printf("%s\n", server.arg("kg1").c_str());
    file.printf("%s\n", server.arg("unit").c_str());
    file.close();

    // reread
    file = LITTLEFS.open("/littlefs/mopeka.txt", "r", false);
    memset(miscread, '\0', sizeof(miscread));
    file.readBytesUntil('\n', miscread, 8);
    mopeka_cm[0] = atoi(miscread);
    memset(miscread, '\0', sizeof(miscread));
    file.readBytesUntil('\n', miscread, 8);
    mopeka_kg[0] = atoi(miscread);
    memset(miscread, '\0', sizeof(miscread));
    file.readBytesUntil('\n', miscread, 8);
    mopeka_cm[1] = atoi(miscread);
    memset(miscread, '\0', sizeof(miscread));
    file.readBytesUntil('\n', miscread, 8);
    mopeka_kg[1] = atoi(miscread);
    memset(miscread, '\0', sizeof(miscread));
    file.readBytesUntil('\n', miscread, 8);
    mopeka_unit = atoi(miscread);
    memset(miscread, '\0', sizeof(miscread));
    file.close();

    if (mopeka_unit < 0 || mopeka_unit > 2)  mopeka_unit = 0;
    if (mopeka_cm[0] < 1)  mopeka_cm[0] = 35;
    if (mopeka_kg[0] < 1)  mopeka_kg[0] = 11;
    if (mopeka_cm[1] < 1)  mopeka_cm[1] = mopeka_cm[0];
    if (mopeka_kg[1] < 1)  mopeka_kg[1] = mopeka_kg[0];

    file = LITTLEFS.open("/littlefs/ok.html", "r", false);
    html = file.readString();
    file.close();

    server.sendHeader("Refresh", "2;url=/");
    server.send(200, "text/html; charset=UTF-8", html);
}


/* ------------------------------------------------------------------------------- */
void httpBoot() {
    portal_timer = millis();
    String html;

    file = LITTLEFS.open("/littlefs/ok.html", "r", false);
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

    const char t[6][90] = {{
            0xa, 0x20, 0x20, 0x20, 0x20, 0x2a, 0x2a, 0x2a, 0x2a, 0x20, 0x43, 0x4f, 0x4d, 0x4d, 0x4f, 0x44, 0x4f, 0x52, 0x45, 0x20, 0x36, 0x34, 0x20, 0x42,
            0x41, 0x53, 0x49, 0x43, 0x20, 0x56, 0x32, 0x20, 0x2a, 0x2a, 0x2a, 0x2a, 0xa, 0xa, 0x20, 0x36, 0x34, 0x4b, 0x20, 0x52, 0x41, 0x4d, 0x20, 0x53, 0x59, 0x53, 0x54, 0x45, 0x4d,
            0x20, 0x20, 0x33, 0x38, 0x39, 0x31, 0x31, 0x20, 0x42, 0x41, 0x53, 0x49, 0x43, 0x20, 0x42, 0x59, 0x54, 0x45, 0x53, 0x20, 0x46, 0x52, 0x45, 0x45, 0xa, 0xa, 0x52, 0x45, 0x41,
            0x44, 0x59, 0x2e, 0x0a
        }, {
            0x4c, 0x4f, 0x41, 0x44, 0x20, 0x22, 0x4f, 0x48, 0x32, 0x4d, 0x50, 0x20, 0x53, 0x4d, 0x41, 0x52, 0x54, 0x20, 0x52, 0x56, 0x22, 0x2c, 0x38, 0x2c,
            0x31, 0x0a, 0x0a
        }, {
            0x53, 0x45, 0x41, 0x52, 0x43, 0x48, 0x49, 0x4e, 0x47, 0x20, 0x46, 0x4f, 0x52, 0x20, 0x4f, 0x48, 0x32, 0x4d, 0x50, 0x20, 0x53, 0x4d, 0x41, 0x52, 0x54,
            0x20, 0x52, 0x56, 0xa
        }, {0x4c, 0x4f, 0x41, 0x44, 0x49, 0x4e, 0x47, 0xa}, {0x52, 0x45, 0x41, 0x44, 0x59, 0x2e, 0xa}, {0x53, 0x59, 0x53, 0x36, 0x34, 0x37, 0x33, 0x38, 0xa}
    };
    const uint16_t d020[] = {0x0000, 0xffff, 0x69a5, 0x7536, 0x69f0, 0x5c68, 0x314f, 0xbe2d, 0x6a64, 0x41c0, 0x9b2b, 0x4228, 0x6b6d, 0x9e90, 0x6af6, 0x94b2};

    tft.setTextSize(1);
    tft.setTextColor(0x6af6);
    tft.fillScreen(0x6af6);
    tft.fillRect(0, 20, TFTW, TFTH / 2, 0x314f);
    tft.setCursor(0, 20);
    for (uint8_t i = 0; i < 6; i++) {
        tft.setCursor(0, tft.getCursorY());
        if (i == 1 || i == 5) {
            for (uint8_t j = 0; j < strlen(t[i]); j++) {
                tft.fillRect(tft.getCursorX(), tft.getCursorY(), 8, 8, 0x314f);
                tft.printf("%c", t[i][j]);
                if (t[i][j] != 0xa) {
                    tft.fillRect(tft.getCursorX(), tft.getCursorY(), 8, 8, 0x6af6);
                    delay(200);
                }
            }
        } else {
            tft.printf(t[i]);
            uint16_t c = 0x314f;
            if (t[i][strlen(t[i]) - 2] == 0x2E) c = 0x6af6;
            tft.fillRect(tft.getCursorX(), tft.getCursorY(), 8, 8, c);
            delay(2000);
        }
    }
    delay(2000);
    tft.setCursor(0, 20);
    for (uint16_t i = 0; i < 800; i++) {
        tft.setTextColor(d020[random(16)], d020[random(16)]);
        tft.printf("%c", random(0x20, 0x5F));
    }
    delay(1000);
    tft.fillScreen(0);
}
/* ------------------------------------------------------------------------------- */
