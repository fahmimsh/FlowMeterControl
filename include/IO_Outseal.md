/* #include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
const char* ssid = "fahmimsh";
const char* password = "tanyadulu";
#define TEST_IO 1
#define FLIPFLOP 2 */
#define ON LOW
#define OFF HIGH
/* OUTSEAL INPUT PIN INTTERRUPT */
static const uint8_t X0 = 27;
/* OUTSEAL INPUT PIN INTTERRUPT */
static const uint8_t X1 = 26;
/* OUTSEAL INPUT PIN INTTERRUPT */
static const uint8_t X2 = 33;
/* OUTSEAL INPUT PIN INTTERRUPT */
static const uint8_t X3 = 32;
/* OUTSEAL INPUT PIN INTTERRUPT */
static const uint8_t X4 = 35;
/* OUTSEAL INPUT PIN INTTERRUPT */
static const uint8_t X5 = 34;
/* OUTSEAL INPUT PIN NO INTERRUPT */
static const uint8_t X6 = 39;
/* OUTSEAL INPUT PIN NO INTERRUPT */
static const uint8_t X7 = 36;

/* OUTSEAL OUPUT PIN Y0-Y5 */
static const uint8_t Y0 = 15;
/* OUTSEAL OUPUT PIN Y0-Y5 */
static const uint8_t Y1 = 2;
/* OUTSEAL OUPUT PIN Y0-Y5 */
static const uint8_t Y2 = 0;
/* OUTSEAL OUPUT PIN Y0-Y5 */
static const uint8_t Y3 = 4;
/* OUTSEAL OUPUT PIN Y0-Y5 */
static const uint8_t Y4 = 13;
/* OUTSEAL OUPUT PIN Y0-Y5 */
static const uint8_t Y5 = 14;
/* OUTSEAL DIGITAL IO COMMUNICATION i2c*/
static const uint8_t PIN_SDA = 21;
/* OUTSEAL DIGITAL IO COMMUNICATION i2c*/
static const uint8_t PIN_SCL = 22;
/* OUTSEAL DIGITAL IO COMMUNICATION Serial0*/
static const uint8_t RX0 = 3;
/* OUTSEAL DIGITAL IO COMMUNICATION Serial0*/
static const uint8_t TX0 = 1;

/* OUTSEAL DIGITAL IO SPI COMMUNICATION CS SD CARD*/
static const uint8_t CS_SD = 5;
/* OUTSEAL DIGITAL IO SPI COMMUNICATION CS 1*/
static const uint8_t CS1 = 12;
/* OUTSEAL DIGITAL IO SPI COMMUNICATION CS 2*/
static const uint8_t CS2 = 25;
/* OUTSEAL DIGITAL IO SPI COMMUNICATION MISO*/
static const uint8_t PIN_MISO = 19;
/* OUTSEAL DIGITAL IO SPI COMMUNICATION MOSI*/
static const uint8_t PIN_MOSI = 23;
/* OUTSEAL DIGITAL IO SPI COMMUNICATION SCK*/
static const uint8_t PIN_SCK = 18;
/* Definisi input array untuk inisialisasi pin pada mode setup program*/
static const uint8_t pin_input[8] = {X0, X1, X2, X3, X4, X5, X6, X7};
/* Definisi output array untuk inisialisasi pin pada mode setup program*/
static const uint8_t pin_output[6] = {Y0, Y1, Y2, Y3, Y4, Y5};

/* void otainit(bool flagota){
    if(flagota == true){
        WiFi.begin(ssid, password);
        uint32_t notConnectedCounter = 0;
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(100);
            notConnectedCounter++;
            if(notConnectedCounter > 150) { // Reset board if not connected after 5s
                ESP.restart();
            }
        }
        ArduinoOTA.onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else // U_SPIFFS
                type = "filesystem";

            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type);
            })
            .onEnd([]() {
            Serial.println("\nEnd");
            })
            .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
            })
            .onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR) Serial.println("End Failed");
            });
        ArduinoOTA.begin();
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    }
}
void otahandle(bool flagota){
    if(flagota == true){
        ArduinoOTA.handle();
    }
} */
void init_IO(){
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(pin_input[i], INPUT);
  }
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(pin_output[i], OUTPUT);
    digitalWrite(pin_output[i], OFF);
  }
}

/* int INX(uint8_t pinsi){
    return digitalRead(pinsi);
}

void OUTY(uint8_t pinso, bool onoff){
    digitalWrite(pinso, onoff);
}
uint8_t ff_io = 0;
unsigned long ff_time = 0;
void demo_io(uint8_t mode_io){
    switch (mode_io)
    {
    case 1:
        if(INX(X0) == ON){
            OUTY(Y0, ON);
        }else if(INX(X0) == OFF){
            OUTY(Y0, OFF);
        }
        if(INX(X1) == ON){
            OUTY(Y1, ON);
        }else if(INX(X1) == OFF){
            OUTY(Y1, OFF);
        }
        if(INX(X2) == ON){
            OUTY(Y2, ON);
        }else if(INX(X2) == OFF){
            OUTY(Y2, OFF);
        }
        if(INX(X3) == ON){
            OUTY(Y3, ON);
        }else if(INX(X3) == OFF){
            OUTY(Y3, OFF);
        }
        if(INX(X4) == ON){
            OUTY(Y4, ON);
        }else if(INX(X4) == OFF){
            OUTY(Y4, OFF);
        }
        if(INX(X5) == ON){
            OUTY(Y5, ON);
        }else if(INX(X5) == OFF){
            OUTY(Y5, OFF);
        }
        if(INX(X6) == ON){
            OUTY(Y0, ON);
            OUTY(Y1, ON);
            OUTY(Y2, ON);
        }else if(INX(X6) == OFF){
            OUTY(Y0, OFF);
            OUTY(Y1, OFF);
            OUTY(Y2, OFF);
        }
        if(INX(X7) == ON){
            OUTY(Y3, ON);
            OUTY(Y4, ON);
            OUTY(Y5, ON);
        }else if(INX(X7) == OFF){
            OUTY(Y3, OFF);
            OUTY(Y4, OFF);
            OUTY(Y5, OFF);
        }
        break;
    case 2:
        if(millis() - ff_time > 1000){
            digitalWrite(pin_output[ff_io], OFF);
            ff_time = millis();
            ff_io += 1;
            if(ff_io > 6){
                ff_io = 0;
                digitalWrite(pin_output[0], OFF);
                digitalWrite(pin_output[1], OFF);
                digitalWrite(pin_output[2], OFF);
                digitalWrite(pin_output[3], OFF);
                digitalWrite(pin_output[4], OFF);
                digitalWrite(pin_output[5], OFF);
            }
            if (ff_io == 6)
            {
                digitalWrite(pin_output[0], ON);
                digitalWrite(pin_output[1], ON);
                digitalWrite(pin_output[2], ON);
                digitalWrite(pin_output[3], ON);
                digitalWrite(pin_output[4], ON);
                digitalWrite(pin_output[5], ON);
            }
            
            digitalWrite(pin_output[ff_io], ON);
        }
        break;
    }
} */