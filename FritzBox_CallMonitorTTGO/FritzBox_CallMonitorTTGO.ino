/*
 * FritzBox CallMonitor
 * --------------------
 * Copyright (C) 2021 by Ralph Rakers - rakete1@mail.de
 * Copyright (C) 2014 by Tobias Link - ranger81@dsuclan.de - www.ranger81.de
 *
 * This sketch can be used on a TTGO T-Display ESP32. It provides a simple
 * call monitoring functionality to see who is currently calling and who you are
 * going to call. Useful for phones without own display.
 *
 * Requirements:
 * - TTGO T-Display ESP32
 * - WiFi connection (onboard)
 * - 1.14-inch color screen, 135X240 resolution (onboard)
 * - AVM FritzBox with enabled CallMonitor support (dial #96*5* to enable)
 *
 * Last changes:
 * - 2021-02-16: Ported project to new hardware platform (TTGO T-Display)
 * - 2014-04-06: Added missed call counter and display of last missed call number on LCD
 * - 2014-04-05: Added missed call LED feature
 * - 2014-04-04: Added dim timeout for display backlight and proper time display after DISCONNECT
 * - 2014-04-03: Added timer for display how long the phone call was
 * - 2014-04-03: Initial internal release
 *
 * Open Topics / ToDo:
 */

/**** INCLUDES *********************************/
#include <WiFiManager.h>    /* needed for wifi setup via the esp internal wifi ap */
#include <WiFi.h>           /* neeeded for WiFiClient */
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include "Button2.h"
#include "esp_adc_cal.h"
/***********************************************/

/**** CONFIGURATION ****************************/
#define LCD_BACKLIGHT_PIN 9                // LCD backlight positive PIN
#define MISSED_CALL_LED_PIN A0             // LED notification if call was missed/not picked up
#define CLEAR_MISSED_CALL_BUTTON_PIN 8     // Attached button to clear missed calls
#define CLEAR_MISESED_CALL_ON_NEW_ACTION 0 // Clear missed call state if new action was done
#define FRITZBOX_HOSTNAME "fritz.box"      // Host name of FritzBox
#define FRITZBOX_PORT 1012                 // Port with running FritzBox call monitor
#define RETRY_TIMEOUT 5000                 // Retry connection to FB every x seconds
#define CALL_DURATION_UPDATE_INTERVAL 1000 // Update ongoing call duration on LCD
#define LCD_PIN_RS 7                       // Pin for initialization of LiquidCrystal library (RS)
#define LCD_PIN_ENABLE 6                   // Pin for initialization of LiquidCrystal library (ENABLE)
#define LCD_PIN_D4 5                       // Pin for initialization of LiquidCrystal library (D4)
#define LCD_PIN_D5 4                       // Pin for initialization of LiquidCrystal library (D5)
#define LCD_PIN_D6 3                       // Pin for initialization of LiquidCrystal library (D6)
#define LCD_PIN_D7 2                       // Pin for initialization of LiquidCrystal library (D7)
#define LCD_MAX_CHARS 16                   // LCD max chars in one line
#define LCD_ENABLE_DIM 1                   // If enabled, LCD backlight will only dim if unused. If disabled LCD backlight will turn off
#define LCD_DIM_TIMEOUT 10000              // Timeout for display dimmer after DISCONNECT
#define LCD_DIM_PWM_VALUE 10               // analogWrite PWM value for dimmed LCD backlight
#define DISPLAY_CALL_DURATION 1            // Enable or Disable display of call duration
#define DEBUG 1                            // Enable serial debugging
#define SERIAL_BAUD_RATE 115200            // Baud rate for serial communication
#define PRICEPERMINUTE 0.025               // Price per minute in Euro
#define CHECKCONNECTION 10000              // Milliseconds
//#define ENABLE_DISPLAY_TIME 1
/***********************************************/

/* crap from example */
#define ADC_EN 14 //ADC_EN is the ADC detection enable port
#define ADC_PIN 34
#define BUTTON_1 35
#define BUTTON_2 0

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library
Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

char buff[512];
int vref = 1100;
int btnCick = false;
/* crap from example END */

/**** GLOBAL VARIABLES *************************/
WiFiClient client;


unsigned long next;
unsigned long callstart;
unsigned long calllaststatus;
unsigned long lcdtimeoutstart;
unsigned long connectioncheck;

byte missedcallcount;
char *lastnumber;
char *lastmissednumber;

boolean call_connected;
boolean lcd_dimmer;
boolean lastcallwasmissedcall;
boolean showprice;
/***********************************************/

void setup()
{
    next = 0;
    call_connected = false;
    lcd_dimmer = false;
    showprice = false;

    missedcallcount = 0;
    lastcallwasmissedcall = 0;

    //pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
    //pinMode(CLEAR_MISSED_CALL_BUTTON_PIN, INPUT_PULLUP);
    //pinMode(MISSED_CALL_LED_PIN, OUTPUT);

    lcdon();
    lcdsplash();
    //lcdstartdim();

#ifdef DEBUG
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println(F("[FritzBox CallMonitor - www.ranger81.de]\n"));
    Serial.println(F("Starting Network..."));
#endif

    WiFi_Setup();

    lastnumber = (char *)malloc(LCD_MAX_CHARS + 1);
    lastmissednumber = (char *)malloc(LCD_MAX_CHARS + 1);

    /*
    ADC_EN is the ADC detection enable port
    If the USB port is used for power supply, it is turned on by default.
    If it is powered by battery, it needs to be set to high level
    */
    pinMode(ADC_EN, OUTPUT);
    digitalWrite(ADC_EN, HIGH);


    /* startup animation */
    tft.init();
    tft.setCursor(0, 0);
    tft.setRotation(3); // 3 = Landscape with buttons on the left
    tft.setTextSize(2);
    tft.setSwapBytes(true);
    tft.setTextColor(TFT_GREEN);
    tft.setTextDatum(MC_DATUM);
    tft.fillScreen(TFT_RED);
    espDelay(1000);
    tft.fillScreen(TFT_BLUE);
    espDelay(1000);
    tft.fillScreen(TFT_GREEN);
    espDelay(1000);

    button_init();

    /*
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);



    tft.drawString("LeftButton:", tft.width() / 2, tft.height() / 2 - 16);      // hi
    tft.drawString("[WiFi Scan]", tft.width() / 2, tft.height() / 2 );
    tft.drawString("RightButton:", tft.width() / 2, tft.height() / 2 + 16);
    tft.drawString("[Voltage Monitor]", tft.width() / 2, tft.height() / 2 + 32 );
    tft.drawString("RightButtonLongPress:", tft.width() / 2, tft.height() / 2 + 48);
    tft.drawString("[Deep Sleep]", tft.width() / 2, tft.height() / 2 + 64 );
    tft.setTextDatum(TL_DATUM);
    */
}

/**** METHOD: LCDSPLASH ************************/
void lcdsplash()
{
    tft.fillScreen(TFT_BLACK);
    //tft.drawString("FB CallMonitor", tft.width() / 2, tft.height() / 2);
    //tft.drawString("www.ranger81.de:", tft.width() / 2, tft.height() / 2 + 16);
}
/***********************************************/
/**** METHOD: RESETETHERNET ********************/
void resetEthernet()
{
    /*
    client.stop();
    delay(1000);
    Ethernet.begin(mac);
    delay(1000);
    */
}
/***********************************************/
/**** METHOD: LCDON ****************************/
void lcdon()
{
    tft.fillScreen(TFT_WHITE);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(2);
    tft.setTextColor(TFT_BLACK);
}
/***********************************************/

/**** METHOD: LCDDIM ***************************/
void lcddim()
{
    if (LCD_ENABLE_DIM)
    {
        if (missedcallcount > 0)
        {
            lcdmissedcall();
        }
        else
        {
            lcdsplash();
        }
    }
    else
    {
        lcdoff();
    }
}
/***********************************************/

/**** METHOD: LCDOFF ***************************/
void lcdoff()
{
    if (missedcallcount > 0)
    {
        lcdmissedcall();
    }
    else
    {
        lcdsplash();
    }
    //digitalWrite(LCD_BACKLIGHT_PIN, LOW);
}
/***********************************************/

/**** METHOD: LCDSTARTDIM **********************/
void lcdstartdim()
{
    lcd_dimmer = true;
    lcdtimeoutstart = millis();
}
/***********************************************/

/**** METHOD: LCDCONNECTING ********************/
void lcdconnecting()
{
    lcdon();
    tft.drawString("Connecting to", tft.width() / 2, tft.height() / 2);
    tft.drawString("FritzBox... ", tft.width() / 2, tft.height() / 2 + 16);
}
/***********************************************/

/**** METHOD: MISSEDCALLLEDON ******************/
void missedcallledon(char *lastnr)
{
    missedcallcount++;
    //digitalWrite(MISSED_CALL_LED_PIN, HIGH);
    // strcpy(lastmissednumber, lastnr);
    memcpy(lastmissednumber, lastnr, LCD_MAX_CHARS + 1);
    lcdmissedcall();
    lcdstartdim();
}
/***********************************************/

/**** METHOD: LCDMISSEDCALL ********************/
void lcdmissedcall()
{
    String callCountMessage = missedcallcount + String(" missed calls");
    lcdon();
    tft.drawString(callCountMessage.c_str(), tft.width() / 2, tft.height() / 2);
    tft.drawString(lastmissednumber, tft.width() / 2, tft.height() / 2 + 16);
    lcdstartdim();
}
/***********************************************/

/**** METHOD: MISSEDCALLLEDOFF *****************/
void missedcallledoff()
{
    if (missedcallcount > 0)
    {
        //digitalWrite(MISSED_CALL_LED_PIN, LOW);
        missedcallcount = 0;
        lcddim();
    }
}
/***********************************************/

/**** METHOD: LCDDISPLAYTIME *******************/
void lcddisplaytime(unsigned long t)
{
#ifdef ENABLE_DISPLAY_TIME
    word h = (t / 3600) % 24;
    byte m = (t / 60) % 60;
    byte s = t % 60;
    lcd.setCursor(0, 1);

    if (!showprice)
    {
        lcd.print("    ");
    }

    if (h < 10)
    {
        lcd.print("0");
    }
    lcd.print(h);
    lcd.print(":");
    if (m < 10)
    {
        lcd.print("0");
    }
    lcd.print(m);
    lcd.print(":");
    if (s < 10)
    {
        lcd.print("0");
    }
    lcd.print(s);

    // Price calculation
    if (showprice && t > 0)
    {

        float currentprice = (1 + m + (h * 60)) * PRICEPERMINUTE;

        lcd.print(" ");
        lcd.print(currentprice);

        /*
  float battvcc = (float) readVcc() / 1000.0;
     char battbuffer[5] = "\0";
     dtostrf(battvcc, 4,2, battbuffer);
     */
    }
    else
    {
        lcd.print("    ");
    }
#endif
}
/***********************************************/

//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
void espDelay(int ms)
{
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_light_sleep_start();
}

void button_init()
{
    btn1.setPressedHandler([](Button2 &b) {
        Serial.println("Detect Voltage..");
        btnCick = true;
    });

    btn2.setPressedHandler([](Button2 &b) {
        btnCick = false;
        Serial.println("btn press wifi scan");
        //wifi_scan();
    });
}

void button_loop()
{
    btn1.loop();
    btn2.loop();
}

void WiFi_Setup()
{
    WiFiManager wifiManager; // Connect to Wi-Fi
    // A new OOB ESP has no Wi-Fi credentials so will connect and not need the next command to be uncommented and compiled in, a used one with incorrect credentials will
    // so restart the ESP and connect your PC to the wireless access point called 'ESP_AP' or whatever you call it below in ""
    // wifiManager.resetSettings(); // Command to be included if needed, then connect to http://192.168.4.1/ and follow instructions to make the WiFi connection
    // Set a timeout until configuration is turned off, useful to retry or go to sleep in n-seconds
    wifiManager.setHostname("FBCallMonitor");
    wifiManager.setTimeout(180);
    //fetches ssid and password and tries to connect, if connections succeeds it starts an access point with the name called "ESP8266_AP" and waits in a blocking loop for configuration
    if (!wifiManager.autoConnect("FBCallMonitor_AP"))
    {
        Serial.println(F("failed to connect and timeout occurred"));
        delay(6000);
        ESP.restart(); //reset and try again
    }
    // At this stage the WiFi manager will have successfully connected to a network, or if not will try again in 180-seconds
    Serial.println(F("WiFi connected..."));
    //----------------------------------------------------------------------
    Serial.println(F("Use this URL to connect: http://"));
    Serial.println(WiFi.localIP().toString()); // Print the IP address
}

void loop()
{

    button_loop();

    if ((millis() - next) > RETRY_TIMEOUT)
    {
        next = millis();
#ifdef DEBUG
        Serial.println(F("Trying to connect..."));
#endif

        lcdconnecting();

        // replace hostname with name of machine running tcpserver.pl
        if (client.connect(FRITZBOX_HOSTNAME, FRITZBOX_PORT))
        //if (client.connect(IPAddress(192,168,171,1),1012))
        {

            if (client.connected())
            {
#ifdef DEBUG
                Serial.println(F("Connected successfully"));
#endif
                connectioncheck = millis();
                lcddim();
            }

            /*client.println("DATA from Client");
       while(client.available()==0)
       {
       if (next - millis() < 0)
       goto close;
       }*/
            while (client.connected())
            {
                // if in a call
                if (call_connected && ((millis() - calllaststatus) > CALL_DURATION_UPDATE_INTERVAL))
                {
                    unsigned long seconds = (millis() - callstart) / 1000;
                    lcddisplaytime(seconds);
                }

                // Dim LCD screen
                if (lcd_dimmer && ((millis() - lcdtimeoutstart) > LCD_DIM_TIMEOUT))
                {
                    lcddim();
                    lcd_dimmer = false;
                }

                if (!digitalRead(CLEAR_MISSED_CALL_BUTTON_PIN))
                {
                    missedcallledoff();
                }

                int size;

                while ((size = client.available()) > 0)
                {
                    uint8_t *msg = (uint8_t *)malloc(size);
                    size = client.read(msg, size);
                    msg[size - 1] = '\0';

#ifdef DEBUG
                    Serial.print(F("->Msg: "));
                    Serial.println((char *)msg);
#endif

                    // Copy of msg needed for strtok/strtok_r because of modification of original value (see https://www.securecoding.cert.org/confluence/display/seccode/STR06-C.+Do+not+assume+that+strtok%28%29+leaves+the+parse+string+unchanged)
                    uint8_t *copymsgforsplit = (uint8_t *)malloc(size);
                    memcpy(copymsgforsplit, msg, size);

                    // Analyze incoming msg
                    int i = 0;
                    char *pch, *ptr;
                    char type[11];
                    pch = strtok_r((char *)copymsgforsplit, ";", &ptr);

                    while (pch != NULL)
                    {

#ifdef DEBUG
                        Serial.print(F("    ->Splitted part "));
                        Serial.print(i);
                        Serial.print(F(": "));
                        Serial.println(pch);
#endif

                        switch (i)
                        {
                        case 0: // Date and Time
                            if (CLEAR_MISESED_CALL_ON_NEW_ACTION)
                            {
                                missedcallledoff();
                            }
                            lastcallwasmissedcall = false;
                            lcdon();
                            {
                                char vbuff[6];
                                memset(vbuff, 0, sizeof(vbuff));
                                memcpy(vbuff, &pch[9], 5);
                                tft.drawString(vbuff, tft.width() / 2, tft.height() / 2);
                            }
                            break;
                        case 1: // TYPE
                            if (((strcmp(type, "RING") == 0) && (strcmp(pch, "DISCONNECT") == 0)) || (strstr((char *)msg, ";40;") && strcmp(type, "RING") == 0 && strcmp(pch, "CONNECT") == 0))
                            {
                                missedcallledon(lastnumber);
                                lastcallwasmissedcall = true;
                            }

                            strcpy(type, pch);

                            if (strcmp(type, "CALL") == 0)
                            {
                                showprice = true;
                            }

                            if (!lastcallwasmissedcall)
                            {
                                tft.drawString(type, tft.width() / 2, tft.height() / 2 - 16);
                            }
                            break;
                        case 2: // ConnectionID
                            // Currently not needed...
                            break;
                        case 3:
                            if (strcmp(type, "RING") == 0) // Who is calling
                            {
                                if (strstr((char *)msg, ";;")) // Unknown caller?
                                {
                                    tft.drawString("Unknown caller", tft.width() / 2, tft.height() / 2);
                                    strcpy(lastnumber, "Unknown caller\0");
                                }
                                else
                                {
                                    tft.drawString(pch, tft.width(), tft.height() / 2);
                                    memcpy(lastnumber, pch, LCD_MAX_CHARS + 1); // Geht das auch? Etwas sicherer, falls pch länger ist als lastnumber // ja, geht
                                    lastnumber[LCD_MAX_CHARS + 1] = '\0';
                                    // strcpy(lastnumber, pch);
                                    //  strcat(lastnumber, "\0");
                                }
                            }
                            else if (strcmp(type, "DISCONNECT") == 0) // How long was the call
                            {
                                //lcd.print(pch);
                                //lcd.print(" seconds");
                                call_connected = false;
                                if (!lastcallwasmissedcall)
                                {
                                    lcddisplaytime(strtoul(pch, NULL, 0));
                                    showprice = false;
                                    lcdstartdim();
                                }
                            }

                            break;
                        case 4:
                            if (strcmp(type, "CONNECT") == 0) // Connected with number
                            {
                                if (!lastcallwasmissedcall)
                                {
                                    tft.drawString(pch, tft.width() / 2, tft.height() / 2);
                                }
                                if (DISPLAY_CALL_DURATION)
                                {
                                    call_connected = true;
                                    callstart = calllaststatus = millis();
                                }
                            }
                            break;
                        case 5:
                            if (strcmp(type, "CALL") == 0) // Calling number
                            {
                                tft.drawString(pch, tft.width() / 2, tft.height() / 2);
                            }
                            break;
                        default:
                            break;
                        }
                        i++;
                        pch = strtok_r(NULL, ";", &ptr); // Split next part
                    }

                    free(msg);
                    free(copymsgforsplit);
                }

                // Check connection
                if ((millis() - connectioncheck) > CHECKCONNECTION)
                {
#ifdef DEBUG
                    Serial.println(F("Checking connection..."));
#endif
                    connectioncheck = millis();

                    // Send dummy data to "refresh" connection state
                    client.write("x");
                }
            }
            //close:
            //disconnect client
#ifdef DEBUG
            Serial.println(F("Disconnected"));
#endif
            //client.stop();
            lcdconnecting();
            resetEthernet();
        }
        else
        {
#ifdef DEBUG
            Serial.println(F("Connection failed, retrying..."));
#endif

            // Hier auch ein resetEthernet() nötig???
        }
    }
}
