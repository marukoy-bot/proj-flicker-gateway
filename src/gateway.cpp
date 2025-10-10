#include <Arduino.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <LoRa.h>

#define NSS 4
#define RST 17
#define DIO0 16
#define LED 2
#define gsm_rx 13
#define gsm_tx 14

#define DEBUG 1
#include "debug.h"

LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial gsm(gsm_rx, gsm_tx);
String flicker_id = "flicker0";

volatile bool packetReceived = false;

void printLCD(String message, int row);
void initializeGSM();
void sendSMS(String num, String message);
void updateGSM();
void onReceive(int packetSize);

void setup() {
#if DEBUG == 1
    Serial.begin(115200);
#endif
    lcd.init();
    lcd.backlight();
    lcd.clear();
    printLCD("Initializing...", 0);

    LoRa.setPins(NSS, RST, DIO0);
    pinMode(LED, OUTPUT); 
    pinMode(DIO0, INPUT);
    
    if (!LoRa.begin(433E6)) {
        db_println("LoRa initialization failed.");
        while(1);
    }
    
    // Register the receive callback
    LoRa.onReceive(onReceive);
    
    // Put LoRa in receive mode
    LoRa.receive();
    
    db_println("LoRa RX ready with interrupt.");

    initializeGSM();
    db_println("GSM ready");

    lcd.clear();
    printLCD("Gateway Ready", 0);
    delay(1000);
    
    lcd.noBacklight();
    lcd.clear();
}

void loop() {
    if (packetReceived) {
        packetReceived = false;
        
        // Wake up display
        lcd.backlight();
        lcd.clear();
        
        String message = "";
        while (LoRa.available()) {
            message += (char)LoRa.read();
        }
        message.trim();
        db_println("Received: " + message);

        // Parse message
        int open_bracket_index = message.indexOf('<');
        int close_bracket_index = message.indexOf('>');
        int colon_index = message.indexOf(':');

        if (open_bracket_index != -1 && close_bracket_index != -1 && colon_index != -1) {
            String lora_id = message.substring(open_bracket_index + 1, close_bracket_index);
            lora_id.trim();
            String msg_part = message.substring(close_bracket_index + 1, colon_index);
            msg_part.trim();
            String num_part = message.substring(colon_index + 1);
            num_part.trim();
 
            printLCD(lora_id + ":", 0);
            printLCD(msg_part + ":" + num_part, 1);
            
            // Keep display on for 3 seconds
            delay(3000);
        }
        
        // Turn off display and go back to listening
        lcd.noBacklight();
        lcd.clear();
        
        // Resume receive mode
        LoRa.receive();
    }
}
    

void onReceive(int packetSize) {
    if (packetSize > 0) {
        packetReceived = true;
    }
}

void printLCD(String message, int row) {
    lcd.setCursor(0, row);
    lcd.print(message);
}

void initializeGSM () {
    db_println("Checking module...");
    gsm.begin(9600);
    gsm.println("AT");
    updateGSM();
    db_println("Updating time...");
    gsm.println("AT+CLTS=1");
    updateGSM();
    gsm.println("AT&W");
    updateGSM();
}

void sendSMS (String num, String message) {
    gsm.println("AT+CMGF=1");
    updateGSM();
    delay(250);

    gsm.println("AT+CMGS=\"" + num + "\"");
    updateGSM();
    delay(1000);    

    gsm.print(message);
    gsm.print(char(26));
    updateGSM();
    delay(1000);
}

void updateGSM() {
    delay(500);
    while (gsm.available()) db_write((char)gsm.read());
}