#include <Arduino.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <LoRa.h>
#include <Preferences.h>

#define NSS 4
#define RST 17
#define DIO0 16
#define LED 2
#define gsm_rx 13
#define gsm_tx 14
#define MAX_NUMBERS 10 

LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial gsm(gsm_rx, gsm_tx);
Preferences preferences;
static const char* PREFERENCES_NAMESPACE = "nums"; 

String subscribers[MAX_NUMBERS];
int subscriberCount = 0;
String flicker_id = "flicker0";
volatile bool packetReceived = false;

unsigned long received_ToA = 0; /// Time on Air (from transmitter)
unsigned long reception_timestamp = 0;
int rssi = 0;   /// Signal strength
float snr = 0;  /// Signal-to-noise ratio

void printLCD(String message, int row);
void initializeGSM();
void sendSMS(String num, String message);
void updateGSM();
void onReceive(int packetSize);
void loadSubscribers();
bool isSubscribed(String num);
void addSubscriber(String num);
void deleteSubscriber(String num);
void getSubscriberList(String sender);
void checkSMS();
void clearNamespace();
void displayLCD(String row0Msg, String row1Msg, long duration);

void setup() {
    Serial.begin(115200);
    lcd.init();
    lcd.backlight();
    lcd.clear();
    printLCD("Initializing...", 0);
    
    Serial.println(preferences.begin(PREFERENCES_NAMESPACE, false) ? "prefs loaded successfully" : "prefs loaded failed");
    //clearNamespace();

    LoRa.setPins(NSS, RST, DIO0);
    pinMode(LED, OUTPUT); 
    pinMode(DIO0, INPUT);
    
    if (!LoRa.begin(433E6)) {
        Serial.println("LoRa initialization failed.");
        while(1);
    }
    
    // Register the receive callback
    LoRa.onReceive(onReceive);
    
    // Put LoRa in receive mode
    LoRa.receive();
    
    Serial.println("LoRa RX ready");

    initializeGSM();
    Serial.println("GSM ready");

    loadSubscribers();

    lcd.clear();
    printLCD("Gateway Ready", 0);
    delay(5000);
    
    lcd.noBacklight();
    lcd.clear();
}

void loop() {
    if (packetReceived) {
        packetReceived = false;

        reception_timestamp = millis();
        rssi = LoRa.packetRssi();
        snr = LoRa.packetSnr();
        
        displayLCD("DATA RECEIVED", "", 3000);
        
        String message = "";
        while (LoRa.available()) {
            message += (char)LoRa.read();
        }
        message.trim();
        Serial.println("Received: " + message);

        // Parse message
        int open_bracket_index = message.indexOf('<');
        int close_bracket_index = message.indexOf('>');

        if (open_bracket_index != -1 && close_bracket_index != -1) {
            String flicker_id = message.substring(open_bracket_index + 1, close_bracket_index);
            flicker_id.trim();

            String data = message.substring(close_bracket_index + 1);
            data.trim();

            if (flicker_id == "flicker00") {
                float temperature = 0, humidity = 0, co = 0, pm2_5 = 0;
                received_ToA = 0;
    
                int t_index = data.indexOf("T:");
                int h_index = data.indexOf("H:");
                int co_index = data.indexOf("CO:");
                int pm_index = data.indexOf("PM2.5:");
                int toa_index = data.indexOf("ToA:");
    
                if (t_index != -1) {
                    int comma = data.indexOf(',', t_index);
                    temperature = data.substring(t_index + 2, comma).toFloat();
                }
    
                if (h_index != -1) {
                    int comma = data.indexOf(',', h_index);
                    humidity = data.substring(h_index + 2, comma).toFloat();
                }
    
                if (co_index != -1) {
                    int comma = data.indexOf(',', co_index);
                    co = data.substring(co_index + 3, comma).toFloat();
                }
    
                if (pm_index != -1) {
                    int comma = data.indexOf(',', pm_index);
                    pm2_5 = data.substring(pm_index + 6, comma).toFloat();
                }

                if (toa_index != -1) {
                    String toa_str = data.substring(toa_index + 4);
                    toa_str.trim();
                    received_ToA = toa_str.toInt();
                }
     
                // Print to Serial and LCD

                Serial.println("========= Packet details =========");
                Serial.println("Flicker ID: " + flicker_id);
                Serial.println("Temp: " + String(temperature) + "C");
                Serial.println("Hmd: " + String(humidity) + "%");
                Serial.println("CO: " + String(co) + "ppm");
                Serial.println("PM2.5: " + String(pm2_5) + "ug/m3");
                Serial.println("ToA (TX): " + String(received_ToA));
                Serial.println("RSSI: " + String(rssi) + " dBm");
                Serial.println("SNR: " + String(snr) + " dB");
                Serial.println("Timestamp: " + String(reception_timestamp) + " ms");
                Serial.println("==================================");
                
                //displayLCD("T:" + String(temperature) + " H:" + String(humidity), "CO:" + String(co) + " P:" + String(pm2_5), 5000);
    
                String msg = 
                    "[ALERT]\nTemp: " + String(temperature) + "C\n"
                    + "Hmd: " + String(humidity) + "%\n"
                    + "CO: " + String(co) + " ppm\n"
                    + "PM 2.5: " + String(pm2_5) + " ug/m3";

                //sendSMS("+639151635499", msg);

                for (int i = 0; i < subscriberCount; i++) {
                    if (subscribers[i].length() > 0) {
                        sendSMS(subscribers[i], msg);
                    }
                }
                
                displayLCD("SMS SENT", "", 5000);
            }            
        }       
        
        // Resume receive mode
        LoRa.receive();
    }
    checkSMS();
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
    Serial.println("Checking module...");
    gsm.begin(9600);

    gsm.println("AT");
    updateGSM();

    gsm.println("AT+CSCS=\"GSM\"");
    updateGSM();

    gsm.println("AT+CNMI=1,2,0,0,0");
    updateGSM();

    gsm.println("AT+CMGF=1");
    updateGSM();

    Serial.println("Updating time...");
    gsm.println("AT+CLTS=1");
    updateGSM();

    gsm.println("AT&W");
    updateGSM();
}

/// @brief handles GSM message
/// @param num mobile number; should start with "+63..."
/// @param message message to be sent to gsm
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
    delay(3500);
}

void updateGSM() {
    delay(500);
    while (gsm.available()) Serial.write((char)gsm.read());
}

void loadSubscribers() {
    preferences.begin(PREFERENCES_NAMESPACE, false);
    subscriberCount = preferences.getInt("count", 0);
    for (int i = 0; i < subscriberCount; i++) {
        String key = "num" + String(i);
        subscribers[i] = preferences.getString(key.c_str(), "");
        Serial.println(subscribers[i]);
    }
    preferences.end();  // Close after all operations
    Serial.println("Loaded " + String(subscriberCount) + " subscribers.");
}

void clearNamespace() {
    preferences.clear();
}

bool isSubscribed(String num) {
    for (int i = 0; i < subscriberCount; i++) {
        if (subscribers[i] == num) return true;
    }
    return false;
}

/// @brief add suscribers for sms feature
/// @param num mobile number of subscriber
void addSubscriber(String num) {
    if (isSubscribed(num)) {
        Serial.println("Already subscribed");
        return;
    }
    if (subscriberCount >= MAX_NUMBERS) {
        Serial.println("Reached max number of subscribers");
        return;
    }
    
    preferences.begin(PREFERENCES_NAMESPACE, false);
    
    String key = "num" + String(subscriberCount);
    preferences.putString(key.c_str(), num);
    subscribers[subscriberCount] = num;  // Update array too!
    subscriberCount++;
    preferences.putInt("count", subscriberCount);
    
    preferences.end();
    Serial.println("Added subscriber: " + num);
    displayLCD("Added", num, 5000);
}

/// @brief display messages to LCD for a certain `duration`
/// @param row0Msg message displayed in row 0
/// @param row1Msg message displayed in row 1
/// @param duration time in which the display will persist before LCD clearing
void displayLCD(String row0Msg, String row1Msg, long duration) {
    lcd.clear();
    lcd.backlight();
    printLCD(row0Msg, 0);
    printLCD(row1Msg, 1);
    delay(duration);
    lcd.clear();
    lcd.noBacklight();
}

void deleteSubscriber(String num) {
    bool found = false;

    for (int i = 0; i < subscriberCount; i++) {
        if (subscribers[i] == num) {
            for (int j = i; j < subscriberCount - 1; j++) {
                subscribers[j] = subscribers[j + 1];
            }
            subscriberCount--;
            found = true;
            break;
        }
    }

    if (found) {
        preferences.begin(PREFERENCES_NAMESPACE, false);
        preferences.putInt("count", subscriberCount);

        for (int i = 0; i < subscriberCount; i++) {
            String key = "num" + String(i);
            preferences.putString(key.c_str(), subscribers[i]);
        }
        
        // Remove the last orphaned key
        String lastKey = "num" + String(subscriberCount);
        preferences.remove(lastKey.c_str());
        
        preferences.end();
        
        Serial.println("Deleted subscriber: " + num);
        displayLCD("Deleted", num, 5000);
    }
}

void checkSMS() {    
    if (gsm.available()) {
        delay(300);
        String response = "";
        
        while (gsm.available()) {
            response += (char)gsm.read();
        }
        response.toUpperCase();
        Serial.println(response);

        int numStartIndex = response.indexOf('"');
        int numEndIndex = response.indexOf('"', numStartIndex + 1);
        String sender = "", message = "";

        if (numStartIndex != -1 && numEndIndex != -1) {
            sender = response.substring(numStartIndex + 1, numEndIndex);
        }

        int msgStart = response.indexOf('\n', numEndIndex);
        if (msgStart != -1) {
            message = response.substring(msgStart + 1);
            message.trim();
        }

        Serial.println("Sender: " + sender);
        Serial.println("message: " + message);

        if (message.indexOf("GSM ADD") > -1) {
            addSubscriber(sender);
        }
        else if (message.indexOf("GSM DELETE") > -1) {
            deleteSubscriber(sender);
        }
        else if (message.indexOf("GSM LIST") > -1) {
            getSubscriberList(sender);
        }
    }    
}

void getSubscriberList(String sender) {
    String message = "";

    for (int i = 0; i < subscriberCount; i++) {
        message += String(i + 1) + ". " + subscribers[i] + "\n";
    }
    sendSMS(sender, message);
    displayLCD("Sending list", "", 5000);
}

