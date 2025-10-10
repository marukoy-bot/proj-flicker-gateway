#include <Arduino.h>
#include "PMS.h"
#include <SoftwareSerial.h>
#include "DFRobot_BME280.h"
#include "Wire.h"
#include <SPI.h>
#include <LoRa.h>

#define DEBUG 1
#include "debug.h"

// definitions
#define rx 15
#define tx 2
#define co_sensor 34
#define co_samples 20
#define SEA_LEVEL_PRESSURE 1015.0f

#define MOSI 23
#define MISO 19
#define SCK 18
#define NSS 25
#define RST 32
#define DIO0 33

#define TEMP_THRESHOLD 35.0f      // example
#define HUM_THRESHOLD 80.0f       // example
#define CO_THRESHOLD 200.0f       // ppm
#define SMOKE_THRESHOLD 300       // arbitrary ADC value or mapped ppm

#define node_id "0x55"

enum STATE {
    IDLE,
    ENV_ALERT,
    CO_ALERT,
    SMOKE_ALERT,
    CRITICAL,
    AFTER_ALERT,
    SHUT_DOWN
} current_state;

// global variables
static const uint32_t PMS_READ_INTERVAL = 270000;
static const uint32_t PMS_READ_DELAY = 30000;
uint32_t timerInterval = 0;
uint32_t lastSampleTime = 0;

uint16_t pms_1 = 0;
uint16_t pms_2_5 = 0;
uint16_t pms_10 = 0;

// instantiations
SoftwareSerial pms_serial(rx, tx);
PMS pms(pms_serial);
PMS::DATA data;
TwoWire bme_wire = TwoWire(0);  //  SDA = 21; SCL = 22
typedef DFRobot_BME280_IIC BME;
BME bme(&bme_wire, 0x77);

// functions
void getPMSData();
float readVoltageV();
float voltageToPpm(float v);
void getBMEOpStatus(BME::eStatus_t status);
void sendLoRaMessage(String message);
void showStats();

void setup() {  
#if DEBUG == 1
    Serial.begin(115200);
#endif
    pms_serial.begin(9600);
    //pms.passiveMode();
    db_println("PMS ready.");

    analogSetPinAttenuation(co_sensor, ADC_11db);

    bme_wire.begin(21, 22);
    bme.reset();
    while (bme.begin() != BME::eStatusOK) {
        db_println("BME initialization failed");
        getBMEOpStatus(bme.lastOperateStatus);
        delay(2000);
    }

    db_println("BME ready.");

    SPI.begin(SCK, MISO, MOSI, NSS);
    LoRa.setPins(NSS, RST, DIO0);
    if (!LoRa.begin(433E6)) {
        db_println("LoRa initialization failed.");
        while(1);
    }
    db_println("LoRa ready.");
    delay(2000);
    pinMode(2, OUTPUT); 
    current_state = IDLE;
}

unsigned long fade_interval = 1000;
unsigned long prev_millis = 0, prev_stats_millis = 0, pms_data_valid_time = 0;
int brightness = 0;
bool fading_up = true, pms_valid = false;

void loop() {
    if (millis() - lastSampleTime >= timerInterval) {
        lastSampleTime = millis();
        
        switch (current_state) {
            case IDLE: {
                float temp = bme.getTemperature();
                float hum = bme.getHumidity();

                db_print("Current Interval: " + String(timerInterval) + " | State: IDLE | ");
                db_println(String(temp) + "° C | " + String(hum) + "% RH");

                if (temp > TEMP_THRESHOLD || hum > HUM_THRESHOLD) {
                    current_state = ENV_ALERT;
                }
                break;
            }

            case ENV_ALERT: {
                float temp = bme.getTemperature();
                float hum = bme.getHumidity();
                float co_ppm = voltageToPpm(readVoltageV());

                db_print("Current Interval: " + String(timerInterval) + " | State: ENV_ALERT | ");
                db_println(String(temp) + "° C | " + String(hum) + "% RH");

                if (co_ppm > CO_THRESHOLD) {
                    current_state = CO_ALERT;
                }
                break;
            }

            case CO_ALERT: {
                float co_ppm = voltageToPpm(readVoltageV());
                db_print("Current Interval: " + String(timerInterval) + " | State: ENV_ALERT | ");
                db_println(String(co_ppm) + " ppm | ppms 2.5: " + String(pms_2_5));
                //getPMSData();
                if (pms_2_5 > SMOKE_THRESHOLD) {
                    current_state = SMOKE_ALERT;
                }
                break;
            }

            case SMOKE_ALERT: {
                // all conditions met → critical
                current_state = CRITICAL;
                break;
            }

            case CRITICAL: {
                String msg = 
                    "<" + 
                    String(node_id) + 
                    ">T:" + String(bme.getTemperature()) +
                    ",H:" + String(bme.getHumidity()) +
                    ",CO:" + String(voltageToPpm(readVoltageV())) +
                    ",PM2.5:" + String(pms_2_5);
                sendLoRaMessage(msg);
                current_state = AFTER_ALERT;
                break;
            }

            case AFTER_ALERT: {
                delay(120000);   // cooldown 2 min
                esp_sleep_enable_timer_wakeup(30LL * 60 * 1000000); // 30 mins
                esp_deep_sleep_start();
                break;
            }

            case SHUT_DOWN:
                // optional deep sleep mode
                break;
        }
    }

    // adjust timer per state
    switch (current_state) {
        case IDLE: timerInterval = 600000; break;       // 10 min
        case ENV_ALERT: 
        case CO_ALERT:
        case SMOKE_ALERT:
        case CRITICAL:
        case AFTER_ALERT: timerInterval = 120000; break; // 2 min
        case SHUT_DOWN: timerInterval = 1800000; break;  // 30 min
        default: timerInterval = 600000; break;
    }

    if (current_state == STATE::CO_ALERT) getPMSData();

    unsigned long elapsed = millis() - prev_millis;

    if (fading_up) {
        brightness = map(elapsed, 0, fade_interval, 0, 255);

        if (elapsed >= fade_interval) {
            brightness = 255;
            fading_up = false;
            prev_millis = millis();
        }
    }
    else {
        brightness = map(elapsed, 0, fade_interval, 255, 0);

        if (elapsed >= fade_interval) {
            brightness = 0;
            fading_up = true;
            prev_millis = millis();
        }
    }

    analogWrite(2, brightness);

    /*
    getPMSData();
    if (pms_1 != 0.0 && pms_2_5 != 0.0 && pms_10 != 0.0 && !pms_valid) {
        pms_data_valid_time = millis();
        pms_valid = true;
    }

    if (millis() - prev_stats_millis >= 10000) {
        prev_stats_millis = millis();
        showStats();
    }
    */
}

void getPMSData() {
    if (pms.read(data)) {
        pms_1 = data.PM_AE_UG_1_0;
        pms_2_5 = data.PM_AE_UG_2_5;
        pms_10 = data.PM_AE_UG_10_0;
    }
}

float readVoltageV() {
    long sum = 0;
    for (int i = 0; i < co_samples; ++i) {
        sum += analogRead(co_sensor);
        delay(5);
    }
    float raw = sum / (float)co_samples;          // 0..4095
    float voltage = raw * (3.3f / 4095.0f);    // in volts
    return voltage;
}

float voltageToPpm(float v) {
    // correct linear mapping (expects volts)
    float ppm = (v - 0.4f) * (4995.0f / 1.6f) + 5.0f;
    if (ppm < 5.0f) ppm = 5.0f;
    if (ppm > 5000.0f) ppm = 5000.0f;
    return ppm;
}

void getBMEOpStatus(BME::eStatus_t eStatus) {
    switch (eStatus) {
        case BME::eStatusOK:                    Serial.println("Status: OK"); break;
        case BME::eStatusErr:                   Serial.println("Status: Unknown Error"); break;
        case BME::eStatusErrDeviceNotDetected:  Serial.println("Status: Unknown Device or Deveice not detected"); break;
        case BME::eStatusErrParameter:          Serial.println("Status: Error Parameter"); break;
        default: Serial.println("Status: Unknown"); 
    }
}

void sendLoRaMessage(String message) {
    LoRa.beginPacket();
    LoRa.print(message);
    LoRa.endPacket();
}

void showStats() {
    db_println("PMS data valid time: " + String(pms_data_valid_time / 1000.0) + " s");
    db_println("PM 1.0:\t\t" + String(pms_1) + " ug/m3");
    db_println("PM 2.5:\t\t" + String(pms_2_5) + " ug/m3");
    db_println("PM 10.0:\t" + String(pms_10) + " ug/m3");
    //db_print("raw:\t");
    float co_voltage = readVoltageV();
    db_println("voltage (v):\t" + String(co_voltage) + "v");
    db_println("ppm :\t\t" + String(voltageToPpm(co_voltage)));
    db_println("temperature :\t" + String(bme.getTemperature()) + " °C");
    db_println("humidity :\t" + String(bme.getHumidity()) + " %");
    uint32_t bme_pressure = bme.getPressure();
    db_println("pressure :\t" + String(bme_pressure) + " pa");
    db_println("altitude :\t" + String(bme.calAltitude(SEA_LEVEL_PRESSURE, bme_pressure)) + " m");
    db_println();
}