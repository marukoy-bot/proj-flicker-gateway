#include <Arduino.h>
#include "PMS.h"
#include <SoftwareSerial.h>
#include "DFRobot_BME280.h"
#include "Wire.h"
#include <SPI.h>
#include <LoRa.h>
#include <Preferences.h>

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

#define TEMP_THRESHOLD 35.0f      /// estimate
#define HUM_THRESHOLD 30.0f       /// estimate
#define CO_THRESHOLD 10.0f       /// CO ppm
#define SMOKE_THRESHOLD 300       /// pms 2.5

#define flicker_id "flicker00"

// flags
bool flag0_idle = true;
bool flag1_env_alert = false;
bool flag2_co_alert = false;
bool flag3_smoke_alert = false;

// status LEDS
#define led_red 14
#define led_yellow 27
#define led_green 26

// global timer variables
uint32_t sampleTimerInterval = 0; ///
uint32_t lastSampleTime = 0;

// PMS5003
uint16_t pms_1 = 0;
uint16_t pms_2_5 = 0;
uint16_t pms_10 = 0;

// CO sensor
float co_voltage = 0;
float co_ppm = 0;

// BME
float bme_temperature = 0;
float bme_humidity = 0;
float bme_pressure = 0;
float bme_altitude = 0;

// Time on Air (estimated transmission time)
unsigned long last_ToA = 0;

// instantiations
SoftwareSerial pms_serial(rx, tx);
PMS pms(pms_serial);
PMS::DATA data;
TwoWire bme_wire = TwoWire(0);  //  SDA = 21; SCL = 22
typedef DFRobot_BME280_IIC BME;
BME bme(&bme_wire, 0x77);

Preferences preferences;
const char* pref_namespace = "ToA";

// functions
void getPMSData();
float readVoltageV();
float voltageToPpm(float v);
void getBMEOpStatus(BME::eStatus_t status);
void sendLoRaMessage(String message);
void showStats();
void updateSensors();
void testTransmit();
void updateLEDs() {
    digitalWrite(led_green, (flag0_idle || flag1_env_alert));
    digitalWrite(led_yellow, flag2_co_alert);
    digitalWrite(led_red, flag3_smoke_alert);
}

void setup() {  
    Serial.begin(115200);
    pms_serial.begin(9600);
    Serial.println("PMS ready.");

    analogSetPinAttenuation(co_sensor, ADC_11db);
    analogReadResolution(12);
    analogSetWidth(12);
    Serial.println("CO sensor ready.");

    preferences.begin(pref_namespace, false);
    last_ToA = preferences.getULong("last_ToA", 0);
    preferences.end();

    bme_wire.begin(21, 22);
    bme.reset();
    while (bme.begin() != BME::eStatusOK) {
        Serial.println("BME initialization failed");
        getBMEOpStatus(bme.lastOperateStatus);
        delay(2000);
    }

    Serial.println("BME ready.");

    SPI.begin(SCK, MISO, MOSI, NSS);
    LoRa.setPins(NSS, RST, DIO0);
    if (!LoRa.begin(433E6)) {
        Serial.println("LoRa initialization failed.");
        while(1);
    }

    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5); 
    LoRa.setTxPower(20);
    Serial.println("LoRa ready.");

    delay(2000);
    pinMode(2, OUTPUT); // LED_BUILTIN

    sampleTimerInterval = 120000; // 2 mins default

    pinMode(led_red, OUTPUT);
    pinMode(led_yellow, OUTPUT);
    pinMode(led_green, OUTPUT);

    // blink LEDs
    digitalWrite(led_red, HIGH);
    digitalWrite(led_yellow, HIGH);
    digitalWrite(led_green, HIGH);

    delay(500);

    digitalWrite(led_red, LOW);
    digitalWrite(led_yellow, LOW);
    digitalWrite(led_green, LOW);

    delay(500);

    digitalWrite(led_red, HIGH);
    digitalWrite(led_yellow, HIGH);
    digitalWrite(led_green, HIGH);

    delay(500);

    digitalWrite(led_red, LOW);
    digitalWrite(led_yellow, LOW);
    digitalWrite(led_green, LOW);

    delay(500);

    //testTransmit();
}

unsigned long last_sensor_sample_time = 0;
unsigned long fade_interval = 1000;
unsigned long prev_millis = 0, prev_stats_millis = 0, pms_data_valid_time = 0;
int brightness = 0;
bool fading_up = true, pms_valid = false;
int smoke_samples = 0;

void loop() {

    // idle; update sensors every second
    if (millis() - last_sensor_sample_time >= 1000) {
        last_sensor_sample_time = millis();
        updateSensors();
        updateLEDs();
        showStats();
    }

    if (millis() - lastSampleTime >= sampleTimerInterval) {
        lastSampleTime = millis();
        
        // temp & humidity; ENV_ALERT
        if (bme_temperature >= TEMP_THRESHOLD || bme_humidity < HUM_THRESHOLD) {
            flag1_env_alert = true; // raise env alert flag 
            sampleTimerInterval = 30000; // 30 secs
        }

        // CO level; CO_ALERT
        if (flag1_env_alert && (co_ppm >= CO_THRESHOLD)) {
            flag2_co_alert = true; // raise CO alert flag
        }
        
        // smoke presence; SMOKE_ALERT
        if (flag2_co_alert && (pms_2_5 > SMOKE_THRESHOLD)) {
            smoke_samples++; // increment smoke samples by 1
        }

        if (smoke_samples == 3) {
            smoke_samples = 0; // reset smoke samples
            flag3_smoke_alert = true; // raise smoke alert flag
        }

        // CRITICAL (fire present)
        if (flag1_env_alert && flag2_co_alert && flag3_smoke_alert) {

            // reset flags
            flag1_env_alert = false;
            flag2_co_alert = false;
            flag3_smoke_alert = false;

            // reset sampling timer interval
            sampleTimerInterval = 120000; // back to 2 mins sampling

            // transmit to LoRa
            String msg = 
                "<" + 
                String(flicker_id) + 
                ">T:" + String(bme.getTemperature()) +
                ",H:" + String(bme.getHumidity()) +
                ",CO:" + String(voltageToPpm(readVoltageV())) +
                ",PM2.5:" + String(pms_2_5) + 
                ",ToA:" + String(last_ToA);

            sendLoRaMessage(msg);
            delay(300000); // delay 5 mins for next transmission; prevents spamming
        }
    }    

    // unsigned long elapsed = millis() - prev_millis;

    // if (fading_up) {
    //     brightness = map(elapsed, 0, fade_interval, 0, 255);

    //     if (elapsed >= fade_interval) {
    //         brightness = 255;
    //         fading_up = false;
    //         prev_millis = millis();
    //     }
    // }
    // else {
    //     brightness = map(elapsed, 0, fade_interval, 255, 0);

    //     if (elapsed >= fade_interval) {
    //         brightness = 0;
    //         fading_up = true;
    //         prev_millis = millis();
    //     }
    // }

    // analogWrite(2, brightness);

    getPMSData();
    if (pms_1 != 0.0 && pms_2_5 != 0.0 && pms_10 != 0.0 && !pms_valid) {
        pms_data_valid_time = millis();
        pms_valid = true;
    }
}

void updateSensors() {
    getPMSData();
    co_voltage = readVoltageV();
    co_ppm = voltageToPpm(co_voltage);
    bme_temperature = bme.getTemperature();
    bme_humidity = bme.getHumidity();
    bme_pressure = bme.getPressure();
    bme.calAltitude(SEA_LEVEL_PRESSURE, bme_pressure);
}

void testTransmit() {
    String msg = 
        "<" + 
        String(flicker_id) + 
        ">T:" + String(bme.getTemperature()) +
        ",H:" + String(bme.getHumidity()) +
        ",CO:" + String(voltageToPpm(readVoltageV())) +
        ",PM2.5:" + String(pms_2_5) + 
        ",ToA:" + String(last_ToA);

    sendLoRaMessage(msg);
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
    
    // Take multiple samples for noise reduction
    for (int i = 0; i < co_samples; ++i) {
        sum += analogRead(co_sensor);
        delay(5);
    }
    
    float raw = sum / (float)co_samples;  // Average ADC value (0..4095)
    
    // ESP32 ADC is 12-bit (0-4095) and measures 0-3.3V with 11dB attenuation
    // However, ESP32 ADC is non-linear, especially near the rails
    // For better accuracy, keep readings in the 0.1V - 2.6V range
    float voltage = raw * (3.3f / 4095.0f);
    
    return voltage;
}

float voltageToPpm(float v) {
    // SEN0564 specifications:
    // - Detection range: 5-5000 ppm
    // - Load resistance: 4.7kΩ
    // - Sensitivity: R0(air)/Rs(150ppm CO) ≥ 3
    // - Operating voltage: 3.3-5V
    
    // MEMS sensors typically show decreasing resistance with increasing CO concentration
    // This means voltage increases with CO concentration
    
    // Voltage divider: Vout = Vcc * RL / (Rs + RL)
    // Rs = sensor resistance, RL = 4.7kΩ
    // Higher CO → Lower Rs → Higher Vout
    
    // For QUALITATIVE estimation only:
    // Assuming linear approximation (NOT ACCURATE for MEMS sensors!)
    // Clean air (~0-1 ppm): ~0.4-0.6V
    // Low CO (5-50 ppm): ~0.6-1.0V
    // Medium CO (50-500 ppm): ~1.0-2.0V
    // High CO (500-5000 ppm): ~2.0-3.0V
    
    // Simple piecewise linear approximation
    float ppm;
    
    if (v < 0.6f) {
        // Below detection threshold - likely clean air
        ppm = 0.0f;
    }
    else if (v < 1.0f) {
        // Low range: 0.6V = 5ppm, 1.0V = 50ppm
        ppm = (v - 0.6f) * ((50.0f - 5.0f) / (1.0f - 0.6f)) + 5.0f;
    }
    else if (v < 2.0f) {
        // Medium range: 1.0V = 50ppm, 2.0V = 500ppm
        ppm = (v - 1.0f) * ((500.0f - 50.0f) / (2.0f - 1.0f)) + 50.0f;
    }
    else if (v < 3.0f) {
        // High range: 2.0V = 500ppm, 3.0V = 5000ppm
        ppm = (v - 2.0f) * ((5000.0f - 500.0f) / (3.0f - 2.0f)) + 500.0f;
    }
    else {
        // Above 3.0V - sensor saturated
        ppm = 5000.0f;
    }
    
    // Clamp to sensor's specified range
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
    unsigned long startTime = micros();

    LoRa.beginPacket();
    LoRa.print(message);
    LoRa.endPacket();

    unsigned long endTime = micros();
    last_ToA = endTime - startTime;

    // upload last ToA to preferences
    preferences.begin(pref_namespace, false);
    preferences.putULong("last_ToA", last_ToA);
    preferences.end();

    Serial.println("Time on Air: " + String(last_ToA) + " μs");
}

float peak_pms_1 = 0, peak_pms_2_5 = 0, peak_pms_10 = 0, peak_co_voltage = 0, peak_co_ppm = 0, peak_temp = 0, peak_hum = 0;
void showStats() {
    Serial.println("PMS data valid time: " + String(pms_data_valid_time / 1000.0) + " s");
    Serial.println("PM 1.0 (ug/m3):\t\t" + String(pms_1) + "\tPeak: " + String(peak_pms_1));
    Serial.println("PM 2.5 (ug/m3):\t\t" + String(pms_2_5) + "\tPeak: " + String(peak_pms_2_5));
    Serial.println("PM 10.0 (ug/m3):\t" + String(pms_10) + "\tPeak: " + String(peak_pms_10));
    Serial.println("voltage (v):\t\t" + String(co_voltage) + "\tPeak: " + String(peak_co_voltage));
    Serial.println("ppm:\t\t\t" + String(co_ppm) + "\tPeak: " + String(peak_co_ppm));
    Serial.println("temperature (°C):\t" + String(bme_temperature) + "\tPeak: " + String(peak_temp));
    Serial.println("humidity (%):\t\t" + String(bme_humidity) + "\tPeak: " + String(peak_hum));
    Serial.println("Last ToA:\t\t" + String(last_ToA) + " μs");
    Serial.println("Flags:\t\t\t" + String(flag1_env_alert) + "-" + String(flag2_co_alert) + "-" +String(flag3_smoke_alert));
    Serial.println("Sampling Interval:\t" + String(sampleTimerInterval) + " ms");
    Serial.println();

    if (pms_1 > peak_pms_1) 
        peak_pms_1 = pms_1;

    if (pms_2_5 > peak_pms_2_5)
        peak_pms_2_5 = pms_2_5;

    if (pms_10 > peak_pms_10)
        peak_pms_10 = pms_10;

    if (co_voltage > peak_co_voltage)
        peak_co_voltage = co_voltage;

    if (co_ppm > peak_co_ppm)
        peak_co_ppm = co_ppm;

    if (bme_temperature > peak_temp)
        peak_temp = bme_temperature;

    if (bme_humidity > peak_hum)
        peak_hum = bme_humidity; 
}