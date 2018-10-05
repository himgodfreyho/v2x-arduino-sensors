#include <SoftwareSerial.h>
#include <SPI.h>
#include <EEPROM.h>
#include <boards.h>
#include <RBL_nRF8001.h>
#include <DHT.h>

/* ===== TEMPERATURE/HUMIDITY ===== */
#define DHTPIN A0     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);

/* ===== BLUETOOTH ===== */
const int bluetoothREQN = 7;
const int bluetoothRDYN = 6;

/* ===== DUST ===== */
int pin = 8;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 3000; //sampe 3000
unsigned long lowpulseoccupancy = 0;
float dust_ratio = 0;
float concentration = 0;

/* ===== GAS ===== */
const float R0 = 1.31;      // Obtained from experiment
float gas_ratio = 0;

/* ===== BLUETOOTH SENDING ORDERS ===== */
int currnet_order = 1;
int max_order = 5;

#define ORDER_TEMPERATURE   1
#define ORDER_HUMIDITY      2
#define ORDER_DUST          3
#define ORDER_UV            4
#define ORDER_GAS           5

char sensorChar[6] = {'0', 'T', 'H', 'D', 'U', 'G'};

void setup()
{
    Serial.begin(9600);
    
    setup_bluetooth_pins_and_name();
    
    dht.begin();
    pinMode(8, INPUT);
    starttime = millis();
}

void setup_bluetooth_pins_and_name()
{
    ble_set_pins(bluetoothREQN, bluetoothRDYN);
    char name[] = "V2X-BLE";   // Declared to suppress warning
    ble_set_name(name);
    ble_begin();
}

unsigned char * get_char_array_for_writing(int order, float value)
{
    char order_string[8] = {sensorChar[order], 0, 0, 0, 0, 0, 0, 0};
    char value_string_array[7];
    dtostrf(value, 6, 3, value_string_array);
    String value_string = String(value_string_array);
    value_string.trim();
    
    char value_string_trimmed[value_string.length()];
    value_string.toCharArray(value_string_trimmed, value_string.length());
    strcat(order_string, value_string_trimmed);
    
    return (unsigned char *)order_string;
}

void print_value_to_serial(int order, float value) {
    Serial.print(sensorChar[order]);
    Serial.print(":");
    Serial.println(value);
}

void read_and_send_temperature()
{
    float temperature = dht.readTemperature();
    
    if (isnan(temperature))
    {
        Serial.println("Failed to read temperature from DHT");
        return;
    }
    
    print_value_to_serial(ORDER_TEMPERATURE, temperature);
    ble_write_bytes(get_char_array_for_writing(ORDER_TEMPERATURE, temperature), 8);
}

void read_and_send_humidity()
{
    float humidity = dht.readHumidity();
    
    if (isnan(humidity))
    {
        Serial.println("Failed to read temperature from DHT");
        return;
    }
    
    print_value_to_serial(ORDER_HUMIDITY, humidity);
    ble_write_bytes(get_char_array_for_writing(ORDER_HUMIDITY, humidity), 8);
}

float read_dust_value()
{
    duration = pulseIn(pin, LOW);
    lowpulseoccupancy = lowpulseoccupancy + duration;
    
    if ((millis() - starttime) >= sampletime_ms) //if the sample time = = 30s
    {
        dust_ratio = lowpulseoccupancy / (sampletime_ms * 10.0);  // Integer percentage 0=>100
        concentration = 1.1 * pow(dust_ratio, 3) - 3.8 * pow(dust_ratio, 2) + 520 * dust_ratio + 0.62; // using spec sheet curve
        lowpulseoccupancy = 0;
        starttime = millis();
    }
    
    return concentration;
}

void read_and_send_dust()
{
    float concentration = read_dust_value();
    
    print_value_to_serial(ORDER_DUST, concentration);
    ble_write_bytes(get_char_array_for_writing(ORDER_DUST, concentration), 8);
}

float read_uv_index()
{
    int sensorValue;
    long sum = 0;
    for (int i = 0; i < 1024; i++)
    {
        sensorValue = analogRead(A1);
        sum = sensorValue + sum;
        delay(1);
    }
    sum = sum >> 10;
    float voltage_mv = sum * 4980.0 / 1023.0;
    float voltage_v = voltage_mv / 1000.0;
    float intensity = 307.0 * voltage_v;
    float uv = intensity / 200.0;
    
    return uv;
}

void read_and_send_uv()
{
    float uv_index = read_uv_index();
    
    print_value_to_serial(ORDER_UV, uv_index);
    ble_write_bytes(get_char_array_for_writing(ORDER_UV, uv_index), 8);
}

float read_gas_value()
{
    float sensor_volt;
    float RS_gas;
    float ratio;
    int sensorValue = analogRead(A1);
    sensor_volt = (float)sensorValue / 1024 * 5.0;
    
    if (sensor_volt == 0)
    {
        return 0.0;
    }
    
    RS_gas = (5.0 - sensor_volt) / sensor_volt;
    gas_ratio = RS_gas / R0;
    
    return gas_ratio;
}

void read_and_send_gas()
{
    float gas_value = read_gas_value();
    
    print_value_to_serial(ORDER_GAS, gas_value);
    ble_write_bytes(get_char_array_for_writing(ORDER_GAS, gas_value), 8);
}

void advance_order()
{
    currnet_order++;
    if (currnet_order > max_order)
    {
        currnet_order = 1;
    }
}

void loop()
{
    switch (currnet_order)
    {
        case ORDER_TEMPERATURE:
        read_and_send_temperature();
        break;
        case ORDER_HUMIDITY:
        read_and_send_humidity();
        break;
        case ORDER_DUST:
        read_and_send_dust();
        break;
        case ORDER_UV:
        read_and_send_uv();
        break;
        case ORDER_GAS:
        read_and_send_gas();
        break;
    }
    
    delay(100);
    
    ble_do_events();
    advance_order();
}

