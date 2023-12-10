#include <Arduino.h>
#include <Wire.h>

#include <SparkFun_Qwiic_Humidity_AHT20.h>
#include <Adafruit_BMP280.h>

AHT20 humiditySensor;

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();


void setup() {
  Serial.begin(9600);  // 9600 бод скорость
  while ( ! Serial);  // ждем иницализации порта связи
  Wire.begin();      // инициализация I2C
  
  _delay_ms(500); 

  Serial.println("TEST MESSAGE");

  if(! bmp.begin() ){  // инициализация датчика давления
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  if(! humiditySensor.begin()){  // инициализация датчика влажности и температуры
    Serial.println("Could not find a valid AHT20 sensor, check wiring!");
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL);

  bmp_temp->printSensorDetails();
  
}

uint64_t timer = 0;

//aht20
float humidity = 0;
float temperature = 0;

sensors_event_t temp_event, pressure_event;
void loop() {
  if(millis() - timer >= 1000){
    bmp_temp->getEvent(&temp_event);
    bmp_pressure->getEvent(&pressure_event);

        
    if(humiditySensor.available()){
      humidity = humiditySensor.getHumidity();
      temperature = humiditySensor.getTemperature();
    }
    Serial.println("================================================");
    
    Serial.print("Pressure: ");
    Serial.println(pressure_event.pressure, 3);

    Serial.print("Temperature AHT20: ");
    Serial.println(temperature,3);

    Serial.print("Temperature BMP280: ");
    Serial.println(temp_event.temperature, 3);

    Serial.print("Humidity: ");
    Serial.println(humidity,3);

    Serial.println("================================================");
    
    timer = millis();
  }
}

