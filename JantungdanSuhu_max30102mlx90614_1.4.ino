#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Adafruit_MLX90614.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

//konfigurasi autentikasi platform adafruit
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "mdzaka"
#define AIO_KEY         "aio_ckuv04C66EopIHqbVatltObOgQ0Q"

//pendeklarasian objek sesuai class yang digunakan pada library
WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish Detak = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Detak");
Adafruit_MQTT_Publish Suhu = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Suhu");
Adafruit_MQTT_Publish statusDetak = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/statusDetak");
Adafruit_MQTT_Publish statusSuhu = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/statusSuhu");

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX30105 maxSensor;

//pendeklarasian pin buzzer D8
byte buzzer = 15; //gpio

//variabel global yang digunakan
const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
long irValue;
long delta;
unsigned long displayTimer=0;
unsigned long displayInterval=500;
unsigned long mesurementStart;
const unsigned long mesurementDuration=15000;

float beatsPerMinute;
int beatAvg;
float bodyTemp;
String heartDiag;
String tempDiag;

//inisiasi awal saat mikrokontroler booting
void setup()
{
  WiFiManager WM;
  pinMode(buzzer, OUTPUT);
  beep();
  Serial.begin(115200);
  // WM.resetSettings();
  WM.setConnectTimeout(10);
  //WM.autoConnect("HeartTempMonitor","password"); // password protected ap

  while (!WM.autoConnect("HeartTempMonitor","password")) { //looping untuk mencoba mengkoneksikan ke ap tersimpan/ konfigurasi ulang
    Serial.println("Gagal Terhubung");
  }
  Serial.println("WiFi Terhubung");
  MQTT_connect(); //mengkoneksikan ke mqtt platform

  // cek MLX90614 sensor
  if (!mlx.begin())
  {
    Serial.println("Sensor MLX90614 tidak terdeteksi !");
    while (1);
  }
  //membaca emisitas
  Serial.print("Emissivity = ");
  Serial.println(mlx.readEmissivity());
  Serial.println("================================================");

  // cek MAX30105 sensor
  if (!maxSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    Serial.println("Sensor MAX30102 tidak terdeteksi !");
    while (1);
  }

  maxSensor.setup();                        // inisiasi sensor max
  maxSensor.setPulseAmplitudeRed(0x0A);      // Turn Red LED to low to indicate sensor is running
  maxSensor.setPulseAmplitudeGreen(0);       // Turn off Green LED
}

void loop()
{
  if (maxSensor.getIR() >= 50000){
    beep();
    Serial.println("Melakukan pengukuran...");
    mesurementStart=millis();
    while (millis() - mesurementStart < mesurementDuration ) {
      mesurement(); //heart rate mesurement
      bodyTemp=mlx.readObjectTempC();
      bodyTemp= (0.2102*bodyTemp)+29.705; //y = 0,2102x + 29,705
    }
    //menampilkan pada serial monitor
    beep2();
    display();
    //kirim data sensor
    sendData();
  }
  else{
    if(millis()-displayTimer>=displayInterval){
      displayTimer=millis();
      Serial.println("Letakan jari");
    }
  }
  MQTT_connect(); //cek koneksi mqtt

}

void sendData(){
  if(beatAvg<=95){
    heartDiag="Bradikardia";
  }
  else if(beatAvg>=170){
    heartDiag="Takikardia";
  }
  else{
    heartDiag="Normal";
  }
  if(bodyTemp<35.0){
    tempDiag="Hipotermia";
  }
  else if(bodyTemp>=37.5){
    tempDiag="Demam";
  }
  else{
    tempDiag="Normal";
  }

  Detak.publish(beatAvg);
  Suhu.publish(bodyTemp,3);
  statusDetak.publish(heartDiag.c_str());
  statusSuhu.publish(tempDiag.c_str());
  Serial.println("Data Terkirim !");
  //reset value
  beatsPerMinute=0;
  beatAvg=0;
  delay(2000);
}

void display(){
  // Read temperature from MLX90614 sensor
  Serial.print("Object = ");
  Serial.print(bodyTemp);
  Serial.println("*C");
    
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.println(beatAvg);
  Serial.println();
  
}
void mesurement(){
  // Read heart rate from MAX30105 sensor
  irValue = maxSensor.getIR();
  // Check for heart beat
  if (checkForBeat(irValue))
  {
    // We sensed a beat!
    delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
      rateSpot %= RATE_SIZE;                    // Wrap variable

      // Take average of readings
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
}
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Menghubungkan ke platform MQTT ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Menghubungkan ulang dalam 5 detik");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT terhubung!");
}
void beep(){
  digitalWrite(buzzer, HIGH);
  delay(80);
  digitalWrite(buzzer, LOW);
}
void beep2(){
  digitalWrite(buzzer, HIGH);
  delay(50);
  digitalWrite(buzzer, LOW);
  delay(50);
  digitalWrite(buzzer, HIGH);
  delay(50);
  digitalWrite(buzzer, LOW);
}
