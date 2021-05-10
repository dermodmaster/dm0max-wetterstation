#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <ArduinoMqttClient.h>
#include <OneWire.h> 
#include <DallasTemperature.h>


// Networking details
byte mac[] = { 0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED }; // Ethernet shield (W5100) MAC address
IPAddress ip(192, 168, 178, 188); // Ethernet shield (W5100) IP address
IPAddress server(192,168,100,24); // MTTQ server IP address

EthernetClient ethClient;
MqttClient mqttClient(ethClient);

// MQTT Settings
const char broker[] = "192.168.100.24";
int        port     = 1883;
const char topic[]  = "wetter1/listen";
const char bme280_temp_topic[] = "wetter1/bme280/temperature";
const char bme280_humidity_topic[] = "wetter1/bme280/humidity";
const char bme280_pressure_topic[] = "wetter1/bme280/pressure";
const char temp1_temp_topic[] = "wetter1/temp1/temperature";
const char wind_speed_topic[] = "wetter1/wind/speed";

// Anemometer Vars
const unsigned long wind_measure_intervall = 5;      //Measuretime in Seconds
unsigned long lastWindMeasurement = 0;
int wind_ct = 0;
float wind = 0.0;
unsigned long time = 0;

// DS1820 Temp Sensor
const unsigned long temp1_measure_intervall = 1*1000UL;      //Measuretime in Seconds
unsigned long lastTemp1Measurement = 0;
// Data wire is plugged into pin 2 on the Arduino 
#define ONE_WIRE_BUS 2 
// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS); 
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

void connectToEventServer(){
  if(!mqttClient.connect(broker, port)){
    Serial.println(mqttClient.connectError());
    Serial.println("MQTT can't establish connection... try again");
    delay(2000);
    connectToEventServer();
  }
  
}

void setup()
{
  Serial.begin(9600);
  sensors.begin(); // Setup DallasTemperature Sensors

  // Ethernet shield configuration
  Ethernet.begin(mac, ip);
  
  delay(2500); // Allow hardware to stabilize
  

  // Setup anemometer
  pinMode(13, OUTPUT);
  time = millis();
  attachInterrupt(1, countWind, RISING);

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  connectToEventServer();

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  Serial.print("Subscribing to topic: ");
  Serial.println(topic);
  Serial.println();

  // subscribe to a topic
  mqttClient.subscribe(topic);

  // topics can be unsubscribed using:
  // mqttClient.unsubscribe(topic);

  Serial.print("Waiting for messages on topic: ");
  Serial.println(topic);
  Serial.println();
}

void countWind() {
  wind_ct ++;
  digitalWrite(13, HIGH);
  delay(50);
  digitalWrite(13, LOW);
}

void measure_anemometer(){
  Serial.print("Windcount");
  Serial.println(wind_ct);
  wind = (float)wind_ct / (float)wind_measure_intervall * 2.4;
  wind_ct = 0;
  time = millis();
}

void sendSensorValue(char topic[], float value){
  mqttClient.beginMessage(topic);
  mqttClient.print(value);
  mqttClient.endMessage();
}

void loop() {
  //Check MQTT Connection Status
  if(!mqttClient.connected()){
    connectToEventServer();
  }
  
   //Read MQTT messages
  int messageSize = mqttClient.parseMessage();
  if (messageSize) {
    // we received a message, print out the topic and contents
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', length ");
    Serial.print(messageSize);
    Serial.println(" bytes:");

    // use the Stream interface to print the contents
    while (mqttClient.available()) {
      Serial.print((char)mqttClient.read());
    }
    Serial.println();
    Serial.println();
  }


  
  // Read Anemometer
  if(millis() - lastWindMeasurement > (wind_measure_intervall*1000UL)){
    lastWindMeasurement = millis();
    measure_anemometer();
    sendSensorValue(wind_speed_topic, wind);
    Serial.print("Windgeschwindigkeit: ");
    Serial.print(wind);       //Speed in Km/h
    Serial.println(" km/h - ");
  }

    // Read Temp1 Sensor
  if(millis() - lastTemp1Measurement > temp1_measure_intervall){
    lastTemp1Measurement = millis();
    sensors.requestTemperatures(); // Send the command to get temperature readings 
    float temp1 = sensors.getTempCByIndex(0);
    sendSensorValue(temp1_temp_topic, temp1);
    Serial.print("Temp1 is: "); 
    Serial.println(temp1);
  }
 
}
