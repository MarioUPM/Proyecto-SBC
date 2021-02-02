#include <WiFi.h>           // WiFi control for ESP32
#include <ThingsBoard.h>    // ThingsBoard SDK

/////////////////AIR_QUALITY SENSOR/////////////////////
#include "Air_Quality_Sensor.h"

AirQualitySensor sensor(35);
//////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////

// WiFi access point
#define WIFI_AP_NAME        ""
// WiFi password
#define WIFI_PASSWORD       ""

// See https://thingsboard.io/docs/getting-started-guides/helloworld/ 
// to understand how to obtain an access token
#define TOKEN               "NeybRCbAM2RjCEyERr9z"
// ThingsBoard server instance.
#define THINGSBOARD_SERVER  "iot.etsisi.upm.es"

// Baud rate for debug serial
#define SERIAL_DEBUG_BAUD    115200
#define timeSeconds 3

////////////////////////////Botón/////////////////////////////////////
#define Button 17 
////////////////////////////////////////////////////////////////////////////

////////////////////////////LED/////////////////////////////////////
const int Led = 22;
const int Led1 = 23;
////////////////////////////////////////////////////////////////////////////

//////////////////////M8X8//////////////////////////////
#include "LedMatrix.h"
#define NUMBER_OF_DEVICES 1 //number of led matrix connect in series
#define CS_PIN 15
#define CLK_PIN 14
#define MISO_PIN 2 //Este pin no se usa, lo ponemos para completar el constructor
#define MOSI_PIN 12

LedMatrix ledMatrix = LedMatrix(NUMBER_OF_DEVICES, CLK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
//////////////////////////////////////////////////////////

////////////////////////UltrS/////////////////////////////
const int Trigger = 2;   //Pin digital 2 para el Trigger del sensor
const int Echo = 27;   //Pin digital 3 para el Echo del sensor

int personas=0;

///////////////////////////////////////////////////////
int val=0;


// Initialize ThingsBoard client
WiFiClient espClient;
// Initialize ThingsBoard instance
ThingsBoard tb(espClient);
// the Wifi radio's status
int status = WL_IDLE_STATUS;


// Main application loop delay
int quant = 10;

// Initial period of LED cycling.
int led_delay = 10;
// Period of sending a temperature/humidity data.
int send_delay = 20;

// Time passed after LED was turned ON, milliseconds.
int led_passed = 0;
// Time passed after temperature/humidity data was sent, milliseconds.
int send_passed = 0;

// Set to true if application is subscribed for the RPC messages.
bool subscribed = false;

//////////////////DECLARACIONES SENSOR POLVO////////////////////////////
const int pinPolvo = 19;
float ratio = 0;
float concentration = 0;
/////////////////////////////////////////////////////////////

//////////////////Variables Sensor Polvo/////////////////////////
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000;//sampe 30s ;
unsigned long lowpulseoccupancy = 0;

/////////////////////////////////////////////////////////////////


void setup() {
  // Initialize serial for debugging
    Serial.begin(115200);
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  InitWiFi();
//////////////////////BOTON//////////////////////////////
  pinMode(Button, INPUT);
  //////////////////////////////////////////////////////////
  
//////////////////////LED//////////////////////////////
  pinMode(Led, OUTPUT);
  pinMode(Led1, OUTPUT);
//////////////////////////////////////////////////////////
  
//////////////////////M8X8//////////////////////////////
  ledMatrix.init();
  int i=0;
  ledMatrix.setText(String(personas));
  //////////////////////////////////////////////////////////
  
////////////////////////UltrS/////////////////////////////
  pinMode(Trigger, OUTPUT); //pin como salida
  pinMode(Echo, INPUT);  //pin como entrada
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0
  ///////////////////////////////////////////////////////    

/////////////////////////////SENSOR POLVO///////////////////////////////

   pinMode(pinPolvo,INPUT);
   starttime = millis();//get the current time;
////////////////////////////////////////////////////////////////////////
   
////////////////////////////AIR_QUEALITY SENSOR/////////////////////////
   while (!Serial);

    Serial.println("Waiting sensor to init...");
    delay(20000);

    if (sensor.init()) {
        Serial.println("Sensor ready.");
    } else {
        Serial.println("Sensor ERROR!");
    }
////////////////////////////////////////////////////////////////////////
  
}



void loop() {
  
  //delay(quant);

  led_passed += quant;
  send_passed += quant;


////////////////////////////BOTON////////////////////////////////////////
  int ButtonState = digitalRead(Button); // read the button pin and store in ButtonState variable

  if(ButtonState == HIGH)  // if button is pressed
  {
    personas=0;

  }
/////////////////////////////////////////////////////////////////////////

///////////////////////////////////////M8X8//////////////////////////////
  ledMatrix.setNextText(String(personas));
  ledMatrix.clear();
  ledMatrix.scrollTextLeft();
  ledMatrix.drawText();
  ledMatrix.commit();
  /////////////////////////////////////////////////////////
  
  ////////////////////////UltrS/////////////////////////////

  long t; //timepo que demora en llegar el eco
  long d; //distancia en centimetros

  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(Trigger, LOW);
  
  t = pulseIn(Echo, HIGH); //obtenemos el ancho del pulso
  d = t/59;             //escalamos el tiempo a una distancia en cm
  
  if(d<7) {
    personas++;
    digitalWrite(Led, HIGH);
    delay(500);
    digitalWrite(Led, LOW);
  }
  
  Serial.print("Personas: ");
  Serial.print(personas);      //Enviamos serialmente el valor de la distancia
  Serial.println();
       
/////////////////////////////////////////////////////////////////////////   

//////////////////////SENSOR POLVO///////////////////////////////////////
    duration = pulseIn(pinPolvo, LOW);
    lowpulseoccupancy = lowpulseoccupancy+duration;

    if ((millis()-starttime) > sampletime_ms)//if the sampel time == 30s
    {
        ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
        concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
        Serial.print(lowpulseoccupancy);
        Serial.print(",");
        Serial.print(ratio);
        Serial.print(",");
        Serial.println(concentration);
        lowpulseoccupancy = 0;
        starttime = millis();
    }
////////////////////////////////////////////////////////////////////////
////////////////////////////AIR_QUEALITY SENSOR/////////////////////////
  int quality = sensor.slope();

    Serial.print("Sensor value: ");
    Serial.println(sensor.getValue());

    if (sensor.getValue() > 3000) {
        Serial.println("¡¡¡¡¡¡DANGEROUS POLLUTION CONCENTRATION!!!!");
            digitalWrite(Led1, HIGH);
            delay(500);
            digitalWrite(Led1, LOW);
    } else if (sensor.getValue() > 1600) {
        Serial.println("High pollution!");
    } else if (sensor.getValue() > 750) {
        Serial.println("Low pollution!");
    } else {
        Serial.println("Fresh air.");
    }

////////////////////////////////////////////////////////////////////////  

  // Reconnect to WiFi, if needed
  if (WiFi.status() != WL_CONNECTED) {
    reconnect();
    return;
  }
  // Reconnect to ThingsBoard, if needed
  if (!tb.connected()) {
    subscribed = false;
    
    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Failed to connect");
      return;
    }
  }

//////////////////////////////////////////////////////////////////////////////////
////////////////////////////////ENVIO DE DATOS///////////////////////////////////
  // Check if it is a time to send DHT22 temperature and humidity
  if (send_passed > send_delay) {
    Serial.println("Sending data...");

      //POLVO
      tb.sendTelemetryFloat("Concentracion", concentration);
      //AIR_QUALITY
      tb.sendTelemetryFloat("Calidad_del_Aire", sensor.getValue());
      //CONTADOR_PERSONAS      
      Serial.print("Personas detectadas con COVID: ");
      Serial.print(personas);
      Serial.println("");
      tb.sendTelemetryFloat("Personas", personas);

    send_passed = 0;
  }

  // Process messages
  tb.loop();
}

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void reconnect() {
  // Loop until we're reconnected
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }
}
