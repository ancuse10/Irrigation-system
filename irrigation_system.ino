//Librerias utilizadas en el proyecto
#include <WiFi.h>           // WiFi control for ESP32
#include <Adafruit_BME280.h>
#include <ThingsBoard.h>    // ThingsBoard SDK

// WiFi access point
#define WIFI_AP_NAME        "***********"
// WiFi password
#define WIFI_PASSWORD       "************"

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))    //Funcion empleada para contar el numero de callbacks automaticamente

#define TOKEN               "ESP32_TOKEN"   //Token de acceso del dispositivo en ThingsBoard

#define THINGSBOARD_SERVER  "*********"     //IP del servidor ThingsBoard

// Baud rate for debug serial
#define SERIAL_DEBUG_BAUD    9600

// Initialize ThingsBoard client
WiFiClientSecure espClient;

PubSubClient client(espClient);

// Initialize ThingsBoard instance
ThingsBoard tb(espClient);

// the Wifi radio's status
int status = WL_IDLE_STATUS;

//Variables booleanas para controlar el estado de los LEDs en los callbacks
boolean switchState, switchState2;

//Definicion de los pines
const int humedadPin1 = 34;
const int humedadPin2 = 32;
const int relePin1 = 14;
const int relePin2 = 15;
const int ledRojo = 25;
const int ledAzul = 4;

// BME object
Adafruit_BME280 bme;

// Main application loop delay
int quant = 20;

// Period of sending a temperature/humidity data.
int send_delay = 2000;

// Time passed after temperature/humidity data was sent, milliseconds.
int send_passed = 0;

//Variables para identificar a los dos relés
int rele1, rele2;

//Variables para convertir el valor de la humedad de analogico a digital y enviarla por telemetría
float lecturaAnalogica1, hum_suelo1, lecturaAnalogica2, hum_suelo2;



//////////////////////////////////////////////////////////

        ////////        RPC          ////////

/////////////////////////////////////////////////////////

// Set to true if application is subscribed for the RPC messages.
bool subscribed = false;

//Funcion para encender y apagar el primer LED y relé manualmente tras haber recibido un mensaje RPC Request
  RPC_Response setLedState(const RPC_Data &data){
    switchState = data;
    Serial.print("Received switch 1 state: ");
    Serial.println(switchState);
    digitalWrite(ledRojo, switchState);    // switch Industruino LCD backlight

    if(switchState){
      digitalWrite(relePin1, LOW);      //Encendemos el relé cuando ponemos el pin a LOW y lo apagamos cuando ponemos el pin a HIGH
    }else{
      digitalWrite(relePin1, HIGH);
    }
    
    return RPC_Response(NULL, switchState);   //Devolvemos true si esta encendido o false si esta apagado
}

//Funcion para encender y apagar el segundo LED y relé manualmente tras haber recibido un mensaje RPC Request
  RPC_Response setLed2State(const RPC_Data &data){
    switchState2 = data;
    Serial.print("Received switch 2 state: ");
    Serial.println(switchState2);
    digitalWrite(ledAzul, switchState2);    // switch Industruino LCD backlight

    if(switchState2){
      digitalWrite(relePin2, LOW);
    }else{
      digitalWrite(relePin2, HIGH);
    }
    
    return RPC_Response(NULL, switchState2);  //Devolvemos true si esta encendido o false si esta apagado
}

//Funcion para obtener el estado del LED y que no se pierda la información al recargar la página por ejemplo
  RPC_Response getLedState(const RPC_Data &data){
    Serial.println("Received the getLed1State method");
    return RPC_Response(NULL, switchState);  // Devolviendo el valor actual de la variable para actualizar el widget
}

//Funcion para obtener el estado del LED y que no se pierda la información al recargar la página por ejemplo
  RPC_Response getLed2State(const RPC_Data &data){
    Serial.println("Received the getLed1State method");
    return RPC_Response(NULL, switchState2);  // Devolviendo el valor actual de la variable para actualizar el widget
}

//Recibimos los valores enviados desde ThingsBoard a traves de un mensaje RPC e interpretamos los datos para encender o apagar
//los LEDs y los relés
  RPC_Response getReleValue(const RPC_Data &data){
    Serial.println("Received the get rele 1 RPC method");

    // Process data

    if(data["bomba"] == 1 && data["rele"] == 1){

      rele1 = data["rele"];

      if(!switchState){
        digitalWrite(ledRojo, HIGH);
        digitalWrite(relePin1, LOW);
      }
    }
    else if(data["bomba"] == 1 && data["rele"] == 0){
      rele1 = data["rele"];

      if(!switchState){
        digitalWrite(ledRojo, LOW);
        digitalWrite(relePin1, HIGH);
      }
    }

    if(data["bomba"] == 2 && data["rele"] == 1){

      rele2 = data["rele"];

      if(!switchState2){
        digitalWrite(ledAzul, HIGH);
        digitalWrite(relePin2, LOW);
      }
    }
    else if(data["bomba"] == 2 && data["rele"] == 0){
      rele2 = data["rele"];

      if(!switchState2){
        digitalWrite(ledAzul, LOW);
        digitalWrite(relePin2, HIGH);
      }
    }

    // Just an response example
    return RPC_Response(NULL, rele1);
}

// RPC handlers
  RPC_Callback callbacks[] = {
  { "setValue", setLedState }, 
  { "getValue", getLedState},
  { "getValue2", getLed2State},
  { "setValue2", setLed2State},
  { "getRele", getReleValue},
};


//////////////////////////////////////////////////////////

        ////////        END RPC          ////////

/////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////

        ////////        SETUP          ////////

/////////////////////////////////////////////////////////

// Setup an application
void setup() {
  // Initialize serial for debugging
  Serial.begin(SERIAL_DEBUG_BAUD);
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  InitWiFi();

  client.setServer(THINGSBOARD_SERVER, 8883);   //Establecemos nuestro servidor y el puerto 8883 (Secure MQTT)
  
  // Pinconfig

  pinMode(ledRojo, OUTPUT);
  pinMode(ledAzul, OUTPUT);
  pinMode(humedadPin1, INPUT);
  pinMode(humedadPin2, INPUT);
  pinMode(relePin1, OUTPUT);
  pinMode(relePin2, OUTPUT);

  digitalWrite(relePin1, HIGH);
  digitalWrite(relePin2, HIGH);

  // Initialize temperature sensor
  bme.begin();

}


//////////////////////////////////////////////////////////

        ////////        END SETUP          ////////

/////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////

        ////////        LOOP          ////////

/////////////////////////////////////////////////////////

// Main application loop
void loop() {
  delay(quant);

  send_passed += quant;
  

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
    delay(5000);
    
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Failed to connect");
      return;
    }
  }

  // Subscribe for RPC, if needed
  if (!subscribed) {
    Serial.println("Subscribing for RPC...");

    // Perform a subscription. All consequent data processing will happen in
    // callbacks as denoted by callbacks[] array.
    if (!tb.RPC_Subscribe(callbacks, COUNT_OF(callbacks))) {
      Serial.println("Failed to subscribe for RPC");
      return;
    }

    Serial.println("Subscribe done");
    subscribed = true;
  }

  // Check if it is a time to send BME280 temperature and humidity
  if (send_passed > send_delay) {
    Serial.println("Sending data...");

  // Uploads new telemetry to ThingsBoard using MQTT.
  //Enviamos la telemetria en formato JSON. Para ellos creamos dos variables de tipo DynamicJsonDocument
  //y una vez les hemos asignado los valores que queremos enviar con sus identificadores correspondientes
  //las serializamos y las guardamos en una variable de tipo char [] y las enviamos
  //mediante el comando sendTelemetryJson, funcion incluida en la librería ThingsBoard.h
  float temp;
  float hum_aire;
  DynamicJsonDocument p1(1024);
  DynamicJsonDocument p2(1024);
  char planta1[100];
  char planta2[100];

  temp = bme.readTemperature();
  hum_aire = bme.readHumidity();
  lecturaAnalogica1 = analogRead(humedadPin1);
  lecturaAnalogica2 = analogRead(humedadPin2);
  hum_suelo1 = map(lecturaAnalogica1, 0, 4095, 100, 0);   //Convertimos el valor analogico entre 0 y 4095 a un valor entre 0 y 100, siendo 0 = 100 y 4095 = 0
  hum_suelo2 = map(lecturaAnalogica2, 0, 4095, 100, 0);

  p1["id"] = 1;
  p1["humedad_suelo"] = hum_suelo1;
  p1["temperature"] = temp;
  p1["humedad_aire"] = hum_aire;

  p2["id"] = 2;
  p2["humedad_suelo_2"] = hum_suelo2;
  p2["temperature"] = temp;
  p2["humedad_aire"] = hum_aire;

  serializeJson(p1, planta1);
  serializeJson(p2, planta2);
  
    if (isnan(temp) || isnan(hum_aire)) {
      Serial.println("Failed to read from sensor!");
    } else {
      Serial.print("Humedad de la planta 1: ");
      Serial.println(hum_suelo1);
      delay(500);

      Serial.print("Humedad de la planta 2: ");
      Serial.println(hum_suelo2);
      delay(500);

      tb.sendTelemetryJson(planta1);
      tb.sendTelemetryJson(planta2);
    }

    send_passed = 0;
  }

  //leeDatos();

  // Process messages
  tb.loop();
}


//////////////////////////////////////////////////////////

        ////////        END LOOP          ////////

/////////////////////////////////////////////////////////





//////////////////////////////////////////////////////////

        ////////        WIFI          ////////

/////////////////////////////////////////////////////////

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
