#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <PID_v1.h>
#include <Wifi.h>
#include <WiFiClient.h> 
#include <WebServer.h>
#include <ArduinoJson.h>


// DS18B20 senzory
#define ONE_WIRE_BUS 13
#define ONE_WIRE_BUS_2 25
OneWire oneWire(ONE_WIRE_BUS);
OneWire oneWire2(ONE_WIRE_BUS_2);
DallasTemperature sensors(&oneWire);
DallasTemperature sensors2(&oneWire2);

const char* ssid = "Murin";        // WiFi názov
const char* password = "0915806553";   // WiFi heslo

WebServer server(80);  // Vytvorenie web servera na porte 80

// Funkcia na získanie sily signálu (v dBm)
int getSignalStrength() {
  return WiFi.RSSI();
}

// Globálne pre časovač
unsigned long lastWifiAttempt = 0;
bool wifiConnecting = false; 


// Relé piny motora
const int motorPinOpen = 14;
const int motorPinClose = 27;
const long dlzkapulzurele = 1500; // Puly rele ms
const long maxcasmotora = 7000; // sekundy motoraot várania ms
const long dlzkaInitPulzu = 4000; //dlzka init zatvarania ms
const long minPauza = 10000;  //min cas pauzy rele
const long maxPauza = 600000; //max cas pauzy rele

// Časovač pre motor
long casmotora = 0;   // Celkový čas otvorenia motora
long waitingStart = 0; //Cas waiting state
long timeIntitClosing = 0; // Cas init zatvarania
long timeStartOpening = 0; // Cas start otvorenia
long timeStartClosing = 0; // Cas start zatvarania
long pulzePauzaTime = 0; // Cas pauzy rele
long pidLastCalculationTime = 0; // Cas posledného vyčitania PID
long mytime = 0; 
// PID parametre
double setpoint;
double inputTemperature;
double outputPID;
PID ventilationPID(&inputTemperature, &outputPID, &setpoint, 2, 0.1, 1, DIRECT);

// Impulzný režim
unsigned long totalOpenTime = 0; // Celkový čas otvorenia ventilácie
unsigned long relayStartTime = 0;
bool relayActive = false;

unsigned long previousMillis = 0;
const unsigned long interval = 4000;

//State of ventilation
uint8_t ventilationState = 0; 

// Stavy stae machine
#define IitingClosing 0
#define WaitInitFinsh 1
#define StartOpening 2
#define Opening 3
#define StartClosing 4
#define Closing 5
#define Waiting 6

// Stavy stae machine pre print stavov
String stateNames[] = {"initingClosing", "WaitInitFinsh", "StartOpening", "Opening", "StartClosing", "Closing", "Waiting"};

// Stavy
String ventilationDirection = "Stopped";
float temperatureThreshold = 27.0;
float temperatureC = 0.0; // Global variable for the first sensor temperature
float temperatureC2 = 0.0;


void ventilatioStateMachine() {
  //Serial.print("State: " );
  //Serial.println ( stateNames[ventilationState]);
  delay(10);
  Serial.print("cas motora=" ); 
  Serial.println(casmotora);

  switch (ventilationState)
  {
    //Init zatvaranie motora
  case IitingClosing :
    digitalWrite(motorPinOpen, LOW);
    digitalWrite(motorPinClose, HIGH);
    timeIntitClosing = millis();
    ventilationState = WaitInitFinsh;
    break;
    //Cakaj na konec init zatvarania
  case WaitInitFinsh:
  //Serial.println("Waiting for init closing...");
    if (millis() - timeIntitClosing > dlzkaInitPulzu) {
      digitalWrite(motorPinOpen, HIGH);
      digitalWrite(motorPinClose, HIGH);
      casmotora = 0;
      waitingStart = millis();
      ventilationState = Waiting;
    }
    break;
   //Start otvorenia
  case StartOpening:
    //Serial.println("Start opening...");
    if (casmotora < maxcasmotora){
      //Serial.println("1skocilsomStart opening...");
    digitalWrite(motorPinOpen, HIGH);
    digitalWrite(motorPinClose, LOW);
    timeStartOpening = millis();
    casmotora = casmotora + dlzkapulzurele;
    ventilationState = Opening;
    }
    else {
      //Serial.println("2skocilsomStart opening else...");
      digitalWrite(motorPinOpen, HIGH);
      digitalWrite(motorPinClose, HIGH);
      waitingStart = millis();
      
      ventilationState = Waiting; 
    }
    break;
    //Otvorenie moptora
  case Opening:
  //Serial.println("Opening...");
  //Serial.print(casmotora);
  //Serial.println(timeStartOpening);
 mytime = millis() - timeStartOpening;
    Serial.print("time=");
    Serial.println(mytime);

    if ((mytime < dlzkapulzurele)  && casmotora < maxcasmotora) {
      Serial.println("3skocilsomOpening...");
      digitalWrite(motorPinOpen, HIGH);
      digitalWrite(motorPinClose, LOW);
      
      waitingStart = millis();

          }
    else {
      Serial.println("4else Opening...");
      digitalWrite(motorPinOpen, HIGH);
      digitalWrite(motorPinClose, HIGH);
      waitingStart = millis();
            ventilationState = Waiting;
    }
    break;
    //Start zatvarania  moptora
  case StartClosing:
  //Serial.println("Start closing...");
    if(casmotora > 0){
      //Serial.println("1skocilsomStart closing...");
      //Serial.print("casmotora=" );
      //Serial.println(casmotora);
    digitalWrite(motorPinOpen, LOW);
    digitalWrite(motorPinClose, HIGH);
    timeStartClosing = millis();
    casmotora -= dlzkapulzurele;
    ventilationState = Closing;
    }
    else {
      //Serial.println("2skocilsomStart closing else...");
      digitalWrite(motorPinOpen, HIGH);
      digitalWrite(motorPinClose, HIGH);
      waitingStart = millis();
      ventilationState = Waiting; 
    }
    break;
    //Zatvaranie motora
    case Closing:
    if (((millis() - timeStartClosing) < dlzkapulzurele)   && casmotora > 0) {
     
      digitalWrite(motorPinOpen, LOW);
      digitalWrite(motorPinClose, HIGH);
      ventilationState = Closing;
    }
    else {
      digitalWrite(motorPinOpen, HIGH);
      digitalWrite(motorPinClose, HIGH);
      waitingStart = millis();
      ventilationState = Waiting;
    }
    break;
    
  //Cakaj na pid a pauzu pulzu
  case Waiting:
  Serial.println("Waiting...");
    if (millis() - waitingStart > pulzePauzaTime) {
      if (outputPID > 1) {
        ventilationState = StartOpening;
      }
      if (outputPID < -1 ) {
        ventilationState = StartClosing;  
      }
    }
    break;


  default:
    break;
  }
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Monitorovanie</title></head><body>";
  html += "<h1>Monitorovanie ESP32</h1>";
  html += "<h2>Teplota: <span id='temp'>Nacitavam...</span> &deg;C</h2>";
  html += "<h2>Teplotadruhy snimač: <span id='temp2'>Nacitavam...</span> &deg;C</h2>";
  html += "<h2>vystup PID: <span id='output'>Nacitavam...</span></h2>";
  html += "<h2>percento otvorenia: <span id='casmotora'>Nacitavam...</span> %</h2>";
  html += "<h2>Signál WiFi: <span id='rssi'>Nacitavam...</span> dBm</h2>";
  html += "<canvas is='Teplota1' width='400' height='400'></canvas>";
 

  // Pridáme JavaScript
  html += "<script>"; 
  html += "setInterval(function() {";
  html += "fetch('/data').then(function(response) {";
  html += "return response.json();";
  html += "}).then(function(data) {";
  html += "document.getElementById('temp').innerHTML = data.temp + ' ;';";
  html += "document.getElementById('temp2').innerHTML = data.temp2 + ' ';"; // Teplota z druhého senzora
  html += "document.getElementById('output').innerHTML = data.output + ' ';"; // Výstup PID
  html += "document.getElementById('casmotora').innerHTML = data.casmotora + ' ';"; // Percento otvorenia
  html += "document.getElementById('rssi').innerHTML = data.rssi;"; // Získanie sily signálu
  html += "});";
  html += "}, 5000);"; // Každých 10 sekúnd (10000 ms)
  html += "</script>";

  html += "</body></html>";

  server.send(200, "text/html", html);

}

// Funkcia pre odpoveď na požiadavku /temperature
void handleTemperature() {
  float temp = temperatureC;
  server.send(200, "text/plain", String(temp));
}





void readAndPrintTemperatures() {
  sensors.requestTemperatures();
  temperatureC = sensors.getTempCByIndex(0);
  Serial.print("Teplota vrchná: ");
  Serial.print(temperatureC);
  Serial.println(" ºC");

  sensors2.requestTemperatures();
  temperatureC2 = sensors2.getTempCByIndex(0);
  Serial.print("Teplota spodná: ");
  Serial.print(temperatureC2);
  Serial.println(" ºC");
  Serial.print("optorenie dvierok ");
  Serial.print(totalOpenTime*10);
  Serial.println(" %");

}

#include <math.h> // for fabs()

unsigned int calculatePauseTime(double pidOutput, unsigned int minPause, unsigned int maxPause) {
    double absOutput = fabs(pidOutput);

    if (absOutput >= 10.0) {
        return minPause;
    } else if (absOutput <= 1.0) {
        return maxPause;
    } else {
        // Linearly interpolate between maxPause and minPause
        double factor = (absOutput - 1.0) / (10.0 - 1.0); // range [0..1]
        double pauseTime = maxPause - factor * (maxPause - minPause);
        return (unsigned int)(pauseTime + 0.5); // round to nearest
    }
}


void handleVentilation() {
  // daj temperaturu zo senzora
  sensors.requestTemperatures();
  inputTemperature = sensors.getTempCByIndex(0);
  Serial.print("Aktuálna teplotado regulacie PID: ");
  Serial.print(inputTemperature);
  Serial.println(" ºC");
  // vypocitaj vystup PID cas po 5 sekundach
  if (millis() - pidLastCalculationTime > 5000) { // Počet milisekund mimo posledního vyčitania 
    pidLastCalculationTime = millis();
    ventilationPID.Compute();
    //vypocitaj pauzu pulzu rele
    pulzePauzaTime = calculatePauseTime(outputPID, minPauza, maxPauza);
    Serial.print("Výstup PID: ");
    Serial.println(outputPID);
    Serial.print("Pauza: ");
    Serial.print(pulzePauzaTime);
    Serial.println(" ms");  
  }
 //State machine pre ovladanie ventilácie
  ventilatioStateMachine();

}


void handleData() {
  float temp = temperatureC;  // Teplota z prvého senzora
  int rssi = getSignalStrength();  // Sila signálu

  // Tvorba JSON dokumentu
  JsonDocument jsonDoc;  // Veľkosť bufferu pre JSON
  jsonDoc["temp"] = temp;  // Uloženie teploty do JSON
  jsonDoc["rssi"] = rssi;  // Uloženie sily signálu do JSON
  jsonDoc["temp2"] = temperatureC2; // Teplota z druhého senzora
  jsonDoc["output"] = outputPID; // Výstup PID
  jsonDoc["casmotora"] = casmotora/35; // Percento otvorenia

  // Serializácia JSON do String
  String jsonResponse;
  serializeJson(jsonDoc, jsonResponse);

  // Odoslanie JSON odpovede cez HTTP
  server.send(200, "application/json", jsonResponse);
}


void setup() {
  Serial.begin(115200);
  pinMode(motorPinOpen, OUTPUT);
  pinMode(motorPinClose, OUTPUT);
  int maxAttempts = 10;  // Maximálny počet pokusov o pripojenie pri starte
  int attempts = 0;      // Počítadlo pokusov
  Serial.print("Pripájam sa na WiFi...");
  // Pokúsiť sa pripojiť 10 raz pri inicializácii
  WiFi.begin(ssid, password);
  
  wifiConnecting = true;  // Nastavenie stavu pripojenia na true
  lastWifiAttempt = millis();  // Uloženie času posledného pokusu
  
  Serial.print("IP adresa: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/data", handleData); // Nová cesta: JSON odpoveď
  server.begin();
  Serial.print("Server spustený na adrese: ");
  Serial.println("Server spustený.");
  sensors.begin();
  sensors2.begin();
  Serial.println("DS18B20 senzory inicializované.");

  Serial.println("ESP32 pripravené, PID vetranie spustené.");

  setpoint = temperatureThreshold;
  ventilationPID.SetMode(AUTOMATIC);
  ventilationPID.SetOutputLimits(-10, 10);
}


void loop() {
  

  //temperatures function
  readAndPrintTemperatures();
  //Ventilation handle function
  handleVentilation();
  server.handleClient();

   //>>> Non-blocking WiFi reconnect
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastWifiAttempt >= 60000) { // každých 60 sekúnd
     Serial.println("Skúšam pripojiť WiFi na pozadí...");
     WiFi.disconnect();  // čistý reštart WiFi
      WiFi.begin(ssid, password);
      wifiConnecting = true;
      lastWifiAttempt = currentMillis;
    }
  } else {
    if (wifiConnecting) {
      Serial.println("WiFi pripojené! IP: " + WiFi.localIP().toString());
      wifiConnecting = false;
    }
  }

  
  
}