// *******************************************************************************************************
// DESCRIPTION: MQTT client to send data from Arduino to Raspberry server. Added control and info topic
// AUTOR: Fran Guillén
// BOARD: Arduino UNO
// ANALOG PINS:
//    * Irradiance  -> A0
//    * Temperature -> A1
//    * Voltage     -> A2
//    * Current     -> A3
//    * Frequency   -> A4
// *****************************************************************************************************


#include <Ethernet.h>       // Ethernet communication
#include <SPI.h>            // SPI protocol (Ethernet shield)
#include <PubSubClient.h>   // MQTT library
#include <ArduinoJson.h>    // JSON library


// ---------- CONFIGURING IMPORTANT THINGS... ----------
#define SAMPLE_TIME 100   // Sample time
#define N_ANALOG    5     // Number of analog inputs


// ---------- ETHERNET AND MQTT CONNECTION ----------
#define ETHERNET_MAC        0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED  // MAC
#define ETHERNET_IP         192, 168, 1, 225                    // Shield IP
#define MQTT_SERVER         "192.168.1.250"                     // MQTT server IP
#define MQTT_PORT           1883                                // MQTT port
#define MQTT_USERNAME       "fran"                              // MQTT server username
#define MQTT_PASSWORD       "raspfran"                          // MQTT server password
#define CLIENT_ID           "Arduino_LAB"                       // MQTT client ID


// ---------- MQTT TOPICS ----------
#define PUB_PARAM           "lab/param"
#define PUB_ALARM           "lab/alarm"
#define PUB_INFO            "lab/info"
#define SUB_CONTROL         "lab/control"


// ---------- INPUT/OUTPUT PIN ----------
#define RUN_PIN 8     // Ethernet uses 10, 11, 12, 13. Can't use for general purpose
#define ALARM_PIN 2   // 2 or 3. Arduino UNO interrupts 




// ********************************************************************
//                     GLOBAL VARIABLES
// ********************************************************************
// ---------- ETHERNET AND MQTT CLIENT ----------
byte mac[]  = {ETHERNET_MAC};
IPAddress ip(ETHERNET_IP);
EthernetClient ethClient;
PubSubClient client(ethClient);


// ----- CONTROL VARIABLES -----
int sample_time = SAMPLE_TIME;   
bool RUN_signal = 0;
bool alarm_signal = 0;

// --------- INTERRUPTS VARIABLES ----------
volatile bool interrupt_flag_alarm = false;   // Flag to detect alarm interrupt in loop
volatile bool interrupt_flag_send = false;    // Flag to detect sample_time to send data in loop
volatile int analog_reads[N_ANALOG];          // Array to store analog measurements


// ********************************************************************
//                     FUNCTION PROTOTYPES
// ********************************************************************
void ConnectEthernet();
void ConnectMQTT();
void UpdateInfo(int, bool, bool);
void SendData(int[N_ANALOG]);
void alarm_interrupt();

// ********************************************************************
//                      CALLBACKS
// ********************************************************************
void onReceiveMQTT(char* topic, byte* payload, unsigned int length) {
  // --------- LOCAL VARIABLES ----------
  // MQTT variables
  StaticJsonDocument<100> doc;
  deserializeJson(doc, payload, length);
  JsonObject obj = doc.as<JsonObject>();  

  if (obj.containsKey("RUN")) {
    bool RUN = doc["RUN"];
    RUN_signal = RUN;
    digitalWrite(RUN_PIN, RUN_signal);   // Stop and start frequency drive
  }

  if (obj.containsKey("sample_time")) {
    // ---------- CHANGING SAMPLE TIME ---------
    sample_time = doc["sample_time"];       // Get sample time from JSON
    noInterrupts();                         // Deactivate interrupt
    OCR1A = 250 * sample_time - 1;          // Change register value: [(16*10^6) / (frequency*prescaler)] - 1 (must be <65536)
    interrupts();                           // Enable interrupts
  }

  UpdateInfo(sample_time, RUN_signal, alarm_signal);  // Update information in server
}



// ********************************************************************
//                     BOARD SETUP
// ********************************************************************
void setup() {
  Serial.begin(9600); // Enable serial monitor

  // ---------- Timer1 interrupt configuration ----------
  noInterrupts();                         // Stop interrupts
  TCCR1A = 0;                             // Set entire TCCR1A register to 0
  TCCR1B = 0;                             // Same for TCCR1B
  TCNT1  = 0;                             // Initialize counter value to 0
  OCR1A = (250 * sample_time) - 1;        // 100ms -> ((16*10^6) / (frequency*prescaler)) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12);                 // Turn on CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10);    // Set CS11 and CS10 bits for 64 prescaler
  TIMSK1 |= (1 << OCIE1A);                // Enable timer compare interrupt
  interrupts();                           // Allow interrupts


  // ------ PIN CONFIGURATION ----------
  pinMode(RUN_PIN, OUTPUT);
  digitalWrite(RUN_PIN, RUN_signal);
  pinMode(ALARM_PIN, INPUT_PULLUP); // If connected to drive, not necessary to use pullup

  // ---------- ALARM PIN INTERRUPT (TODO) ---------
  //attachInterrupt(digitalPinToInterrupt(ALARM_PIN), alarm_interrupt, CHANGE);

  // --------- ETHERNET AND MQTT CONFIGURATION ---------
  ConnectEthernet();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  ConnectMQTT();
  client.setCallback(onReceiveMQTT);

  // --------- EXTERNAL VOLTAGE REFERENCE ----------
  analogReference(EXTERNAL);

  // ---------- UPDATE SYSTEM INFO ----------
  UpdateInfo(sample_time, RUN_signal, alarm_signal);
}



// ********************************************************************
//                           LOOP
// ********************************************************************
void loop() {
  // ---------- IF DISCONNECTED, RECONNECT ----------
  if (!client.connected())
  {
    Serial.println("ERROR: Disconnected");
    ConnectMQTT();
  }


  // ---------- SENDING DATA TO MQTT SERVER ----------
  if (interrupt_flag_send) {
    // Deactivate interrupt flag
    interrupt_flag_send = false;

    // Make a copy of variables (volatile only for 8 bits, be sure that value won't change)
    int analog_reads_copy[N_ANALOG];
    noInterrupts();
    for (int i=0; i<N_ANALOG; i++) {
      analog_reads_copy[i] = analog_reads[i]; // Inelegant, but works ...
    }
    interrupts();
    SendData(analog_reads_copy);
  }


  // ---------- SENDING ALARM ----------
  if (interrupt_flag_alarm) {
    interrupt_flag_alarm = false;
    bool alarm = digitalRead(ALARM_PIN);
    UpdateInfo(sample_time, RUN_signal, alarm);
  }

  // ---------- CLIENT LOOP ----------
  client.loop();
}


// ********************************************************************
//                      INTERRUPTS
// ********************************************************************

// ---------- TIMER 1 INTERRUPT ----------
ISR(TIMER1_COMPA_vect) {
  // Interrupt flag to true
  interrupt_flag_send = true;

  // Analog read
  for (int i=0; i<N_ANALOG; i++) {
    analog_reads[i] = analogRead(i);
  }
}

// ---------- ALARM INTERRUPT ----------
void alarm_interrupt() {
  interrupt_flag_alarm = true;
}



// ********************************************************************
//                      LOCAL FUNCTIONS
// ********************************************************************

void ConnectEthernet() {
  Ethernet.begin(mac, ip);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
}

void ConnectMQTT() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = CLIENT_ID;

    // Attempt to connect
    if (client.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("CONNECTED!");
      client.subscribe(SUB_CONTROL, 1); // Subscribe to topic
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" Try again in 5 seconds");
      delay(5000);   // Wait 5 seconds before retrying
    }
  }
}

void SendData(int analog_reads[N_ANALOG]) {
  // Local variables to send data
    const int capacity = JSON_OBJECT_SIZE(N_ANALOG);
    StaticJsonDocument<capacity> doc;
    char buffer[80];

    unsigned long  aux = 0;
    // Irradiance
    aux =  analog_reads[0] * 1164L;
    aux = aux / 1023;
    doc["G"] = aux;

    // Temperature
    doc["T"] = roundf(analog_reads[1] * 14.6627566 - 2000.0) / 100.0;

    // Voltage
    aux = analog_reads[2] * 500L;
    aux = aux / 1023;
    doc["V"] = aux;

    // Current
    doc["I"] = roundf(analog_reads[3] * 331.0 / 1023.0) / 100.0;

    // Frequency
    doc["f"] = roundf(analog_reads[4] * 50000.0 / 1023.0) / 1000.0;

    //Send data to MQTT
    serializeJsonPretty(doc, buffer);
    client.publish(PUB_PARAM, buffer, false);  
}

void UpdateInfo(int sample_time, bool run_signal, bool alarm_signal) {
  // Local variables to send data
    const int capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> doc;
    char buffer[80];

    doc["sample_time"] = int((OCR1A+1)/250);
    doc["run_signal"] = run_signal;
    doc["alarm"] = alarm_signal;

       //Send data to MQTT (RETAIN FLAG TO TRUE)
    serializeJsonPretty(doc, buffer);
    client.publish(PUB_INFO, buffer, true);  
}