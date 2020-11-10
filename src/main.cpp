#include <Ethernet.h>      	// Ethernet communication
#include <SPI.h>			      // SPI protocol (Ethernet shield)
#include <PubSubClient.h>   // MQTT library
#include <ArduinoJson.h>    // JSON library

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
int sample_time = 100;   
bool RUN_signal = 0;

// --------- INTERRUPTS VARIABLES ----------
volatile bool alarm_flag = false;
volatile bool interrupt_flag_send = false;
volatile int analog_0 = 0;
volatile int analog_1 = 0; 
volatile int analog_2 = 0;
volatile int analog_3 = 0;
volatile int analog_4 = 0;



// ********************************************************************
//                     LOCAL FUNCTION PROTOTYPES
// ********************************************************************
void ConnectEthernet();
void ConnectMQTT();
void UpdateInfo(int, bool);


// ********************************************************************
//                      CALLBACKS
// ********************************************************************
void onReceiveMQTT(char* topic, byte* payload, unsigned int length) {
  // --------- LOCAL VARIABLES ----------
  // MQTT variables
  StaticJsonDocument<100> doc;
  deserializeJson(doc, payload, length);
  JsonObject obj = doc.as<JsonObject>();

  //Info variables
  

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

  UpdateInfo(sample_time, RUN_signal);                     // Update information in server
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
  OCR1A = 250 * sample_time - 1;          // 100ms -> ((16*10^6) / (frequency*prescaler)) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12);                 // Turn on CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10);    // Set CS11 and CS10 bits for 64 prescaler
  TIMSK1 |= (1 << OCIE1A);                // Enable timer compare interrupt
  interrupts();                           // Allow interrupts


  // ------ PIN CONFIGURATION ----------
  pinMode(RUN_PIN, OUTPUT);
  digitalWrite(RUN_PIN, RUN_signal);
  pinMode(ALARM_PIN, INPUT_PULLUP); // If connected to drive, not necessary to use pullup

  // ---------- ALARM PIN INTERRUPT (TODO) ---------
  //attachInterrupt(digitalPinToInterrupt(ALARM_PIN), alarm, CHANGE);

  // --------- ETHERNET AND MQTT CONFIGURATION ---------
  ConnectEthernet();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  ConnectMQTT();
  client.setCallback(onReceiveMQTT);

  // --------- EXTERNAL VOLTAGE REFERENCE ----------
  analogReference(EXTERNAL);

  // ---------- UPDATE SYSTEM INFO ----------
  UpdateInfo(sample_time, RUN_signal);
}



// ********************************************************************
//                           LOOP
// ********************************************************************
void loop() {
  // ----- COMPROBAR RED (MQTT LO GESTIONA TODO SOLO) -----
  if (!client.connected())
  {
    Serial.println("ERROR: Disconnected");
    ConnectMQTT();
  }

  client.loop();


  // ---------- SENDING DATA TO MQTT SERVER ----------
  if (interrupt_flag_send) {
    // Deactivate interrupt flag
    interrupt_flag_send = false;

    // Make a copy of variables (volatile only for 8 bits, be sure that value won't change)
    noInterrupts();
    int analog_0_copy = analog_0;
    int analog_1_copy = analog_1;
    int analog_2_copy = analog_2;
    int analog_3_copy = analog_3;
    int analog_4_copy = analog_4;
    interrupts();

    // Local variables to send data
    const int capacity = JSON_OBJECT_SIZE(5);
    StaticJsonDocument<capacity> doc;
    char buffer[80];

    unsigned long  aux = 0;
    // Irradiance
    aux =  analog_0_copy * 1164L;
    aux = aux / 1023;
    doc["G"] = aux;

    // Temperature
    doc["T"] = roundf(analog_1_copy * 14.6627566 - 2000.0) / 100.0;

    // Voltage
    aux = analog_2_copy * 500L;
    aux = aux / 1023;
    doc["V"] = aux;

    // Current
    doc["I"] = roundf(analog_3_copy * 331.0 / 1023.0) / 100.0;

    // Frequency
    doc["f"] = roundf(analog_4_copy * 50000.0 / 1023.0) / 1000.0;

    //Send data to MQTT
    serializeJsonPretty(doc, buffer);
    client.publish(PUB_PARAM, buffer, false);  

  }
}


// ********************************************************************
//                      TIMER1 INTERRUPT
// ********************************************************************

ISR(TIMER1_COMPA_vect) {
  // Flat interrupt to true
  interrupt_flag_send = true;

  // Analog read
  analog_0 = analogRead(A0);
  analog_1 = analogRead(A1);
  analog_2 = analogRead(A2);
  analog_3 = analogRead(A3);
  analog_4 = analogRead(A4);
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

void UpdateInfo(int sample_time, bool run_signal) {
  // Local variables to send data
    const int capacity = JSON_OBJECT_SIZE(2);
    StaticJsonDocument<capacity> doc;
    char buffer[60];

    doc["sample_time"] = (OCR1A+1)/250;
    doc["run_signal"] = run_signal;

       //Send data to MQTT
    serializeJsonPretty(doc, buffer);
    client.publish(PUB_INFO, buffer, true);  
}