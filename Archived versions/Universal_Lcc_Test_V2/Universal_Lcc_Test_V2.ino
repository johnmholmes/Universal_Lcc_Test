
/*This is in beta testing but here to give a chance to have a look
  at the sketch

  2026.01.17 changes: Added second MCP23017, modified CDI

  2025.08.25 changes: Can Transceiver Version Only
  Use update and update16 instead of write to reduce EEPROM wear
  Moved initialization of curpos (may have been overwriting things!)

This is my test version for demonstration CAN Bus use only by John Holmes
  - Pins 19 RX and 18 TX for the transceiver module
  - Pins 14,27,26,32,15,4,16,23  are used for input or output
  - Pins 25,33 servos
  - Pin  21 SDA 
  - Pin  22 SCL

*/
//==============================================================
// AVR 2Servos NIO using ESPcan
//
// Coprright 2024 David P Harris
// derived from work by Alex Shepherd and David Harris
// Updated 2024.11.14
//==============================================================
// - 2 Servo channels, each with 
//     - three settable positions
//     - three set position events 
// - N input/output channels:
//     - type: 0=None, 
//             1=Input, 2=Input inverted, 
//             3=Input with pull-up, 4=Input with pull-up inverted, 
//             5=Input toggle, 6=Toggle with pull-up
//             7=Output, 8=Output inverted.
//     - for Outputs: 
//       - Events are consumed
//       - On-duration: how long the output is set, from 10ms - 2550ms, 0 means forever
//       - Off-period: the period until a repeat pulse, 0 means no repeat
//     - for Inputs:
//       - Events are produced
//       - On-delay: delay before on-event is sent
//       - Off-delay: the period before the off-event is sent
//
//==============================================================
#include <Arduino.h>
#include "Config.h"   // Contains configuration, see "Config.h"

#include "mdebugging.h"           // debugging
//#include "processCAN.h"           // Auto-select CAN library
//#include "processor.h"            // auto-selects the processor type, EEPROM lib etc.
#include "OpenLCBHeader.h"        // System house-keeping.

// CDI (Configuration Description Information) in xml, must match MemStruct
// See: http://openlcb.com/wp-content/uploads/2016/02/S-9.7.4.1-ConfigurationDescriptionInformation-2016-02-06.pdf
extern "C" {
    #define N(x) xN(x)     // allow the insertion of the value (x) ..
    #define xN(x) #x       // .. into the CDI string.
const char configDefInfo[] PROGMEM =
// ===== Enter User definitions below =====
  CDIheader R"(
    <name>Application Configuration</name>
    <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
    <group>
      <name>Native Turnout Servo Configuration</name>
      <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
      <int size='1'>
        <name>Speed 5-50 (delay between steps)</name>
        <min>5</min><max>50</max>
        <hints><slider tickSpacing='15' immediate='yes' showValue='yes'> </slider></hints>
      </int>
      <int size='1'><name>Save servo positions every x*x= seconds</name></int>
    </group>
    <group replication=')" N(NUM_SERVOS) R"('>
      <name>Native Servos</name>
      <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
      <repname>Servo </repname>
      <string size='8'><name>Description</name></string>
      <group replication=')" N(NUM_POS) R"('>
      <name>  Closed     Midpoint   Thrown</name>
        <repname>Position</repname>
        <eventid><name>EventID</name></eventid>
        <int size='1'>
          <name>Servo Position in Degrees</name>
          <min>0</min><max>180</max>
          <hints><slider tickSpacing='45' immediate='yes' showValue='yes'> </slider></hints>
        </int>
      </group>
    </group>
    <group replication=')" N(NUM_NATIVE_IO) R"('>
      <name>Native IO</name>
      <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
      <repname>1</repname>
      <string size='8'><name>Description</name></string>
      <int size='1'>
        <name>Channel type</name>
        <map>
          <relation><property>0</property><value>None</value></relation> 
          <relation><property>1</property><value>Input</value></relation> 
          <relation><property>2</property><value>Input Inverted</value></relation> 
          <relation><property>3</property><value>Input with pull-up</value></relation>
          <relation><property>4</property><value>Input with pull-up Inverted</value></relation>
          <relation><property>5</property><value>Toggle</value></relation>
          <relation><property>6</property><value>Toggle with pull-up</value></relation>
          <relation><property>7</property><value>Output PA</value></relation>
          <relation><property>8</property><value>Output PA Inverted</value></relation>
          <relation><property>9</property><value>Output PB</value></relation>
          <relation><property>10</property><value>Output PB Inverted</value></relation>
        </map>
      </int>
      <int size='1'>
        <name>On-Duration/On-delay 1-255 = 100ms-25.5s, 0=steady-state</name>
        <hints><slider tickSpacing='50' immediate='yes' showValue='yes'> </slider></hints>
      </int>
      <int size='1'>
        <name>Off-Period/Off-delay 1-255 = 100ms-25.5s, 0=No repeat</name>
        <hints><slider tickSpacing='50' immediate='yes' showValue='yes'> </slider></hints>
      </int>
      <eventid><name>On-Event</name></eventid>
      <eventid><name>Off-Event</name></eventid>
    </group>
    <group replication=')" N(NUM_MCP_PORTS) R"('>
      <name>MCP23017 IO</name>
      <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
      <repname>Port 1 A</repname>
      <repname>Port 1 B</repname>
      <repname>Port 2 A</repname>
      <repname>Port 2 B</repname>
      <group replication=')" N(NUM_MCP_IO_PER_PORT) R"('>
        <name>Port IO</name>
        <repname>0</repname>
        <repname>1</repname>
        <repname>2</repname>
        <repname>3</repname>
        <repname>4</repname>
        <repname>5</repname>
        <repname>6</repname>
        <repname>7(OUT ONLY)</repname>
        <string size='8'><name>Description</name></string>
        <int size='1'>
          <name>Channel type</name>
          <map>
            <relation><property>0</property><value>None</value></relation> 
            <relation><property>1</property><value>Input</value></relation> 
            <relation><property>2</property><value>Input Inverted</value></relation> 
            <relation><property>3</property><value>Input with pull-up</value></relation>
            <relation><property>4</property><value>Input with pull-up Inverted</value></relation>
            <relation><property>5</property><value>Toggle</value></relation>
            <relation><property>6</property><value>Toggle with pull-up</value></relation>
            <relation><property>7</property><value>Output PA</value></relation>
            <relation><property>8</property><value>Output PA Inverted</value></relation>
            <relation><property>9</property><value>Output PB</value></relation>
            <relation><property>10</property><value>Output PB Inverted</value></relation>
          </map>
        </int>
        <int size='1'>
          <name>On-Duration/On-delay 1-255 = 100ms-25.5s, 0=steady-state</name>
          <hints><slider tickSpacing='50' immediate='yes' showValue='yes'> </slider></hints>
        </int>
        <int size='1'>
          <name>Off-Period/Off-delay 1-255 = 100ms-25.5s, 0=No repeat</name>
          <hints><slider tickSpacing='50' immediate='yes' showValue='yes'> </slider></hints>
        </int>
        <eventid><name>On-Event</name></eventid>
        <eventid><name>Off-Event</name></eventid>
      </group>
    </group>
    <group replication=')" N(NUM_PCA_PORTS) R"('>
      <name>PCA9685 Servos</name>
      <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
      <repname>PCA1 Channels 0-7</repname>
      <repname>PCA1 Channels 8-15</repname>
      <repname>PCA2 Channels 0-7</repname>
      <repname>PCA2 Channels 8-15</repname>
      <group replication=')" N(8) R"('>
        <name>Servos</name>
        <repname>0/8</repname>
        <repname>1/9</repname>
        <repname>2/10</repname>
        <repname>3/11</repname>
        <repname>4/12</repname>
        <repname>5/13</repname>
        <repname>6/14</repname>
        <repname>7/15</repname>
        <string size='8'><name>Description</name></string>
        <int size='1'><name>Speed of movement 1-50</name>
          <min>0</min><max>50</max>
          <hints><slider tickSpacing='20' immediate='yes' showValue='yes'> </slider></hints>
        </int>
        <int size='1'><name>Position 1 Angle 0-180></name>
          <min>0</min><max>180</max>
          <hints><slider tickSpacing='45' immediate='yes' showValue='yes'> </slider></hints>
        </int>
        <eventid><name>When consumed, move to this angle</name></eventid>
        <int size='1'><name>Position 2 Angle 0-180></name>
          <min>0</min><max>180</max>
          <hints><slider tickSpacing='45' immediate='yes' showValue='yes'> </slider></hints>
        </int>
        <eventid><name>When consumed, move to this angle</name></eventid>
        <group>
          <name>MidPoint Events</name>
          <eventid><name>Sends this event When the servo passes the midpoint moving towards position 2</name></eventid>
          <eventid><name>Sends this event When the servo passes the midpoint moving towards position 1</name></eventid>
        </group>
      </group>
    </group>
    )" CDIfooter;
// ===== Enter User definitions above =====
} // end extern

// ===== MemStruct =====
//   Memory structure of EEPROM, must match CDI above
    typedef struct {
          EVENT_SPACE_HEADER eventSpaceHeader; // MUST BE AT THE TOP OF STRUCT - DO NOT REMOVE!!!
          char nodeName[20];  // optional node-name, used by ACDI
          char nodeDesc[24];  // optional node-description, used by ACDI
      // ===== Enter User definitions below =====
          uint8_t servodelay;
          uint8_t saveperiod; // period in seconds to save the servo positions
          struct {
            char desc[8];        // description of this Servo Turnout Driver
            struct {
              EventID eid;       // consumer eventID
              uint8_t angle;     // position
            } pos[NUM_POS];
          } servos[NUM_SERVOS];
          struct {
            char desc[8];
            uint8_t type;
            uint8_t duration;    // 100ms-25.5s, 0=solid
            uint8_t period;      // 100ms-25.5s, 0=no repeat
            EventID onEid;
            EventID offEid;
          } io[NUM_IO];
          struct {
            char desc[8];
            uint8_t speed;
            uint8_t angle1;
            EventID eid1;
            uint8_t angle2;
            EventID eid2;
            EventID eidup;
            EventID eiddown;
          } pcaservo[NUM_PCA_SERVO];
      // ===== Enter User definitions above =====
      uint8_t curpos[NUM_SERVOS];  // save current positions of servos
      // items below will be included in the EEPROM, but are not part of the CDI
    } MemStruct;                 // type definition

uint8_t curpos[NUM_SERVOS]; 


extern "C" {
    // ===== eventid Table =====
    //#define REG_SERVO_OUTPUT(s) CEID(servos[s].pos[0].eid), CEID(servos[s].pos[1].eid), CEID(servos[s].pos[2].eid)
    //#define REG_IO(i) PCEID(io[i].onEid), PCEID(io[i].offEid)
    //#define REG_NAT(g) REG_IO(g+0), REG_IO(g+1), REG_IO(g+2), REG_IO(g+3), REG_IO(g+4), REG_IO(g+5), REG_IO(g+6), REG_IO(g+7) 
    //#define REG_MCP(g) REG_IO(g+0), REG_IO(g+1), REG_IO(g+2), REG_IO(g+3), REG_IO(g+4), REG_IO(g+5), REG_IO(g+6), REG_IO(g+7) 
    //#define REG_SVO(s) CEID(servo[s].eid1), CEID(servo[s].eid2), PEID(servo[s].eidup), PEID(servo[s].eiddown)  
    //#define REG_PCA(g) REG_SVO(g+0), REG_SVO(g+1), REG_SVO(g+2), REG_SVO(g+3), REG_SVO(g+4), REG_SVO(g+5), REG_SVO(g+6), REG_SVO(g+7)

    //  Array of the offsets to every eventID in MemStruct/EEPROM/mem, and P/C flags
    const EIDTab eidtab[NUM_EVENT] PROGMEM = {
        //REG_SERVO_OUTPUT(0), REG_SERVO_OUTPUT(1),
        //REG_NAT(0), REG_NAT(8), REG_NAT(16), 
        SERVOEID(NUM_SERVOS),  // servos
        IOEID(NUM_NATIVE_IO),  // native io
        MCPEID(NUM_MCP_PORTS),
        PCAEID(NUM_PCA_PORTS)
    };
    
    // SNIP Short node description for use by the Simple Node Information Protocol
    // See: http://openlcb.com/wp-content/uploads/2016/02/S-9.7.4.3-SimpleNodeInformation-2016-02-06.pdf
    extern const char SNII_const_data[] PROGMEM = "\001" MANU "\000" MODEL "\000" HWVERSION "\000" OlcbCommonVersion ; // last zero in double-quote
} // end extern "C"

// PIP Protocol Identification Protocol uses a bit-field to indicate which protocols this node supports
// See 3.3.6 and 3.3.7 in http://openlcb.com/wp-content/uploads/2016/02/S-9.7.3-MessageNetwork-2016-02-06.pdf
uint8_t protocolIdentValue[6] = {   //0xD7,0x58,0x00,0,0,0};
        pSimple | pDatagram | pMemConfig | pPCEvents | !pIdent    | pTeach     | !pStream   | !pReservation, // 1st byte
        pACDI   | pSNIP     | pCDI       | !pRemote  | !pDisplay  | !pTraction | !pFunction | !pDCC        , // 2nd byte
        0, 0, 0, 0                                                                                           // remaining 4 bytes
    };

#define OLCB_NO_BLUE_GOLD  // blue/gold not used in this sketch
#ifndef OLCB_NO_BLUE_GOLD
    #define BLUE 40  // built-in blue LED
    #define GOLD 39  // built-in green LED
    ButtonLed blue(BLUE, LOW);
    ButtonLed gold(GOLD, LOW);
    
    uint32_t patterns[8] = { 0x00010001L, 0xFFFEFFFEL }; // two per channel, one per event
    ButtonLed pA(13, LOW);
    ButtonLed pB(14, LOW);
    ButtonLed pC(15, LOW);
    ButtonLed pD(16, LOW);
    ButtonLed* buttons[8] = { &pA,&pA,&pB,&pB,&pC,&pC,&pD,&pD };
#endif // OLCB_NO_BLUE_GOLD

//#include <Servo.h>    //// NANO, Minima etc
//#include <ESP32Servo.h> //// ESP32
Servo servo[2];
uint8_t servodelay;
//uint8_t servopin[NUM_SERVOS] = {25,33};  // CHOOSE PINS FOR SERVOS
uint8_t servopin[NUM_SERVOS] = { SERVOPINS };  // CHOOSE PINS FOR SERVOS
uint8_t servoActual[NUM_SERVOS];
uint8_t servoTarget[NUM_SERVOS];

uint8_t iopin[NUM_IO] = { IOPINS }; //// ESP32 

enum Type { tNONE=0, tIN, tINI, tINP, tINPI, tTOG, tTOGI, tPA, tPAI, tPB, tPBI };
bool iostate[NUM_IO] = {0};  // state of the iopin
bool logstate[NUM_IO] = {0}; // logic state for toggle
unsigned long next[NUM_IO] = {0};

uint8_t pcastate[NUM_PCA_SERVO];
//uint8_t target[NUM_PCA_SERVO];
//uint8_t actual[NUM_PCA_SERVO];

// This is called to initialize the EEPROM to Factory Reset
void userInitAll()
{ 
  NODECONFIG.put(EEADDR(nodeName), ESTRING(BOARD));
  NODECONFIG.put(EEADDR(nodeDesc), ESTRING("2Servos2MCP2PCA"));
  NODECONFIG.update(EEADDR(servodelay), 50);
  NODECONFIG.update(EEADDR(saveperiod), 50);
  for(uint8_t i = 0; i < NUM_SERVOS; i++) {
    NODECONFIG.put(EEADDR(servos[i].desc), ESTRING(""));
    for(int p=0; p<NUM_POS; p++) {
      NODECONFIG.update(EEADDR(servos[i].pos[p].angle), 90);
      NODECONFIG.update(EEADDR(curpos[i]), 0);
    }
  }
  dP("\n NUM_IO"); dP(NUM_IO);
  for(uint8_t i = 0; i < NUM_IO; i++) {
    NODECONFIG.put(EEADDR(io[i].desc), ESTRING(""));
    NODECONFIG.update(EEADDR(io[i].type), 0);
    NODECONFIG.update(EEADDR(io[i].duration), 0);
    NODECONFIG.update(EEADDR(io[i].period), 0);
  }  
  for(uint8_t i=0; i<NUM_PCA_SERVO; i++) {
    NODECONFIG.put(EEADDR(pcaservo[i].desc), ESTRING(""));
    NODECONFIG.update(EEADDR(pcaservo[i].speed), 20);
    NODECONFIG.update(EEADDR(pcaservo[i].angle1), 85);
    NODECONFIG.update(EEADDR(pcaservo[i].angle2), 95);
  }
}

// determine the state of each eventid
enum evStates { VALID=4, INVALID=5, UNKNOWN=7 };
uint8_t userState(uint16_t index) {
  //dP("\n userState "); dP((uint16_t) index);
    if(index < NUM_SERVO_EVENT) {
      int ch = index/3;
      int pos = index%3;
      //dP( (uint8_t) curpos[ch]==pos);
      if( curpos[ch]==pos ) return VALID;
      else return INVALID;
    }
    index -= NUM_SERVO_EVENT;
    if( index<NUM_IO_EVENT ) {
      int ch = (index - NUM_SERVOS*NUM_POS)/2;
      if( NODECONFIG.read(EEADDR(io[ch].type))==0) return UNKNOWN;
      int evst = index % 2;
      //dP((uint8_t) iostate[ch]==evst);
      if( iostate[ch]==evst ) return VALID;
      return INVALID;
    }
    index -= NUM_IO_EVENT;
    if( index<NUM_PCA_SERVO_EVENT ) {
      uint8_t s = index/2;
      if( pcastate[s]==2 ) return UNKNOWN;
      if( index%1 ) {
        if( pcastate[s]==1 ) return VALID;
        else return INVALID;
      }
    }
    return UNKNOWN;
}
    

  #ifdef DEBUG
    #define PV(x) { dP(" " #x "="); dP(x); }
  #else
    #define PV(x) 
  #endif

// MCP23017 INIT
void mcpinit() {
  for (int i = 0; i < NUM_MCP; i++) {
    mcp[i] = new MCP23017(MCP_ADDRESSES[i]);
    mcp[i]->init(); 
    Serial.print("\nMCP23017 at ");
    Serial.print(MCP_ADDRESSES[i], HEX);
    Serial.println(" is ready.");
  }
}

// PCA9685 ROUTINES
uint8_t target[NUM_PCA_SERVO];
uint16_t pcabase = NUM_SERVO_EVENT + NUM_IO_EVENT;

void getAndAttach16ServosToPCA9685Expander(uint8_t base, uint8_t aPCA9685I2CAddress) {
    ServoEasing *tServoEasingObjectPtr;
    Serial.print(F("Get ServoEasing objects and attach servos to PCA9685 expander at address=0x"));
    Serial.println(aPCA9685I2CAddress, HEX);
    for (uint_fast8_t i = 0; i < PCA9685_MAX_CHANNELS; ++i) {
        tServoEasingObjectPtr = new ServoEasing(aPCA9685I2CAddress); // Get the ServoEasing object
        uint8_t angle;
        #ifdef PCA_INIT_TO_90
          angle = 90;
        #else
          uint8_t ch = base + i; 
          uint8_t a1 = NODECONFIG.read( EEADDR(pcaservo[ch].angle1) );
          uint8_t a2 = NODECONFIG.read( EEADDR(pcaservo[ch].angle2) );
          angle = (a1+a2)/2;
        #endif
        if (tServoEasingObjectPtr->attach(i, angle) == INVALID_SERVO) {
            Serial.print(F("Address=0x"));
            Serial.print(aPCA9685I2CAddress, HEX);
            Serial.print(F(" i="));
            Serial.print(i);
            Serial.println(F(" Error attaching servo - maybe MAX_EASING_SERVOS=" STR(MAX_EASING_SERVOS) " is to small to hold all servos"));
        }
    }
}
bool pcaavail[NUM_PCA_SERVO];
void pcaInit() {
  dP("\n pcaInit");
  for(uint8_t i=0; i<NUM_PCA; i++) {
    pcaavail[i]=true;
    ///if (checkI2CConnection(PCA_ADDRESS1, &Serial)) {
    if (checkI2CConnection(PCA_ADDRESSES[i], &Serial)) {
      Serial.println(F("PCA9685 expander not connected -> disabled."));
      pcaavail[i]=false;
    } else {
      //if(i==0) getAndAttach16ServosToPCA9685Expander(0, PCA_ADDRESS1);
      //if(i==1) getAndAttach16ServosToPCA9685Expander(16, PCA_ADDRESS2);
      for(int i=0; i<NUM_PCA; i++) getAndAttach16ServosToPCA9685Expander(i*16, PCA_ADDRESSES[i]);
    }
  }

  setSpeedForAllServos(20);  // should be one speed for all or individual speeds?
  setEasingTypeForAllServos(EASE_CUBIC_IN_OUT); //
  for(int ch=0; ch<NUM_PCA_SERVO; ch++) {
    if( pcaavail[ch/16] ) {
      ServoEasing::ServoEasingArray[ch]->setTargetPositionReachedHandler(endOfMove);
    }
    pcastate[ch]=2;
  }
  // test
 #if 0
  pcawrite(0, 75); delay(500);
  pcawrite(0, 90); delay(500);
  pcawrite(0, 115); delay(500);
  pcawrite(0, 120); delay(500);
  pcawrite(0, 90); delay(500);
 #endif
}
void pcawrite(int ch, int angle) {
  //ServoEasing::ServoEasingArray[i]->easeTo(angle);
  dP("\n pcawrite");
  if( pcaavail[ch/16] ) {
    dP(" startEaseTo "); dP(angle); dP("\n");
    if(doreattach) {
      dP("\n Channel "); dP(ch); dP(" reattached");
      ServoEasing::ServoEasingArray[ch]->reattach();
    }
    dP("\n pcawrite ch="); dP(ch); dP(" angle="); dP(angle);
    ServoEasing::ServoEasingArray[ch]->startEaseTo(angle);
  }
}
void processPCA() {
  static long last = 0;
  if( (millis()-last) < 100 ) return;
  last = millis();
  //uint8_t s = 0;
  for(int ch=0; ch<NUM_PCA_SERVO; ch++) {
    if( !ServoEasing::ServoEasingArray[ch]->isMoving() ) {
      //dP("\n pca "); dP(i); dP(" is stopped.");
      if( ServoEasing::ServoEasingArray[ch]->getCurrentAngle() == target[ch] ) {
        //dP(" at target, so shutdown the PWM.");
        ServoEasing::ServoEasingArray[ch]->setPWM(0, 4096);
        continue;
      }
      uint8_t t1 = NODECONFIG.read( EEADDR(pcaservo[ch].angle1) );
      uint8_t t2 = NODECONFIG.read( EEADDR(pcaservo[ch].angle2) );
      uint8_t mdpt = (t1+t2)/2;
      if( ServoEasing::ServoEasingArray[ch]->getCurrentAngle() == mdpt ) {
        if( ServoEasing::ServoEasingArray[ch]->getCurrentAngle() < target[ch] ) {
          dP("\n is going up");
          OpenLcb.produce(pcabase+ch*4+3);
        } else {
          dP("\n is going down");
          OpenLcb.produce(pcabase+ch*4+3);
        }
        pcawrite(ch, target[ch]);
      }
      //dP("\n continue to the target: pcaservo "); dP(ch/16); dP(":"); dP(ch%16); dP(" to "); dP(target[ch]);
    }
  }
}

// this routine is called when a servo reaches its endpoint
void endOfMove(ServoEasing* servo) {
    int servoIndex = -1;
    // Iterate through the global array to find a pointer match
    for (int ch = 0; ch < NUM_PCA_SERVO; ch++) {
        if (servo == ServoEasing::ServoEasingArray[ch]) {
            servoIndex = ch;  // the matching servo's index
            break;
        }
    }
    dP("\n Found index = "); dP(servoIndex);
    if (servoIndex != -1) { // if found
      uint8_t a1 = NODECONFIG.read( EEADDR(pcaservo[servoIndex].angle1) );
      uint8_t a2 = NODECONFIG.read( EEADDR(pcaservo[servoIndex].angle2) );
      uint8_t mp = (a1+a2)/2;  // calulate the midpoint
      if( servo->getCurrentAngle() == mp ) {
        if( mp<target[servoIndex] ) OpenLcb.produce(pcabase+servoIndex*4+2); // if going up, send the up-event
        if( mp>target[servoIndex] ) OpenLcb.produce(pcabase+servoIndex*4+3); // if going down, send the down-event
        ServoEasing::ServoEasingArray[servoIndex]->setEasingType(EASE_CUBIC_OUT);
        pcawrite(servoIndex, target[servoIndex]);  // finish the move
      } else { // we assume its at the endpoint
        if( doreattach) ServoEasing::ServoEasingArray[servoIndex]->detach();
      }
    } 
}

// ===== Process Consumer-eventIDs =====
void pceCallback(uint16_t index) {
// Invoked when an event is consumed; drive pins as needed
// from index of all events.
// Sample code uses inverse of low bit of pattern to drive pin all on or all off.
// The pattern is mostly one way, blinking the other, hence inverse.
//
  dP("\npceCallback, index="); dP((uint16_t)index);
  
  // Native Servos
  if(index<NUM_SERVO_EVENT) {
    dP("\nnative servo");
    uint8_t outputIndex = index / 3;
    uint8_t outputState = index % 3;
    dP("\n native servo "); dP(outputIndex); dP(" to "); dP(outputState);
    curpos[outputIndex] = outputState;
    servo[outputIndex].attach(servopin[outputIndex]);
    servoTarget[outputIndex] = NODECONFIG.read( EEADDR(servos[outputIndex].pos[outputState].angle) );
    return;
  }
  // All IO, both native and MCP23017
  index -= NUM_SERVO_EVENT; // adjust index
  dP("\nless servo index="); dP((uint16_t)index);
  if( index< (NUM_NATIVE_IO_EVENT+NUM_MCP_EVENT) ) {
    dP("\nnative and mcp io");
    dP("\n NUM_NATIVE_IO_EVENT+NUM_MCP_EVENT="); dP(index);
    int ch = index/2;
    uint8_t type = NODECONFIG.read(EEADDR(io[ch].type));
    if(ch<NUM_NATIVE_IO) { dP("\n Native io "); dP(ch); PV(type); }
    else  { dP("\n MCP io "); dP(ch-NUM_NATIVE_IO); PV(type); }
    if(type>=7) {
      // 7=PA 8=PAI 9=PB 10=PBI
      bool inv = !(type&1);       // inverted
      bool pha = type<9;          // phaseA
      if( index%2 ) {
        dP("\noff"); PV(ch); PV(type); PV(inv);
        if(ch<NUM_NATIVE_IO) digitalWrite( iopin[ch], inv);
        else {
          uint8_t c = ch-NUM_NATIVE_IO;
          //if(c<16) mcp1.digitalWrite( c, inv);
          //else mcp2.digitalWrite( c-16, inv);
          mcp[c/16]->digitalWrite( c%16, inv);
        }
        next[ch] = 0;
      } else {
        dP("\nON"); PV(ch); PV(pha); PV(inv); 
        if(ch<NUM_NATIVE_IO) digitalWrite( iopin[ch], pha ^ inv);
        else {
          uint8_t c = ch-NUM_NATIVE_IO;
          //if(c<16) mcp1.digitalWrite( c, pha ^ inv);
          //else  mcp2.digitalWrite( c-16, pha ^ inv);
          mcp[c/16]->digitalWrite( c%16, pha ^ inv);
        }
        iostate[ch] = 1;
        uint8_t durn;
        durn = NODECONFIG.read(EEADDR(io[ch].duration));
        PV(durn);
        if(durn) next[ch] = millis() + 100*durn; // note duration==0 means forever
        else next[ch]=0;
      }
      PV(millis()); PV(next[ch]);
    }
    return;
  }
  // PCA9685 Servos
  index -= (NUM_NATIVE_IO_EVENT+NUM_MCP_EVENT);    // adjust index   
  dP("\nless IO+mcp servos index="); dP((uint16_t)index);      
  if( index<NUM_PCA_SERVO_EVENT) {
    dP("\n NUM_PCA_SERVO_EVENT="); dP(index);
    uint8_t i = index/(16*4);       // which PCA board  0->1
    uint8_t ch = index/4;           // which channel    0->32
    uint8_t e = index%4;            // which consumer event 0->3
    uint8_t speed = NODECONFIG.read( EEADDR(pcaservo[ch].speed) );
    if(speed==0) speed=1;
    if(speed>50) speed=50;
    ServoEasing::ServoEasingArray[ch]->setSpeed(speed);
    uint8_t t1 = NODECONFIG.read( EEADDR(pcaservo[ch].angle1) ); // get the angle-positions for this channel
    uint8_t t2 = NODECONFIG.read( EEADDR(pcaservo[ch].angle2) );
    dP("\n pca board="); dP(i); dP(" channel="); dP(ch); dP(" eid#="); dP(e);
    uint8_t mdpt = (t1+t2)/2;    // call the midpoint
    dP(" posn1="); dP(t1); dP(" posn2="); dP(t2); dP(" current position="); dP(ServoEasing::ServoEasingArray[ch]->getCurrentAngle());
    if(e==0 && ServoEasing::ServoEasingArray[ch]->getCurrentAngle()!=t1) {
      target[ch] = t1;  // eventual target isangle 1, after passing thorugh the midpoint
      dP("\n move pcaservo "); dP(i); dP(":"); dP(ch); dP(" to midpoint at "); dP(mdpt); dP(" then to "); dP(t1); 
      ServoEasing::ServoEasingArray[ch]->setEasingType(EASE_CUBIC_IN);
      pcawrite(ch, mdpt); // go to the midpoint, endOfMove() will be called when it reaches the midpoint
    }
    if(e==1 && ServoEasing::ServoEasingArray[ch]->getCurrentAngle()!=t2) {
      target[ch] = t2;  // eventual target is angle 2, after passing thorugh the midpoint
      dP("\n move pcaservo "); dP(i); dP(":"); dP(ch); dP(" to midpoint at "); dP(mdpt); dP(" then to "); dP(t2); 
      ServoEasing::ServoEasingArray[ch]->setEasingType(EASE_CUBIC_IN);
      pcawrite(ch, mdpt); // go to the midpoint, endOfMove() will be called when it reaches the midpoint
    }
  }
}

// === Process servos ===
// This is called from loop to service the servos
bool posdirty = false;
void natServoProcess() {
  static long last = 0;
  if( (millis()-last) < servodelay ) return;
  last = millis();
  static long lastmove = 0;
  for(int i=0; i<NUM_SERVOS; i++) {
    if(servoTarget[i] == servoActual[i] ) continue;
    if(servoTarget[i] > servoActual[i] ) servoActual[i]++;
    else if(servoTarget[i] < servoActual[i] ) servoActual[i]--;
    if(!servo[i].attached()) { 
      servo[i].attach(servopin[i]);
      delay(100);
    }
    //servo[i].attach(servopin[i]);
    servo[i].write(servoActual[i]);
    //P("\n servomove"); PV(i); PV(servoTarget[i]); PV(servoActual[i]);
    lastmove = millis();
    posdirty =true;
  }

  if( lastmove && (millis()-lastmove)>4000) {
    for(int i=0; i<NUM_SERVOS; i++) servo[i].detach();
    lastmove = 0;
    //dP("\n detach()");
    //printMem();
  }

  static long lastsave = 0;
  uint32_t saveperiod = getSavePeriod();
  if(saveperiod && posdirty && (millis()-lastsave) > saveperiod ) {
    lastsave = millis();
    posdirty = false;
    //for(int i=0; i<NUM_SERVOS; i++) 
    //  NODECONFIG.update( EEADDR(curpos[i]), curpos[i]);
    for(int i=0; i<NUM_SERVOS; i++) NODECONFIG.update( EEADDR(curpos[i]), curpos[i]);
    EEPROMcommit;
    //dP("\n save curpos\n curpos[0]="); dP(NODECONFIG.read( EEADDR(curpos[0])));
    //dP("\n curpos[1]="); dP(NODECONFIG.read( EEADDR(curpos[1])));
  }
}

// ==== Process Inputs ====
void produceFromInputs() {
    // called from loop(), this looks at changes in input pins and
    // and decides which events to fire
    // with pce.produce(i);
    const uint8_t base = NUM_SERVOS*NUM_POS;
    static uint8_t c = 0;
    static unsigned long last = 0;
    if((millis()-last)<(50/NUM_IO)) return;
    last = millis();
    uint8_t type = NODECONFIG.read(EEADDR(io[c].type));
    uint8_t d;
    if(type==5 || type==6) {
      //dP("\n"); PV(c); PV(type);
      bool s;
      if(c<NUM_NATIVE_IO) s = digitalRead(iopin[c]);
      else {
        uint8_t ch = c-NUM_NATIVE_IO;
        //if(ch<16) s = mcp1.digitalRead(ch);
        //else s = mcp2.digitalRead(ch-16);
        s = mcp[ch/16]->digitalRead(ch%16);
      }
      //PV(s); PV(iostate[c]);
      if(s != iostate[c]) {
        iostate[c] = s;
        if(!s) {
          logstate[c] ^= 1;
          if(logstate[c]) d = NODECONFIG.read(EEADDR(io[c].duration));
          else            d = NODECONFIG.read(EEADDR(io[c].period));
          //dP("\ninput "); PV(c); PV(type); PV(s); PV(logstate[c]); PV(d);
          if(d==0) OpenLcb.produce( base+c*2 + logstate[c] ); // if no delay send the event
          else next[c] = millis() + (uint16_t)d*100;          // else register the delay
          //PV(millis()); PV(next[c]);
        }
      }
    }
    if(type>0 && type<5) {
      //dP("\n"); PV(c); PV(type);
      bool s;
      if(c<NUM_NATIVE_IO) s = digitalRead(iopin[c]);
      else {
        uint8_t ch = c-NUM_NATIVE_IO;
        //if(ch<16) s = mcp1.digitalRead(ch);
        //else mcp2.digitalRead(ch-16);
        s = mcp[ch/16]->digitalRead(ch%16);
      }
      if(s != iostate[c]) {
        iostate[c] = s;
        d = NODECONFIG.read(EEADDR(io[c].duration)); 
        //dP("\ninput "); PV(type); PV(s); PV(d);
        if(d==0) OpenLcb.produce( base+c*2 + (!s^(type&1)) ); // if no delay send event immediately
        else {
          next[c] = millis() + (uint16_t)d*100;                   // else register the delay
          //PV(millis()); PV(next[c]);
        }
      }
    }
    if(++c>=NUM_IO) c = 0;
}

// Process pending producer events
// Called from loop to service any pending event waiting on a delay
void processProducer() {
  const uint8_t base = NUM_SERVOS*NUM_POS;
  static unsigned long last = 0;
  unsigned long now = millis();
  if( (now-last) < 50 ) return;
  for(int c=0; c<NUM_IO; c++) {
    if(next[c]==0) continue;
    if(now<next[c]) continue; 
    uint8_t type = NODECONFIG.read(EEADDR(io[c].type));
    if(type>6) return; // do not process outputs
    uint8_t s = iostate[c];
    //dP("\nproducer"); PV(type); PV(s); PV((!s^(type&1)));
    if(type<5)  OpenLcb.produce( base+c*2 + (!s^(type&1)) ); // reg inputs
    else OpenLcb.produce( base+c*2 + logstate[c] );          // toggle inputs
    next[c] = 0;
  }
}

void userSoftReset() {}
void userHardReset() {}

#include "OpenLCBMid.h"    // Essential, do not move or delete

#if 0
#define P(...) Serial.print( __VA_ARGS__)
#define PV(x) { P(" "); P(#x  "="); P(x); }
#define PVL(x) { P("\n"); P(#x  "="); P(x); }
void printMem() {
  PVL(NODECONFIG.read(EEADDR(servodelay)));
  for(int s=0;s<NUM_SERVOS;s++) {
    P("\nServo "); P(s);
    for(int p=0;p<NUM_POS;p++) {
      PV(NODECONFIG.read(EEADDR(servos[s].pos[p])));
    }
  }
  /*
  for(int i=0;s<NUM_IO;i++) {
    P("\nIO "); P(i);
    PV(NODECONFIG.read(EEADDR(io[i].type)));
    PV(NODECONFIG.read(EEADDR(io[i].duration)));
    PV(NODECONFIG.read(EEADDR(io[i].period)));
  }
  */
}
#endif

void pcaServoSet(uint32_t address) {
  for(int s=0; s<NUM_PCA_SERVO; s++) {
    if(address==EEADDR(pcaservo[s].angle1)) { 
      if(doreattach) ServoEasing::ServoEasingArray[s]->reattach();
      ServoEasing::ServoEasingArray[s]->setEasingType(EASE_CUBIC_OUT);
      pcawrite(s, NODECONFIG.read( EEADDR(pcaservo[s].angle1) ) );  
      if(doreattach) ServoEasing::ServoEasingArray[s]->detach();
      return;
    }
    if(address==EEADDR(pcaservo[s].angle2)) { 
      if(doreattach) ServoEasing::ServoEasingArray[s]->reattach();
      ServoEasing::ServoEasingArray[s]->setEasingType(EASE_CUBIC_OUT);
      pcawrite(s, NODECONFIG.read( EEADDR(pcaservo[s].angle2) ) );  
      if(doreattach) ServoEasing::ServoEasingArray[s]->detach();
      return;
    }
  }
}
// Callback from a Configuration write
// Use this to detect changes in the node's configuration
// This may be useful to take immediate action on a change.
void userConfigWritten(uint32_t address, uint16_t length, uint16_t func)
{
  dPS("\nuserConfigWritten: Addr: ", (uint32_t)address);
  dPS(" Len: ", (uint16_t)length);
  dPS(" Func: ", (uint8_t)func);
  setupIOPins();
  servodelay = NODECONFIG.read( EEADDR( servodelay ) );
  pcaServoSet(address);
}

// retrieve savePeriod
uint32_t getSavePeriod() {
  uint32_t saveperiod = NODECONFIG.read( EEADDR(saveperiod) );
  return saveperiod * saveperiod * 1000;
}

// On startup: set curpos[i] to 1 and set servo to 90 or saved angle
void natServoStartUp() {
  dP("\n natServoStartUp");
  servodelay = NODECONFIG.read( EEADDR( servodelay ) );
  for(int i=0; i<NUM_SERVOS; i++) {
    if(getSavePeriod()==0) curpos[i] = 1;                    // if positions are not saved, then default to middle position
    else curpos[i] = NODECONFIG.read( EEADDR( curpos[i] ) ); // else retrieve the last postion
    dP("\n i="); dP(i); dP(" curpos[i]="); dP(curpos[i]);
    if( USE_90_ON_STARTUP ) servoActual[i] = 90;                      // if to use 90degrees, then set its Actual postion to 90
    else servoActual[i] = 
         NODECONFIG.read( EEADDR( servos[i].pos[curpos[i]].angle ) ); // else retrieve the angle of the set position
    dP(" servoActual[i]="); dP(servoActual[i]);
    servo[i].write(servoActual[i]);
    delay(100);
    servo[i].attach(servopin[i]);   
    delay(100);
    servo[i].write(servoActual[i]);  // put the servo to its Actual position.  
    dP(" done");
  }
  servoSet();
}
// Allow Servo adjustments
void servoSet() {
  for(int i=0; i<NUM_SERVOS; i++) {
    servoTarget[i] = NODECONFIG.read( EEADDR( servos[i].pos[curpos[i]].angle ) );
  }
}

// Setup the io pins
// called by setup() and after a configuration change
void setupIOPins() {
  dP("\nPins: ");
  for(uint8_t i=0; i<NUM_IO; i++) {
    uint8_t type = NODECONFIG.read( EEADDR(io[i].type));
    dP("\n "); dP(i); dP(" type="); dP(type);
    switch (type) {  // No PULLUP
      case 1: case 2: case 5:
        if(type==1) dP(" IN");
        if(type==2) dP(" INI");
        if(type==5) dP(" TOG");
        if(i<NUM_NATIVE_IO) pinMode(iopin[i], INPUT);
        else {
          uint8_t ch = i-NUM_NATIVE_IO;
          //if(ch<16) mcp1.pinMode(ch, INPUT);
          //else mcp2.pinMode(ch-16, INPUT);
          mcp[ch/16]->pinMode(ch%16, INPUT);
        }
        iostate[i] = type&1;
        if(type==5) iostate[i] = 0;
        break;
      case 3: case 4: case 6: // PULLUPS
        if(type==3) dP(" INP");
        if(type==4) dP(" INPI");
        if(type==6) dP(" TOGP");
        if(i<NUM_NATIVE_IO) pinMode(iopin[i], INPUT_PULLUP); 
        else {
          uint8_t ch = i-NUM_NATIVE_IO;
          //if(ch<16) mcp1.pinMode(ch, INPUT_PULLUP);
          //else mcp2.pinMode(ch-16, INPUT_PULLUP);
          mcp[ch/16]->pinMode(ch%16, INPUT_PULLUP);
        }
        iostate[i] = type&1;
        break;
      case 7: case 8: case 9: case 10:
        if(type==7) dP(" PA");
        if(type==8) dP(" PAI");
        if(type==9) dP(" PB");
        if(type==10) dP(" PBI");
        if(i<NUM_NATIVE_IO) pinMode(iopin[i], OUTPUT); 
        else {
          uint8_t ch = i-NUM_NATIVE_IO;
          //if(ch<16) mcp1.pinMode(ch, OUTPUT); 
          //else mcp2.pinMode(ch-16, OUTPUT); 
          dP("\nmcp.pinMode ch="); dP(ch); delay(500);
          mcp[ch/16]->pinMode(ch%16, OUTPUT); 
        }
        bool inv = !(type&1);
        iostate[i] = inv;
        if(i<NUM_NATIVE_IO) digitalWrite(iopin[i], !type&1);
        else {
          uint8_t ch = i-NUM_NATIVE_IO;
          bool inv = !(type&1);
          //if(ch<16) mcp1.digitalWrite(ch, iostate[i]); 
          //else mcp2.digitalWrite(ch-16, iostate[i]);
          mcp[ch/16]->digitalWrite(ch%16, iostate[i]); 
        }
        break;
    }
    if(i<NUM_NATIVE_IO) { dP("\n native "); dP(iopin[i]); dP(":"); dP(type); }
    else { dP("\n mcp "); dP(i-NUM_NATIVE_IO); dP(":"); dP(type); }
  }
}

// Process IO pins
// called by loop to implement flashing on io pins
void appProcess() {
  uint8_t base = NUM_SERVOS * NUM_POS;
  unsigned long now = millis();
  for(int i=0; i<NUM_IO; i++) {
    uint8_t type = NODECONFIG.read(EEADDR(io[i].type));
    if(type >= 7) {
      if( next[i] && now>next[i] ) {
        //dP("\nappProcess "); PV(now);
        bool inv = !(type&1);
        bool phb = type>8;
        if(iostate[i]) {
          // phaseB
          dP("\nphaseB"); PV(i); PV(phb); PV(inv); PV(phb ^ inv);
          if(i<NUM_NATIVE_IO) {
            digitalWrite(iopin[i], phb ^ inv);
            PV(iopin[i]);
          } else {
            uint8_t ch = i-NUM_NATIVE_IO;
            //if(ch<16) mcp1.digitalWrite(ch, phb ^ inv);
            //else mcp2.digitalWrite(ch-16, phb ^ inv);
            mcp[ch/16]->digitalWrite(ch%16, phb ^ inv);
            PV(i-NUM_NATIVE_IO)
          }
          iostate[i] = 0;
          if( NODECONFIG.read(EEADDR(io[i].period)) > 0 ) 
          next[i] = now + 100*NODECONFIG.read(EEADDR(io[i].period));
          else next[i] = 0;
            PV(next[i]);
        } else {
          // phaseA
          dP("\nphaseA"); PV(i); PV(phb); PV(inv); PV(!phb ^ inv);
          if(i<NUM_NATIVE_IO) digitalWrite(iopin[i], !phb ^ inv);
          else {
            uint8_t ch = i-NUM_NATIVE_IO;
            ///if(ch<16) mcp1.digitalWrite(ch, !phb ^ inv);
            ///else mcp2.digitalWrite(ch-16, !phb ^ inv);
            mcp[ch/16]->digitalWrite(ch%16, !phb ^ inv);
          }
          iostate[i] = 1;
          if( NODECONFIG.read(EEADDR(io[i].duration)) > 0 )
            next[i] = now + 100*NODECONFIG.read(EEADDR(io[i].duration));
          else next[i] = 0;
            //PV(next[i]);
        }
      }
    }
  }
}

// ==== Setup does initial configuration ======================
void setup()
{
  #ifdef DEBUG
    #define dP(...) Serial.print(__VA_ARGS__)
    Serial.begin(115200); while(!Serial) delay(100); delay(2000);
    dP("\n AVR-2Servo8IOCAN");
    dP("\n num servos = "); dP(NUM_SERVOS);
    dP("\n num servo pos = "); dP(NUM_POS);
    dP("\n num native io = "); dP(NUM_NATIVE_IO);
    dP("\n num mcp groups = "); dP(NUM_MCP_PORTS);
    dP("\n num io in each mcp group = "); dP(NUM_MCP_IO_PER_PORT);
    dP("\n num mcp io = "); dP(NUM_MCP_IO);
    //dP("\n mcp23017-1 address = "); dPH(MCP_ADDRESS1);
    //dP("\n mcp23017-2 address = "); dPH(MCP_ADDRESS2);
    for(int i=0; i<NUM_MCP; i++) { dP("\n mcp23017("); dP(i); dP(") address = "); dPH(MCP_ADDRESSES[i]); }
    //dP("\n pca9685-1 address = "); dPH(PCA_ADDRESS1);
    //dP("\n pca9685-2 address = "); dPH(PCA_ADDRESS2);
    for(int i=0; i<NUM_PCA; i++) { dP("\n pca9685("); dP(i); dP(") address = "); dPH(PCA_ADDRESSES[i]); }
    dP("\n total num io = "); dP(NUM_IO);
    dP("\n num events = "); dP(NUM_EVENT);
    dP("\n num native servo events = "); dP(NUM_SERVO_EVENT);
    dP("\n num native io events = "); dP(NUM_NATIVE_IO_EVENT);
    dP("\n num mcp events = "); dP(NUM_MCP_EVENT);
    dP("\n num pca servo events = "); dP(NUM_PCA_SERVO_EVENT);
    dP("\n size of MemStruct = "); dP(sizeof(MemStruct));
  #endif

  WIRE_begin;                        // defined in boards.h
  EEPROMbegin;
  NodeID nodeid(NODE_ADDRESS);       // this node's nodeid
  Olcb_init(nodeid, RESET_TO_FACTORY_DEFAULTS);
  mcpinit();
  setupIOPins();
  natServoStartUp();
  pcaInit();
  dP("\n NUM_EVENT="); dP(NUM_EVENT);
  
//   #if 0
//   for(uint8_t i=0; i<NUM_IO; i++) {
//     NODECONFIG.write( EEADDR( io[i].type ), tNONE);
//     NODECONFIG.write( EEADDR( io[i].duration ), 0);
//     NODECONFIG.write( EEADDR( io[i].period ), 0);
//   }

//   NODECONFIG.write( EEADDR( io[16+2].type ), tPA);
//   NODECONFIG.write( EEADDR( io[16+2].duration ), 5);
//   NODECONFIG.write( EEADDR( io[16+2].period ), 5);

//   NODECONFIG.write( EEADDR( io[16+3].type ), tPB);
//   NODECONFIG.write( EEADDR( io[16+3].duration ), 5);
//   NODECONFIG.write( EEADDR( io[16+3].period ), 5);

//   NODECONFIG.write( EEADDR( io[16+4].type ), tIN);
//   NODECONFIG.write( EEADDR( io[16+4].duration ), 10);
//   NODECONFIG.write( EEADDR( io[16+4].period ), 10);
//   #endif

//     for(int i=0; i<NUM_IO; i++) {
//       dP("\n"); dP(i); 
//       dP(" type="); dP( NODECONFIG.read( EEADDR( io[i].type ) ) ); 
//     }

 }

// ==== Loop ==========================
void loop() {
  bool activity = Olcb_process();
  #ifndef OLCB_NO_BLUE_GOLD
    if (activity) {
      blue.blink(0x1); // blink blue to show that the frame was received
    }
    if (olcbcanTx.active) {
      gold.blink(0x1); // blink gold when a frame sent
      olcbcanTx.active = false;
    }
    // handle the status lights
    gold.process();
    blue.process();
  #endif // OLCB_NO_BLUE_GOLD
  produceFromInputs();  // scans inputs and generates events on change
  appProcess();         // processes io pins, eg flashing
  natServoProcess();       // processes servos, moves them to their target
  processProducer();    // processes delayed producer events from inputs
  //processPCA();
}

