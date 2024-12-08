#include <Wire.h>

//#define VERBOSE_1
//#define VERBOSE_2
//#define VERBOSE_3

//#define DEBUG


#define A0_READ_PIN                                   P1_3
#define A1_READ_PIN                                   P1_4
#define A2_READ_PIN                                   P1_5
#define DETECT_MINER_RESET_PIN                        P2_4
#define I2C_SDA_PULLUP_PIN                            P2_5
#define LED_ON_PIN                                    P2_6
#define SDA_FLOW_CONTROL_PIN                          P2_7

#define CLEAR_MINER_RESET_INTERRUPT()                 (P2IFG &= ~BIT4)
#define MAX_HB_RESET_COUNT_BEFORE_SELF_RESET          2

#define I2C_RX_BUFFER_LEN                             20
#define I2C_LONG_RESPONSE_BUFFER_LEN                  7
#define ATTENTION_BYTE_INDEX                          2
#define RESPONSE_SWITCH_BYTE_INDEX                    3
#define VOLTAGE_READ_REQUEST_SENTINEL                 0x3A

#define SDA_FLOW_CONTROL_PIN_MAX_LOW_INTERVAL_ms      225     //Must be multiple of 5ms (5ms is min resolution)
#define TA0_COUNT_UP_TO_VALUE                         10000   //Value set to interrupt every 5ms when TA0 CLK FREQ = 2MHz (SMCLK / ID_3 = 16000000 / 8)                             
#define MAX_TIMER_ROLLOVER_COUNT                      SDA_FLOW_CONTROL_PIN_MAX_LOW_INTERVAL_ms / (TA0_COUNT_UP_TO_VALUE / 2000)

/*
 * Setting SINGLE_SHOT_TIMER_COUNT_VALUE too low causes Bitmain FW to fail. Bitmain FW takes longer than most firmwares to complete 
 * a voltage read request. If the single shot timer is too short, Loki-HBS gives control of the SDA line back to the hashbaord in the 
 * middle of a voltage read command set. The hashboard never saw the initial commands for the voltage read, so it responds with 0x00.
 * This causes the miner to fault and power down.
 * 
 * On the other hand, setting SINGLE_SHOT_TIMER_COUNT_VALUE too high causes Vnish FW to fail. This is because Vnish tends to
 * take I2C temperature reads quickly after reading the hashboard voltage. If the single shot timer is too long, Loki-HBS 
 * retains control at the start of the temperature request command set and doesn't recognize the request as a voltage read 
 * request, so it responds with 0x00.
*/
#define SINGLE_SHOT_TIMER_COUNT_VALUE                 (3500 * 2) //(n * F) where n is desired single shot time in useconds and F is TA1CLK frequency in MHz


#ifdef DEBUG 
  #define DEBUG_BROADCAST_INTERVAL_MS                   1000
  unsigned long last_debug_broadcast_time = 0;
#endif

bool new_i2c_data_received = false;
bool new_i2c_data_requested = false;

byte i2c_read_buffer[I2C_RX_BUFFER_LEN];
int8_t  i2c_read_buffer_index = 0;
int8_t i2c_tx_response_index = 0;

//PSU Voltage Message Preamble
const byte psu_voltage_msg_preamble[2] = {0xFE, 0xFF};

//HB Spoofed Voltage Responses
//const byte h3A_initailResp[I2C_LONG_RESPONSE_BUFFER_LEN] = {0x07, 0x3A, 0x01, 0x02, 0x7B, 0x00, 0xBF};  //Default voltage read response if no PSU voltage message has been received
const byte h3A_dynamicResp[256][2] = {
  {0x02, 0x7B}, {0x02, 0x7B}, {0x02, 0x7A}, {0x02, 0x7A}, {0x02, 0x79}, {0x02, 0x79},
  {0x02, 0x78}, {0x02, 0x78}, {0x02, 0x77}, {0x02, 0x77}, {0x02, 0x76}, {0x02, 0x76},
  {0x02, 0x75}, {0x02, 0x75}, {0x02, 0x74}, {0x02, 0x74}, {0x02, 0x73}, {0x02, 0x73},
  {0x02, 0x72}, {0x02, 0x72}, {0x02, 0x71}, {0x02, 0x71}, {0x02, 0x70}, {0x02, 0x70},
  {0x02, 0x6F}, {0x02, 0x6E}, {0x02, 0x6E}, {0x02, 0x6D}, {0x02, 0x6D}, {0x02, 0x6C},
  {0x02, 0x6C}, {0x02, 0x6B}, {0x02, 0x6B}, {0x02, 0x6A}, {0x02, 0x6A}, {0x02, 0x69},
  {0x02, 0x69}, {0x02, 0x68}, {0x02, 0x68}, {0x02, 0x67}, {0x02, 0x67}, {0x02, 0x66},
  {0x02, 0x66}, {0x02, 0x65}, {0x02, 0x65}, {0x02, 0x64}, {0x02, 0x64}, {0x02, 0x63},
  {0x02, 0x63}, {0x02, 0x62}, {0x02, 0x62}, {0x02, 0x61}, {0x02, 0x61}, {0x02, 0x60},
  {0x02, 0x60}, {0x02, 0x5F}, {0x02, 0x5F}, {0x02, 0x5E}, {0x02, 0x5E}, {0x02, 0x5D},
  {0x02, 0x5C}, {0x02, 0x5C}, {0x02, 0x5B}, {0x02, 0x5B}, {0x02, 0x5A}, {0x02, 0x5A},
  {0x02, 0x59}, {0x02, 0x59}, {0x02, 0x58}, {0x02, 0x58}, {0x02, 0x57}, {0x02, 0x57},
  {0x02, 0x56}, {0x02, 0x56}, {0x02, 0x55}, {0x02, 0x55}, {0x02, 0x54}, {0x02, 0x54},
  {0x02, 0x53}, {0x02, 0x53}, {0x02, 0x52}, {0x02, 0x52}, {0x02, 0x51}, {0x02, 0x51},
  {0x02, 0x50}, {0x02, 0x50}, {0x02, 0x4F}, {0x02, 0x4F}, {0x02, 0x4E}, {0x02, 0x4E},
  {0x02, 0x4D}, {0x02, 0x4D}, {0x02, 0x4C}, {0x02, 0x4C}, {0x02, 0x4B}, {0x02, 0x4A},
  {0x02, 0x4A}, {0x02, 0x49}, {0x02, 0x49}, {0x02, 0x48}, {0x02, 0x48}, {0x02, 0x47},
  {0x02, 0x47}, {0x02, 0x46}, {0x02, 0x46}, {0x02, 0x45}, {0x02, 0x45}, {0x02, 0x44},
  {0x02, 0x44}, {0x02, 0x43}, {0x02, 0x43}, {0x02, 0x42}, {0x02, 0x42}, {0x02, 0x41},
  {0x02, 0x41}, {0x02, 0x40}, {0x02, 0x40}, {0x02, 0x3F}, {0x02, 0x3F}, {0x02, 0x3E},
  {0x02, 0x3E}, {0x02, 0x3D}, {0x02, 0x3D}, {0x02, 0x3C}, {0x02, 0x3C}, {0x02, 0x3B},
  {0x02, 0x3B}, {0x02, 0x3A}, {0x02, 0x3A}, {0x02, 0x39}, {0x02, 0x38}, {0x02, 0x38},
  {0x02, 0x37}, {0x02, 0x37}, {0x02, 0x36}, {0x02, 0x36}, {0x02, 0x35}, {0x02, 0x35},
  {0x02, 0x34}, {0x02, 0x34}, {0x02, 0x33}, {0x02, 0x33}, {0x02, 0x32}, {0x02, 0x32},
  {0x02, 0x31}, {0x02, 0x31}, {0x02, 0x30}, {0x02, 0x30}, {0x02, 0x2F}, {0x02, 0x2F},
  {0x02, 0x2E}, {0x02, 0x2E}, {0x02, 0x2D}, {0x02, 0x2D}, {0x02, 0x2C}, {0x02, 0x2C},
  {0x02, 0x2B}, {0x02, 0x2B}, {0x02, 0x2A}, {0x02, 0x2A}, {0x02, 0x29}, {0x02, 0x29},
  {0x02, 0x28}, {0x02, 0x28}, {0x02, 0x27}, {0x02, 0x26}, {0x02, 0x26}, {0x02, 0x25},
  {0x02, 0x25}, {0x02, 0x24}, {0x02, 0x24}, {0x02, 0x23}, {0x02, 0x23}, {0x02, 0x22},
  {0x02, 0x22}, {0x02, 0x21}, {0x02, 0x21}, {0x02, 0x20}, {0x02, 0x20}, {0x02, 0x1F},
  {0x02, 0x1F}, {0x02, 0x1E}, {0x02, 0x1E}, {0x02, 0x1D}, {0x02, 0x1D}, {0x02, 0x1C},
  {0x02, 0x1C}, {0x02, 0x1B}, {0x02, 0x1B}, {0x02, 0x1A}, {0x02, 0x1A}, {0x02, 0x19},
  {0x02, 0x19}, {0x02, 0x18}, {0x02, 0x18}, {0x02, 0x17}, {0x02, 0x17}, {0x02, 0x16},
  {0x02, 0x16}, {0x02, 0x15}, {0x02, 0x14}, {0x02, 0x14}, {0x02, 0x13}, {0x02, 0x13},
  {0x02, 0x12}, {0x02, 0x12}, {0x02, 0x11}, {0x02, 0x11}, {0x02, 0x10}, {0x02, 0x10},
  {0x02, 0x0F}, {0x02, 0x0F}, {0x02, 0x0E}, {0x02, 0x0E}, {0x02, 0x0D}, {0x02, 0x0D},
  {0x02, 0x0C}, {0x02, 0x0C}, {0x02, 0x0B}, {0x02, 0x0B}, {0x02, 0x0A}, {0x02, 0x0A},
  {0x02, 0x09}, {0x02, 0x09}, {0x02, 0x08}, {0x02, 0x08}, {0x02, 0x07}, {0x02, 0x07},
  {0x02, 0x06}, {0x02, 0x06}, {0x02, 0x05}, {0x02, 0x05}, {0x02, 0x04}, {0x02, 0x04},
  {0x02, 0x03}, {0x02, 0x02}, {0x02, 0x02}, {0x02, 0x01}, {0x02, 0x01}, {0x02, 0x00},
  {0x02, 0x00}, {0x01, 0xFF}, {0x01, 0xFF}, {0x01, 0xFE}, {0x01, 0xFE}, {0x01, 0xFD},
  {0x01, 0xFD}, {0x01, 0xFC}, {0x01, 0xFC}, {0x01, 0xFB}, {0x01, 0xFB}, {0x01, 0xFA},
  {0x01, 0xFA}, {0x01, 0xF9}, {0x01, 0xF9}, {0x01, 0xF8} 
};

char assigned_I2C_address;
uint16_t timer_rollover_count = 0;
bool initial_spoof_complete = false;
uint8_t hb_reset_count = 0;

bool psu_voltage_message_received = false;
uint8_t voltage_resp_index = 0;

void setup() {
  pinMode(SDA_FLOW_CONTROL_PIN, OUTPUT);
  digitalWrite(SDA_FLOW_CONTROL_PIN, HIGH);

  pinMode(I2C_SDA_PULLUP_PIN, INPUT);

  pinMode(LED_ON_PIN, OUTPUT);
  digitalWrite(LED_ON_PIN, LOW);
  delay(200);
  digitalWrite(LED_ON_PIN, HIGH);
  
  pinMode(DETECT_MINER_RESET_PIN, INPUT_PULLUP);
  
  //Setup Serial comms with console
  Serial.begin(9600);           // start serial for output
  #ifdef VERBOSE_1
    Serial.println("LokiHBS online...");
  #endif

  //Set Ax_READ pins as input with pullup resistor to read port assigned I2C address
  pinMode(A0_READ_PIN, INPUT_PULLUP);
  pinMode(A1_READ_PIN, INPUT_PULLUP);
  pinMode(A2_READ_PIN, INPUT_PULLUP);
  assigned_I2C_address = 0x20 + digitalRead(A0_READ_PIN) + digitalRead(A1_READ_PIN) * 2 + digitalRead(A2_READ_PIN) * 4;
  #ifdef VERBOSE_1
    Serial.print("I2C address: ");
    Serial.println(assigned_I2C_address, HEX);
  #endif
  
  //Setup device as I2C slave
  Wire.setModule(0);            // map i2c functionality to module 0 pinset (1.6, 1.7)
  Wire.begin(assigned_I2C_address);         // join i2c bus with port assigned i2c address
  Wire.onRequest(requestEvent); // Function to run when data requested from master
  Wire.onReceive(receiveEvent); // Function to run when data received from master

  //Prepare SDA_FLOW_CONTROL_PIN safety timer but don't start it yet
  TA0CTL = TACLR;
  TA0CCR0 = TA0_COUNT_UP_TO_VALUE;
  TA0CCTL0 = CCIE;

  //Enable interrupt on MINER_DETECT_RESET_PIN in order to restart Loki when miner restarts
  CLEAR_MINER_RESET_INTERRUPT();
  attachInterrupt(digitalPinToInterrupt(DETECT_MINER_RESET_PIN), miner_reset_event_isr, FALLING);
}

void loop() {
#ifdef VERBOSE_3
  if (new_i2c_data_received) {
    if (i2c_read_buffer[i2c_read_buffer_index - 1] == 0x00) {
      Serial.write(i2c_read_buffer, (i2c_read_buffer_index));
    }
    new_i2c_data_received = false;
  }
#endif
  //Check to see if new PSU voltage message has been received
  
  while(Serial.available()) {
    voltage_resp_index = Serial.read();
    #ifdef VERBOSE_1
      Serial.println(voltage_resp_index);
    #endif
  }

  #ifdef DEBUG
    if(millis() - last_debug_broadcast_time >= DEBUG_BROADCAST_INTERVAL_MS){
      last_debug_broadcast_time = millis();
      Serial.println(voltage_resp_index);
    }
  #endif

  if(initial_spoof_complete) {
    digitalWrite(LED_ON_PIN, LOW);
  }

  delay(1);
}

// *************************** ISRs *******************************

void receiveEvent(int howMany)
{
  //flush i2c RX buffer if this is the first byte being received since last read
  if(i2c_read_buffer_index == 0) {
    for(int i = 0; i < I2C_RX_BUFFER_LEN; i++) {
      i2c_read_buffer[i] = 0xFF;
    }
  }
  while(Wire.available() && i2c_read_buffer_index < I2C_RX_BUFFER_LEN) {
    char c = Wire.read();
    i2c_read_buffer[i2c_read_buffer_index] = c;
    i2c_read_buffer_index++;
#ifdef VERBOSE_2
    Serial.print(c);
#endif
  }
  i2c_tx_response_index = 0;
  new_i2c_data_received = true;
}

void requestEvent()
{
  i2c_read_buffer_index = 0;
  if (i2c_read_buffer[RESPONSE_SWITCH_BYTE_INDEX] == VOLTAGE_READ_REQUEST_SENTINEL) {
 
    if(i2c_tx_response_index == 0){
        //Voltage read request is being sent. Preempt HB response by disconnecting HB's SDA line from I2C
        //Enable pullup on Loki's SDA line
        pinMode(I2C_SDA_PULLUP_PIN, INPUT_PULLUP);
        //Disconnect HB from SDA line and connect Loki directly to SDA line
        digitalWrite(SDA_FLOW_CONTROL_PIN, LOW);
        //Start timer to ensure that SDA_FLOW_CONTROL_PIN doesn't stay low for too long
        TA0CTL = TASSEL_2 + ID_3 + MC_1;
        timer_rollover_count = 0;        
    }
  
    //0x3A voltage read request received. Time to spoof voltage response.
    byte b;
    uint16_t cs;
    switch(i2c_tx_response_index) {
      case 0: //prepare to send first preamble byte
        b = 0x07; 
        break;
      
      case 1: //prepare to send second preamble byte
        b = 0x3A; 
        break;
      
      case 2: //prepare to send third preamble byte
        b = 0x01; 
        break;
      
      case 3: //prepare to send first data byte
        b = h3A_dynamicResp[voltage_resp_index][0]; 
        break;
      
      case 4: //prepare to send second data byte
        b = h3A_dynamicResp[voltage_resp_index][1]; 
        break;
      
      case 5: //prepare to send first checksum byte (checksum sums all preceding bytes including preamble)
        cs = 0x42 + h3A_dynamicResp[voltage_resp_index][0] + h3A_dynamicResp[voltage_resp_index][1];
        b = cs >> 8;
        break;
        
      case 6: //prepare to send second checksum byte (checksum sums all preceding bytes including preamble)
        cs = 0x42 + h3A_dynamicResp[voltage_resp_index][0] + h3A_dynamicResp[voltage_resp_index][1];
        b = cs & 0xFF;
        break;
        
      default:
        b = 0x00;
    }
    Wire.write(b);
    #ifdef VERBOSE_2
      Serial.print(b);
    #endif
    
    if(i2c_tx_response_index == 6){
      //last byte has been written. Set single shot timer to set SDA_FLOW_CONTROL_PIN
      TA1CCR0 = SINGLE_SHOT_TIMER_COUNT_VALUE;
      TA1CCTL0 = CCIE;
      TA1CTL = TASSEL_2 + ID_3 + MC_2;
      initial_spoof_complete = true;
    }

    i2c_tx_response_index++;
  }
}


// *************************** ISRs *******************************


void miner_reset_event_isr(){
  //CB has reset HB. Reset Loki to clear any possible out-of-sync errors
  hb_reset_count++;
  if(hb_reset_count >= MAX_HB_RESET_COUNT_BEFORE_SELF_RESET){
    WDTCTL = 0; 
  }
}

/*
 * TIMER A0 serves as a pseudo watchdog timer for the SDA_FLOW_CONTROL_PIN.
 * If the SDA_FLOW_CONTROL_PIN stays low for too long, the miner's control board
 * will not be able to communicate with the hash board at all. 
 */
__attribute__((interrupt(TIMER0_A0_VECTOR)))
void myTimer_A0(void)
{
  timer_rollover_count++;
  if(timer_rollover_count == MAX_TIMER_ROLLOVER_COUNT){
    //SDA_FLOW_CONTROL_PIN has been low for too long. Force assert it.
    digitalWrite(SDA_FLOW_CONTROL_PIN, HIGH);
    pinMode(I2C_SDA_PULLUP_PIN, INPUT);
    TA0CTL = TACLR;
    timer_rollover_count = 0;
  }  
}

/*
 * TIMER A1 acts as a single shot timer to set the SDA_FLOW_CONTROL_PIN
 * X microseconds after the last voltage read byte (check sum) has been written to the  
 * i2c buffer. If the pin is set immediately after writing to the i2c buffer,
 * the miner's control boards never sees the last byte, and the entire i2c voltage
 * read is discarded by the control board.
 */
__attribute__((interrupt(TIMER1_A0_VECTOR)))
void myTimer_A1(void)
{
  TA1CTL = TACLR;
  
  //All done spoofing voltage read. Time to give HB control of SDA line again.
  digitalWrite(SDA_FLOW_CONTROL_PIN, HIGH);
  pinMode(I2C_SDA_PULLUP_PIN, INPUT);
  //Stop safety timer since SDA_FLOW_CONTROL_PIN has been asserted naturally
  TA0CTL = TACLR;
}
