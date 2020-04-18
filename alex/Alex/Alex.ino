#include <serialize.h>
#include <stdarg.h>
#include <math.h>
#include <avr/sleep.h>

#include "packet.h"
#include "constants.h"

#define PRR_TWI_MASK              0b10000000
#define PRR_SPI_MASK              0b00000100
#define ADCSRA_ADC_MASK           0b10000000
#define PRR_ADC_MASK              0b00000001
#define PRR_TIMER2_MASK           0b01000000
#define PRR_TIMER0_MASK           0b00100000
#define PRR_TIMER1_MASK           0b00001000
#define SMCR_SLEEP_ENABLE_MASK    0b00000001
#define SMCR_IDLE_MODE_MASK       0b11110001

#define THRESHOLD 3

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  REVERSE = 2,
  LEFT = 3,
  RIGHT = 4
  
} TDirection;

volatile TDirection dir = STOP;
// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      170

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.4

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  11  // Right forward pin
#define RR                  10  // Right reverse pin

// PI, for calculating turn circumference
//#define PI                  3.141592654

// Alex's length and breadth in cm
#define ALEX_LENGTH         16.5
#define ALEX_WIDTH          11.0

// Alex's diagonal. We compute and store this once
// since it is expensive to compute and really doesn't change.
float alexDiagonal = 0.0;

// Alex's turning circumference, calculated once
float alexCirc = 0.0;

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variables to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

// Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

void WDT_off()
{
  /* Global interrupt should be turned OFF here if not already done so */
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1<<WDRF);
  
  /* Write logical one to WDCE and WDE */
  /* Keep old prescaler setting to prevent unintentional time-out */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  
  /* Turn off WDT */
  WDTCSR = 0x00;
  
  /* Global interrupt should be turned ON here if subsequent operations after
  calling this function do not require turning off global interrupt */
}

void setupPowerSaving()
{
  // Turn off the Watchdog Timer
  WDT_off();

  // Modify PRR to shut down TWI
  PRR |= 0b10000000;

  // Modify PRR to shut down SPI
  PRR |= 0b00000100;
  
  // Modify ADCSRA to disable ADC,
  // then modify PRR to shut down ADC
  ADCSRA &= 0b01111111;
  PRR |= 0b00000001;
  
  // Set the SMCR to choose the IDLE sleep mode
  // Do not set the Sleep Enable (SE) bit yet
  SMCR &= 0b11110001;
  
  // Set Port B Pin 5 as output pin, then write a logic LOW
  // to it so that the LED tied to Arduino's Pin 13 is OFF.
  DDRB |= 0b00100000;
  PORTB &= 0b1101111;
}

void putArduinoToIdle()
{
  // Modify PRR to shut down TIMER 0, 1, and 2
  PRR |= 0b01101000;
  
  // Modify SE bit in SMCR to enable (i.e., allow) sleep
  SMCR |= 0b00000001;
    
  // The following function puts ATmega328P’s MCU into sleep;
  // it wakes up from sleep when USART serial data arrives
  sleep_cpu();
  
  // Modify SE bit in SMCR to disable (i.e., disallow) sleep
  SMCR &= 0b11111110;
  
  // Modify PRR to power up TIMER 0, 1, and 2
  PRR &= 0b10010111;
}

TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);
    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;

  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;

  sendResponse (&statusPacket);
  
}

void sendMessage(const char *message)
{ 
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}


void dbprint(char *format, ...){
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  if (dir == FORWARD){
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == REVERSE){
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == LEFT)
    leftReverseTicksTurns++;
  else if (dir == RIGHT)
    leftForwardTicksTurns++;
  stabilisation();
}

void rightISR()
{
  if (dir == FORWARD)
    rightForwardTicks++;
  else if (dir == REVERSE)
    rightReverseTicks++;
  else if (dir == RIGHT)
    rightReverseTicksTurns++;
  else if (dir == LEFT)
    rightForwardTicksTurns++;
  stabilisation();
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EICRA |= 0b00001010;
  EIMSK |= 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect){
  leftISR();
}

ISR(INT1_vect){
  rightISR();
}

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  
  // Setup USART to 8N1 at 9600bps
  UCSR0C = 0b00000110;
  UCSR0A = 0;
  
  // Set higher bytes and lower bytes
  UBRR0L = 103;
  UBRR0H = 0;
  
  //Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  UCSR0B = 0b10011000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{
  // receiving data
  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  // sending data
  Serial.write(buffer, len);
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  DDRD |= 0b01100000;
  DDRB |= 0b00001100;
    
  TCNT0 = 0;
  TCNT1 = 0;
  TCNT2 = 0;
  
  sei(); 
  
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B, LR
   *    A2IN - Pin 6, PD6, OC0A, LF
   *    B1IN - Pin 10, PB2, OC1B, RR
   *    B2In - pIN 11, PB3, OC2A, RF
   */
}

ISR(TIMER0_COMPA_VECT) {
}

ISR(TIMER0_COMPB_VECT) {
}

ISR(TIMER1_COMPB_VECT) {
}

ISR(TIMER2_COMPA_VECT) {
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  // count to this value
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0); 
}

int pwmVal_timer1(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 65535.0); //Original 100
}

// Convert percentages to PWM values
int pwmVal_right(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0; 

  return (int) ((speed / 100.0) * 255.0);
}

//Stabilisation function
void stabilisation()
{
  int lefttickDifference = 0, righttickDifference = 0;
  
  switch(dir) {
    case FORWARD: lefttickDifference = leftForwardTicks - rightForwardTicks;
                  righttickDifference = rightForwardTicks - leftForwardTicks;
                  // Left moving faster
                  if(lefttickDifference >= THRESHOLD) {              
                    if(OCR0A <= 10)
                      OCR0A = 10;
                    else
                      OCR0A -= 10;
                    if(OCR2A >= 250)
                      OCR0A = 200;
                    else
                      OCR2A += 10;
                    return;                                       
                  } 
                  // Right moving faster
                  else if (righttickDifference >= THRESHOLD) {     
                    if(OCR2A <= 10)
                      OCR2A = 10;
                    else
                      OCR2A -= 10;
                    if(OCR0A >= 250)
                      OCR2A = 200;
                    else
                      OCR0A += 10;
                    return;    
                  }
                  break;
    case REVERSE: lefttickDifference = leftReverseTicks - rightReverseTicks;
                  righttickDifference = rightReverseTicks - leftReverseTicks;
                  //Left Moving Faster
                  if(lefttickDifference >= THRESHOLD) {
                    if(OCR0B <= 10)
                      OCR0B = 10;
                    else
                      OCR0B -= 10;
                    if(OCR1B >= 64224)
                      OCR0B = 225;
                    else
                      OCR1B += 2570;
                    return;
                  }
                  //Right Moving Faster
                  else if(righttickDifference >= THRESHOLD) {
                    if(OCR1B <= 2620)
                      OCR1B = 2620;
                    else
                      OCR1B -= 2570;
                    if(OCR0B >= 250)
                      OCR1B = 58000;
                    else
                      OCR0B += 10;
                    return;
                  }
                  break;
    case LEFT: return;
    case RIGHT: return;
  }
  return;
}


// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{

  dir = FORWARD;

  clearCounters();
  
  int val = pwmVal(speed);
  int val_right = pwmVal_right(speed);

  // Code to tell us how far to move

  if (dist == 0)
    deltaDist = 999999;
  else
    deltaDist = dist;

  newDist = forwardDist + deltaDist;
  
  TCCR0A = 0b10000001;
  TCCR2A = 0b10000001;
  TCCR1A = 0b00000001;
  
  OCR0A = val;
  OCR2A = val_right;
  
  TCCR0B = 0b00000011;
  TCCR2B = 0b00000011;
}
 
// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  dir = REVERSE; 

  clearCounters();

  int val = pwmVal(speed);
  int val_right = pwmVal_timer1(speed);  

  if (dist == 0)
    deltaDist = 999999;
  else
    deltaDist = dist;

  newDist = reverseDist + deltaDist;

 // Reverse - OC0B and OC1B
  TCCR0A = 0b00100001;
  TCCR1A = 0b00100001;
  TCCR2A = 0b00000001;
  
  // Set pwm value
  OCR0B = val;
  OCR1B = val_right;
  
  // Start Timer
  TCCR0B = 0b00000011;
  TCCR1B = 0b00000011;
}


unsigned long computeDeltaTicks (float ang)
{
  unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}


// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;
  
  int val = pwmVal(speed);
  int val_right = pwmVal_right(speed);
  
  if (ang == 0)
    deltaTicks = 99999999;
  else
    deltaTicks = computeDeltaTicks(ang);

  targetTicks = (leftReverseTicksTurns + deltaTicks) / 2;

  //Turn left
  TCCR0A = 0b00100001;
  TCCR2A = 0b10000001;
  TCCR1A = 0b00000001;
  
  //Set PWM value
  OCR0B = val;
  OCR2A = val_right;
  
  //Start Timer
  TCCR0B = 0b00000011;
  TCCR2B = 0b00000011;  
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;

  clearCounters();
  
  int val = pwmVal(speed);
  int val_right = pwmVal_timer1(speed);  

  if (ang == 0)
    deltaTicks = 99999999;
  else
    deltaTicks = computeDeltaTicks(ang);

  targetTicks = (leftForwardTicksTurns + deltaTicks) / 2;
  
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  //Move Right
  TCCR0A = 0b10000001;
  TCCR2A = 0b00000001;
  TCCR1A = 0b00100001;
  
  //Set PWM value
  OCR0A = val;
  OCR1B = val_right;
  
  //Start Timer
  TCCR0B = 0b00000011;
  TCCR1B = 0b00000011;
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  //Stop
  TCCR0A = 0b10000001;
  TCCR2A = 0b10000001;
  TCCR1A = 0b00000001;
  
  //Set PWM value
  OCR0A = 0;
  OCR2A = 0;
  
  //Start Timer
  TCCR0B = 0b00000011;
  TCCR2B = 0b00000011;
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  
  leftForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightForwardTicksTurns = 0;
  rightReverseTicksTurns = 0;
  
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}

// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_STOP:
      sendOK();
      stop();
      break;

    case COMMAND_GET_STATS:
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
      break;

    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:

  // Compute the diagonal
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_WIDTH * ALEX_WIDTH));
  alexCirc = PI * alexDiagonal;
  
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  enablePullups();
  initializeState();
  setupPowerSaving();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
  TPacket recvPacket; // This holds commands from the Pi
  
  if (dir == STOP) putArduinoToIdle();
  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else if(result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if(result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  } 


  if (deltaDist > 0)
  {
    if (dir == FORWARD)
    {
      if (forwardDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == REVERSE)
    {
      if (reverseDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == STOP){
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }


  if (deltaTicks > 0)
  {
    if (dir == LEFT)
    {
      if (leftReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
      
    }
    else if (dir == RIGHT)
    {
      if (rightReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == STOP){
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
}
