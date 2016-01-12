#include <PinChangeInt.h>

#include <Servo.h>

// Define RC channels pin numbers
#define RC_THROTTLE      2
#define RC_PITCH         4
#define RC_ROLL          7
#define RC_YAW           8
#define RC_IN_PIN_5     12
#define RC_IN_PIN_6     13 

// Define drone motor pin numbers
#define MOTOR_FL         9
#define MOTOR_FR        10
#define MOTOR_BL         6
#define MOTOR_BR        11

// Define ESC servo objects
Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

// Radio min/max for each stick
#define RC_MIN       1050
#define RC_MAX       1800
#define ESC_MIN      1400
#define ESC_MAX      1750

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG   1
#define YAW_FLAG        2
#define PITCH_FLAG      4
#define ROLL_FLAG       8

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unYawInShared;
volatile uint16_t unPitchInShared;
volatile uint16_t unRollInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulYawStart;
uint32_t ulPitchStart;
uint32_t ulRollStart;

// Intermediate variables to hold values
uint16_t fast, slow, rcthr, rcyaw, rcpit, rcroll;


void setup()
{
  Serial.begin(115200);
 
  ESC1.attach(MOTOR_FL);
  ESC2.attach(MOTOR_FR);
  ESC3.attach(MOTOR_BL);
  ESC4.attach(MOTOR_BR);

  PCintPort::attachInterrupt(RC_THROTTLE,calcThrottle,CHANGE);
  PCintPort::attachInterrupt(RC_YAW,calcYaw,CHANGE);
  PCintPort::attachInterrupt(RC_PITCH,calcPitch,CHANGE);
  PCintPort::attachInterrupt(RC_ROLL,calcRoll,CHANGE);
}

void loop()
{
  // local copy of channel inputs
  static uint16_t unThrottleIn;
  static uint16_t unYawIn;
  static uint16_t unPitchIn;
  static uint16_t unRollIn;
  
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
   
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }

    if(bUpdateFlags & YAW_FLAG)
    {
      unYawIn = unYawInShared;
    }

    if(bUpdateFlags & PITCH_FLAG)
    {
      unPitchIn = unPitchInShared;
    }
    
    if(bUpdateFlags & ROLL_FLAG)
    {
      unRollIn = unRollInShared;
    }
    
    bUpdateFlagsShared = 0;
   
    interrupts(); 
  }
 
  if(bUpdateFlags & THROTTLE_FLAG)
  {
    rcthr = map(unThrottleIn, RC_MIN, RC_MAX, ESC_MIN, ESC_MAX);
    ESC1.writeMicroseconds(rcthr);
    ESC2.writeMicroseconds(rcthr);
    ESC3.writeMicroseconds(rcthr);
    ESC4.writeMicroseconds(rcthr);
  }
 
  if(bUpdateFlags & YAW_FLAG)
  {
    rcthr = map(unThrottleIn, RC_MIN, RC_MAX, ESC_MIN, ESC_MAX);
    rcyaw = map(unYawIn, RC_MIN, RC_MAX, -200, 200)    
    fast = constrain(rcthr + rcyaw, ESC_MIN, ESC_MAX);
    slow = constrain(rcthr - rcyaw, ESC_MIN, ESC_MAX);
    ESC1.writeMicroseconds(fast);
    ESC4.writeMicroseconds(fast);
    ESC2.writeMicroseconds(slow);
    ESC3.writeMicroseconds(slow);
  } 

  if(bUpdateFlags & PITCH_FLAG)
  {
    rcthr = map(unThrottleIn, RC_MIN, RC_MAX, ESC_MIN, ESC_MAX);
    rcpit = map(unPitchIn, RC_MIN, RC_MAX, -170, 170)    
    fast = constrain(rcthr + rcpit, ESC_MIN, ESC_MAX);
    slow = constrain(rcthr - rcpit, ESC_MIN, ESC_MAX);
    ESC1.writeMicroseconds(fast);
    ESC2.writeMicroseconds(fast);
    ESC3.writeMicroseconds(slow);
    ESC4.writeMicroseconds(slow);
  } 

  if(bUpdateFlags & ROLL_FLAG)
  {
    rcthr = map(unThrottleIn, RC_MIN, RC_MAX, ESC_MIN, ESC_MAX);
    rcroll = map(unRollIn, RC_MIN, RC_MAX, -170, 170)    
    fast = constrain(rcthr + rcroll, ESC_MIN, ESC_MAX);
    slow = constrain(rcthr - rcroll, ESC_MIN, ESC_MAX);
    ESC1.writeMicroseconds(fast);
    ESC3.writeMicroseconds(fast);
    ESC2.writeMicroseconds(slow);
    ESC4.writeMicroseconds(slow);
  }
              
  bUpdateFlags = 0;
}


// simple interrupt service routine
void calcThrottle()
{
  if(digitalRead(RC_THROTTLE) == HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}


void calcYaw()
{
  if(digitalRead(RC_YAW) == HIGH)
  {
    ulYawStart = micros();
  }
  else
  {
    unYawInShared = (uint16_t)(micros() - ulYawStart);
    bUpdateFlagsShared |= YAW_FLAG;
  }
}

void calcPitch()
{
  if(digitalRead(RC_PITCH) == HIGH)
  {
    ulPitchStart = micros();
  }
  else
  {
    unPitchInShared = (uint16_t)(micros() - ulPitchStart);
    bUpdateFlagsShared |= PITCH_FLAG;
  }
}


void calcRoll()
{
  if(digitalRead(RC_ROLL) == HIGH)
  {
    ulRollStart = micros();
  }
  else
  {
    unRollInShared = (uint16_t)(micros() - ulRollStart);
    bUpdateFlagsShared |= ROLL_FLAG;
  }
}