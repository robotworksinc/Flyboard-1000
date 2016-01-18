#include <PinChangeInt.h>

// Variable to read channel values into
int channels[4];

// Define RC channels pin numbers
#define RC_THROTTLE      2
#define RC_ROLL          4
#define RC_PITCH         7
#define RC_YAW           8
#define RC_IN_PIN_5     12
#define RC_IN_PIN_6     13

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG   1
#define ROLL_FLAG       2
#define PITCH_FLAG      4
#define YAW_FLAG        8

// Volatile global variables
volatile uint8_t bUpdateFlagsShared;
volatile uint16_t unThrottleInShared, unYawInShared, unPitchInShared, unRollInShared;

uint32_t ulThrottleStart, ulYawStart, ulPitchStart, ulRollStart;


// ===== RC Interrupt functions =================================================
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


void read(int channels[]) {
  // local copy of channel inputs
  static uint16_t unThrottleIn;
  static uint16_t unYawIn;
  static uint16_t unPitchIn;
  static uint16_t unRollIn;
  
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to  see if any channels have a new signal
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
    channels[0] = unThrottleIn;
  }

  if(bUpdateFlags & ROLL_FLAG)
  {
    channels[1] = unRollIn;
  }

  if(bUpdateFlags & PITCH_FLAG)
  {
    channels[2] = unPitchIn;
  } 
  
  if(bUpdateFlags & YAW_FLAG)
  {
    channels[3] = unYawIn;
  } 

  bUpdateFlags = 0;
  
}


void setup() {
  // Setup Serial port
  Serial.begin(115200);

  // Attach pin interrupt to RC channels
  PCintPort::attachInterrupt(RC_THROTTLE,calcThrottle,CHANGE);
  PCintPort::attachInterrupt(RC_ROLL,calcRoll,CHANGE);
  PCintPort::attachInterrupt(RC_PITCH,calcPitch,CHANGE);
  PCintPort::attachInterrupt(RC_YAW,calcYaw,CHANGE);  
}

void loop() {
  // Get values from 4 RC channels (Throttle, Yaw, Pitch and Roll)
  read(channels);

  Serial.println("Throttle:");
  Serial.println(channels[0]);

  Serial.println("Roll:");
  Serial.println(channels[1]);

  Serial.println("Pitch:");
  Serial.println(channels[2]);

  Serial.println("Yaw:");
  Serial.println(channels[3]);
}
