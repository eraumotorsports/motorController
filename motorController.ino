/*******************************************************************************
                      ERAU Formula Hybrid Motor Controller
********************************************************************************

  There are 3 modes of operation...

    Gas: Regardless of pedal/rpm values the motor will always be off.
    Electric: The motor output will be controled by pedal position.
    Hybrid: The motor output is determined by engine RPM.

  Operation mode is determined by mode inputs pins. Set a mode pin HIGH to
  enable the corresponding mode.  If both pins are LOW it will result in
  Hybrid mode.

*******************************************************************************/

#include "mode.h"

// To disable serial messages, comment out the following line
#define VERBOSE
#define RPMDEBUG

// Pin Configuration
#define PIN_RPM         19
#define PIN_GEAR_POS    5
#define PIN_MOTOR_OUT   7
#define PIN_PEDAL_POS   8
#define PIN_GAS_MODE    9
#define PIN_ELC_MODE    10

// Constants
#define MIN_HYBRID_PEDAL_POS  10
#define ICE_FINAL_DRIVE_RATIO 3.667
#define EM_FINAL_DRIVE_RATIO  2.667
#define PRIMARY_RATIO         3.666
#define EM_SLOPE              0.0647
#define EM_OFFSET             6.8341
#define K_FACTOR              0
const double GEAR_RATIOS[] = {0, 2.00, 1.611, 1.333, 1.086, 0.920, 0.814};

double differentialRpm, emRpm, pwmOut = 0;
int currentGear, pedalPos;
unsigned int engineRpm;
unsigned long timeOld;
volatile int rpmCount;

void rpm_count()
{
  rpmCount++;
}

void setup()
{

#ifdef VERBOSE
  Serial.begin(9600);
#endif

  pinMode(PIN_MOTOR_OUT, OUTPUT);
  pinMode(PIN_GEAR_POS, INPUT);
  pinMode(PIN_PEDAL_POS, INPUT);
  pinMode(PIN_ELC_MODE, INPUT);
  pinMode(PIN_GAS_MODE, INPUT);
  digitalWrite(PIN_ELC_MODE, HIGH);
  digitalWrite(PIN_GAS_MODE, HIGH);

  attachInterrupt(4, rpm_count, FALLING);

  rpmCount = 0;
  engineRpm = 0;
  timeOld = 0;
}

int GetGearPosition(int val)
{
  if (val > 44 && val < 80)         // Neutral
    return 0;
  else if (val > 80 && val < 116)   // 1st gear
    return 1;
  else if (val > 126 && val < 162)  // 2nd gear
    return 2;
  else if (val > 167 && val < 203)  // 3rd gear
    return 3;
  else if (val > 205 && val < 241)  // 4th gear
    return 4;
  else if (val > 243 && val < 279)  // 5th gear
    return 5;
  else if (val > 279 && val < 312)  // 6th gear
    return 6;
  else                              // bad reading
    return -1;
}

void UpdateEngineRpm()
{
  detachInterrupt(4);
  int deltaT = millis() - timeOld;
  engineRpm = 30 * 1000 / deltaT * rpmCount;

#ifdef RPMDEBUG
  Serial.print("RPM: ");
  Serial.print(engineRpm);
  Serial.print(", Pulse Count: ");
  Serial.print(rpmCount);
  Serial.print(", Delta T: ");
  Serial.println(deltaT);
#endif

  timeOld = millis();
  rpmCount = 0;
  attachInterrupt(4, rpm_count, FALLING);
}

Mode GetHybridMode()
{
  if (digitalRead(PIN_GAS_MODE) == LOW)
    return Gas;
  else if (digitalRead(PIN_ELC_MODE) == LOW)
    return Electric;
  else
    return Hybrid;
}

void SetMotorPower()
{
  switch(GetHybridMode())
  {
    case Gas:
      // Keep the motor turned off
      analogWrite(PIN_MOTOR_OUT, 0);
      break;
    case Electric:
      // Use only the pedal position
      analogWrite(PIN_MOTOR_OUT, pedalPos);
      break;
    case Hybrid:
      if (pedalPos > MIN_HYBRID_PEDAL_POS)
      {
        // If in actual gear, compute required motor RPM
        if (currentGear > 0)
        {
          // Calculate Differential RPM
          differentialRpm = engineRpm /
            ( PRIMARY_RATIO * GEAR_RATIOS[currentGear] * ICE_FINAL_DRIVE_RATIO);

          // Calculate needed electic motor RPM
          emRpm = differentialRpm / EM_FINAL_DRIVE_RATIO;

          // Calculate PWM output
          pwmOut = (emRpm * EM_SLOPE) + EM_OFFSET + K_FACTOR;
        }

        // Turn motor off when in neutral or bad gear read of gear postion
        else
        {
          pwmOut = 0;
        }

        analogWrite(PIN_MOTOR_OUT, pwmOut);
      }
      else
      {
        // Turn off motor to regenerate power
        analogWrite(PIN_MOTOR_OUT, 0);
      }
      break;
  }
}

#ifdef VERBOSE
void PrintValues()
{
  Serial.print("Pedal: ");
  Serial.print(pedalPos);
  Serial.print(", Eng: ");
  Serial.print(engineRpm);
  Serial.print(", Diff: ");
  Serial.print(differentialRpm);
  Serial.print(", Mot: ");
  Serial.print(emRpm);
  Serial.print(", Out: ");
  Serial.println(pwmOut);
}
#endif

void loop()
{
  // Update current gear
  currentGear = GetGearPosition(analogRead(PIN_GEAR_POS));

  // Update current pedal position
  pedalPos = map(analogRead(PIN_PEDAL_POS), 75, 623, 0, 255);

  // Update current engine RPM
  UpdateEngineRpm();

  // Update the motor output based on current inputs
  SetMotorPower();

#ifdef VERBOSE
  PrintValues();
#endif
}
