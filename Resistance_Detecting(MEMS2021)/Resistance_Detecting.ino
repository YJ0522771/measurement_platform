#include <math.h>

#define R_D 100000        //Detect Resistance

#define STRAIN_50 2.0
#define STRAIN_100 4.0
#define STRAIN_150 6.0
#define STRAIN_200 7.0    //R/R0 of sensor at 200% strain 

#define INPUT_VOLT 4.8    //Input Votage in Detecting Circuit ()

//Pin Number
#define LEDPIN_1 4
#define LEDPIN_2 5
#define LEDPIN_3 6
#define LEDPIN_4 7

#define DETECT 1          //Analog Input
//#define CIRCUIT_INPUT 52

uint32_t VolToRes(float vol);


uint16_t DetectInput;
float DetectVol;

uint32_t SensorR0;
uint32_t SensorR;

boolean State[4] = {false, false, false, false};
float Res[4];

void setup() {
  Serial.begin(9600);

  pinMode(LEDPIN_1, OUTPUT);
  pinMode(LEDPIN_2, OUTPUT);
  pinMode(LEDPIN_3, OUTPUT);
  pinMode(LEDPIN_4, OUTPUT);

  //pinMode(CIRCUIT_INPUT, OUTPUT);                   //Power On for Detecting Circuit
  //digitalWrite(CIRCUIT_INPUT, HIGH);

  delay(2000);

  //Initialization of R0
  DetectInput = analogRead(DETECT);
  DetectVol = (DetectInput / 1023.0) * INPUT_VOLT;  //Voltage on R_D

  SensorR0 = VolToRes(DetectVol);

  Serial.print("R0 = "); Serial.println(SensorR0);

  Res[0] = SensorR0 * STRAIN_50;                    //Threshold Resistance
  Res[1] = SensorR0 * STRAIN_100;
  Res[2] = SensorR0 * STRAIN_150;
  Res[3] = SensorR0 * STRAIN_200;

  Serial.print("R_50 = "); Serial.println(Res[0]);
  Serial.print("R_100 = "); Serial.println(Res[1]);
  Serial.print("R_150 = "); Serial.println(Res[2]);
  Serial.print("R_200 = "); Serial.println(Res[3]);
}

void loop() {

  DetectInput = analogRead(DETECT);
  DetectVol = (DetectInput / 1023.0) * INPUT_VOLT;  //Voltage on R_D

  SensorR = VolToRes(DetectVol);
  Serial.print("R = "); Serial.println(SensorR);
  //Serial.print("in = "); Serial.println(DetectInput);

  if (SensorR > Res[3] && !State[3])
  {
    State[3] = true;
    digitalWrite(LEDPIN_4, HIGH);
  }
  else if (SensorR < Res[3] && State[3])
  {
    State[3] = false;
    digitalWrite(LEDPIN_4, LOW);
  }

  if (SensorR > Res[2] && !State[2])
  {
    State[2] = true;
    digitalWrite(LEDPIN_3, HIGH);
  }
  else if (SensorR < Res[2] && State[2])
  {
    State[2] = false;
    digitalWrite(LEDPIN_3, LOW);
  }

  if (SensorR > Res[1] && !State[1])
  {
    State[1] = true;
    digitalWrite(LEDPIN_2, HIGH);
  }
  else if (SensorR < Res[1] && State[1])
  {
    State[1] = false;
    digitalWrite(LEDPIN_2, LOW);
  }

  if (SensorR > Res[0] && !State[0])
  {
    State[0] = true;
    digitalWrite(LEDPIN_1, HIGH);
  }
  else if (SensorR < Res[0] && State[0])
  {
    State[0] = false;
    digitalWrite(LEDPIN_1, LOW);
  }

  delay(500);


}

uint32_t VolToRes(float vol)
{
  return (R_D * ((INPUT_VOLT - vol) / vol));
}
