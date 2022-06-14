#include <math.h>

#define R_D 100000        //Detect Resistance

#define VOL_100 1.5
#define VOL_150 4.0
#define VOL_200 6.0    //R/R0 of sensor at 200% strain 

#define INPUT_VOLT 4.8    //Input Votage in Detecting Circuit ()

//Pin Number
#define LED_G 8
#define LED_Y 9
#define LED_R 10

#define DETECT A1          //Analog Input
#define R_N 200
//#define CIRCUIT_INPUT 52

uint32_t VolToRes(float vol);


uint16_t DetectInput;
float DetectVol;

uint32_t SensorR0;
uint32_t SensorR;

boolean State[3] = {false, false, false};
float Res[3];

void setup() {
  Serial.begin(9600);

  pinMode(LED_G, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_R, OUTPUT);

  //pinMode(CIRCUIT_INPUT, OUTPUT);                   //Power On for Detecting Circuit
  //digitalWrite(CIRCUIT_INPUT, HIGH);

  delay(2000);

  //Initialization of R0
  SensorR0 = 0;
  for(uint8_t j = 0; j < R_N; j++)
  {
    DetectInput = analogRead(DETECT);                 
    DetectVol = (DetectInput / 1023.0) * INPUT_VOLT;  // Voltage on R_D
    SensorR0 += VolToRes(DetectVol);
  }
  SensorR0 /= R_N;

  Serial.print("R0 = "); Serial.println(SensorR0);

  //Threshold Resistance
  Res[0] = SensorR0 * VOL_100;
  Res[1] = SensorR0 * VOL_150;
  Res[2] = SensorR0 * VOL_200;

  Serial.print("R_100 = "); Serial.println(Res[0]);
  Serial.print("R_150 = "); Serial.println(Res[1]);
  Serial.print("R_200 = "); Serial.println(Res[2]);
}

void loop() {
  
  SensorR = 0;
  for(uint8_t j = 0; j < R_N; j++)
  {
    DetectInput = analogRead(DETECT);                 
    DetectVol = (DetectInput / 1023.0) * INPUT_VOLT;  // Voltage on R_D
    SensorR += VolToRes(DetectVol);
  }
  SensorR /= R_N;

  if (SensorR > Res[2] && !State[2])
  {
    State[2] = true;
    digitalWrite(LED_R, HIGH);
  }
  else if (SensorR < Res[2] && State[2])
  {
    State[2] = false;
    digitalWrite(LED_R, LOW);
  }

  if (SensorR > Res[1] && !State[1])
  {
    State[1] = true;
    digitalWrite(LED_Y, HIGH);
  }
  else if (SensorR < Res[1] && State[1])
  {
    State[1] = false;
    digitalWrite(LED_Y, LOW);
  }

  if (SensorR > Res[0] && !State[0])
  {
    State[0] = true;
    digitalWrite(LED_G, HIGH);
  }
  else if (SensorR < Res[0] && State[0])
  {
    State[0] = false;
    digitalWrite(LED_G, LOW);
  }

  delay(500);


}

uint32_t VolToRes(float vol)
{
  return (R_D * ((INPUT_VOLT - vol) / vol));
}
