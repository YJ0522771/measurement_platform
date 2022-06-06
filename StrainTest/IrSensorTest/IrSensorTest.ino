#include <math.h>

#define IR_SENSOR A2
#define IR_N 100

float VoltoDis(float vol);

uint32_t IrInput;
float IrVol;
float Distance;

void setup() {
  Serial.begin(9600);
  
  delay(2000);

  // 평균 확인용
  /* 
  Distance = 0;
  for(uint32_t i = 0; i < IR_N; i++)
  {
    IrInput = analogRead(IR_SENSOR);
    IrVol = (IrInput / 1023.0) * 5.0;
    Distance += VoltoDis(IrVol);
  }
  Distance /= IR_N;

  Serial.print(Distance); Serial.println(" mm");
  */
}

void loop() {
  
  Distance = 0;
  for(uint16_t i = 0; i < IR_N; i++)
  {
    IrInput = analogRead(IR_SENSOR);
    IrVol = (IrInput / 1023.0) * 5.0;
    Distance += IrVol;
    //Distance += VoltoDis(IrVol);
  }
  Distance /= IR_N;
  //Distance = 132.0 - Distance;

  Serial.print(Distance); Serial.println(" V");
  //Serial.print(Distance); Serial.println(" mm");
  delay(500);
  
}

float VoltoDis(float vol)
{
  float res;
  
  //res = (0.0115 * pow(vol, 3)) - (0.0538 * pow(vol,2)) + (0.1596 * vol) - 0.0429;   // 1/(cm + 0.42)
  /*
  if(vol <= 1.25)
  {
    res = (0.05 * vol) + 0.0275;
  }
  else if(vol > 1.25 && vol <= 1.4)
  {
    res = (0.1333 * vol) - 0.0767; 
  }
  else if(vol > 1.4 && vol <= 1.75)
  {
    res = (-0.0476 * pow(vol, 2)) + (0.2071 * vol) - 0.0867;
  }
  else if(vol > 1.75 && vol <= 2.35)
  {
    res = (-0.0556 * pow(vol, 2)) + (0.3111 * vol) -0.2443;
  }
  else
  {
    res = (0.1223 * vol) - 0.1095;
  }
  
  res = 1/res - 0.42;
  res *= 10;
  */

  // res = (-23.171 * pow(vol, 2)) + (129.53 * vol) - 100.14;   // 직접 측정 (2차)
  // res = (18.213 * pow(vol, 3)) - (121.15 * pow(vol, 2)) + (299.37 * vol) - 194.81;    // 직접 측정 (3차)
  res = (-16.358 * pow(vol, 4)) + (135.89 * pow(vol, 3)) - (431.25 * pow(vol, 2)) + (653.74 * vol) - 342.92;    // 직접 측정 (4차)

  if(res > 27.0) { res -= 1.0; }

  return res;
}
