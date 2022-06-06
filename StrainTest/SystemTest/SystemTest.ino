#include <math.h>

////////////////////////////// NEED TO SET PARAMETERS //////////////////////////////
#define FILE_NAME "test2.txt"                                       // 저장 파일 이름
#define TOTAL_CYCLES 2                                              // 반복 횟수
#define LENGTH_0 20.0                                               // 처음 길이 (mm)
#define MAX_STRAIN 3.0                                              // 최대 변형 (L/L0)
#define STRAIN_INTERVAL 0.1                                         // 측정 단위 ()
////////////////////////////////////////////////////////////////////////////////////

#define RELAY1 3
#define RELAY2 2
//릴레이 모듈 > LOW : NC에 연결, HIGH : NO에 연결
#define LED_RED 5
#define LED_GREEN 6

#define IR_SENSOR A2
#define IR_N 100

#define LOAD 24     // LOW
#define UNLOAD 25

float VoltoDis(float vol);

uint8_t Cycles = 0;
uint8_t State = LOAD;
float MaxLength = LENGTH_0 * MAX_STRAIN;
float Interval = LENGTH_0 * STRAIN_INTERVAL;
uint8_t DetectingPoints = MAX_STRAIN / STRAIN_INTERVAL;
uint8_t NextPoint = 0;
uint32_t IrInput;
float IrVol;
float Distance;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  digitalWrite(LED_RED, HIGH); 

  delay(5*1000);
/*
  //////////////////////// L0 로 이동 ////////////////////////
  Distance = 0;
  for(uint16_t i = 0; i < IR_N; i++)
  {
    IrInput = analogRead(IR_SENSOR);
    IrVol = (IrInput / 1023.0) * 5.0;
    Distance += VoltoDis(IrVol);
  }
  Distance /= IR_N;
  
  if(Distance > LENGTH_0 + 1.0 || Distance < LENGTH_0 - 1.0)
  {
    if(Distance <= LENGTH_0)
    {
      digitalWrite(RELAY1, HIGH);       //NC 연결
      digitalWrite(RELAY2, HIGH);

      Serial.println("Loading");
    }
    else
    {
      State = UNLOAD;

      digitalWrite(RELAY2, LOW);
      digitalWrite(RELAY1, LOW);       //NO 연결

      Serial.println("Unloading");
    }
  
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);    // 초록 LED를 켜서 외부전원을 ON 시켜도 된다는 신호를 줌
  
    while(true)
    {
      Distance = 0;
      for(uint16_t i = 0; i < IR_N; i++)
      {
        IrInput = analogRead(IR_SENSOR);
        IrVol = (IrInput / 1023.0) * 5.0;
        Distance += VoltoDis(IrVol);
      }
      Distance /= IR_N;
      //Serial.print(Distance); Serial.println(" V");
      Serial.print("Set up "); Serial.print(Distance); Serial.println(" mm");
  
      if(State == LOAD && Distance >= LENGTH_0)
      {
        digitalWrite(RELAY1, LOW);
        break;
      }
      else if(State == UNLOAD && Distance <= LENGTH_0)
      {
        digitalWrite(RELAY2, HIGH);
        break;
      }
      delay(50);
    }
    digitalWrite(LED_GREEN, LOW); 
    digitalWrite(LED_RED, HIGH);
  
    delay(10000);
  }

  State = LOAD;
  */
  digitalWrite(RELAY1, HIGH);       // off 상태
  digitalWrite(RELAY2, LOW);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);    // 초록 LED를 켜서 외부전원을 ON 시켜도 된다는 신호를 줌
  
}

void loop() {
  while(Cycles < TOTAL_CYCLES)
  {
    if(Cycles == 0 && State == LOAD)
    {
      digitalWrite(RELAY1, HIGH);       
      digitalWrite(RELAY2, HIGH);
    }
    
    Distance = 0;
    for(uint16_t i = 0; i < IR_N; i++)
    {
      IrInput = analogRead(IR_SENSOR);
      IrVol = (IrInput / 1023.0) * 5.0;
      Distance += VoltoDis(IrVol);
    }
    Distance /= IR_N;

    if(State == LOAD && (NextPoint * Interval <= Distance - LENGTH_0))
    {
      //Serial.print(Distance); Serial.println(" mm");
      if(NextPoint < DetectingPoints) { NextPoint++; }
    }
    else if(State == UNLOAD && (NextPoint * Interval >= Distance - LENGTH_0))
    {
      //Serial.print(Distance); Serial.println(" mm");
      if(NextPoint > 0) { NextPoint--; }
    }
  
    Serial.print(State); Serial.print(Distance); Serial.println(" mm");
    
    if(State == LOAD && Distance >= MaxLength)
    {
      digitalWrite(RELAY2, LOW);
      digitalWrite(RELAY1, LOW);   //NO 연결
  
      State = UNLOAD;
    }
    else if(State == UNLOAD && Distance <= LENGTH_0)
    {
      digitalWrite(RELAY1, HIGH);    //2번부터
      digitalWrite(RELAY2, HIGH);    //NC 연결
    
      State = LOAD;
      Cycles++;
    }

    if(Cycles >= TOTAL_CYCLES)                            // 목표했던 사이클을 다 채웠을 때
    {
      digitalWrite(RELAY2, LOW);                         // 모터 양단을 전원의 COM에 연결 (off)
      digitalWrite(LED_GREEN, LOW);       
      //sdFile.close();
      
      break;                                              // loop 탈출
    }

    delay(50);
  }

  digitalWrite(LED_RED, HIGH);                            // 끝났음을 알리기 위해 빨간 LED 점멸
  delay(1000);
  digitalWrite(LED_RED, LOW);
  delay(1000);
}

float VoltoDis(float vol)
{
  float res;
  if(State == LOAD)
  {
    res = (-16.358 * pow(vol, 4)) + (135.89 * pow(vol, 3)) - (431.25 * pow(vol, 2)) + (653.74 * vol) - 342.92;    // 직접 측정 (4차)
    res -= (100.0 - res) / 20.0;
  }
  else if(State == UNLOAD)
  {
    res = (-11.405 * pow(vol, 4)) + (103.11 * pow(vol, 3)) - (358.39 * pow(vol, 2)) + (599.54 * vol) - 349.77;    // 직접 측정 (4차)
    res -= (100.0 - res) / 20.0;
  }

  //if(res > 27.0) { res -= 1.0; }

  return res;
}
