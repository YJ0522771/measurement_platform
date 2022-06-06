#include <math.h>
#include <SD.h>
#include <SPI.h>
/*
 Using Arduino Uno
 Need to set some parameters : FILE_NAME, TOTAL_CYCLES, LENGTH_0, MAX_STRAIN, STRAIN_INTERVAL, R_D
 
*/

////////////////////////////// NEED TO SET PARAMETERS //////////////////////////////
#define FILE_NAME "11102#1A.txt"                                    // 저장 파일 이름 
#define TITLE "201110-2 Sample #1 (AuCNT test, CV)"                     // 파일 첫 줄에 들어갈 문구                      
#define TOTAL_CYCLES 40                                             // 반복 횟수
#define LENGTH_0 20.0                                               // 처음 길이 (mm)
#define MAX_STRAIN 3.0                                              // 최대 변형 (L/L0)
#define STRAIN_INTERVAL 0.2                                         // 측정 단위 ()
#define R_D 100000                                                  // Detect Resistance
////////////////////////////////////////////////////////////////////////////////////

#define RELAY1 9
#define RELAY2 8
//릴레이 모듈 > LOW : NC에 연결, HIGH : NO에 연결
#define LED_RED 5
#define LED_GREEN 6
#define SIGNAL A3

#define IR_SENSOR A2
#define IR_N 100

#define INPUT_VOLT 5.0
#define R_DETECT A0          //Analog Input
#define R_N 200

#define LOAD 24       // LOW
#define UNLOAD 25

/*  굳이 필요 없음
#define SD_MOSI 11 
#define SD_MISO 12 
#define SD_SCK 13
#define SD_CS 4
*/

float VoltoDis(float vol);
uint32_t VolToRes(float vol);

uint8_t Cycles = 0;
uint8_t State = LOAD;
float MaxLength = LENGTH_0 * MAX_STRAIN;
float Interval = LENGTH_0 * STRAIN_INTERVAL;
uint8_t DetectingPoints = (MAX_STRAIN - 1.0) / STRAIN_INTERVAL;
uint8_t NextPoint = 0;

uint32_t IrInput;
float IrVol;
float Distance;

uint16_t DetectInput;
float DetectVol;
uint32_t SensorR0;
uint32_t SensorR;

File sdFile;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  if(!SD.begin(4))
  {
    Serial.println("Failure");
    while(1);
  }
  Serial.println("Success");

  sdFile = SD.open(FILE_NAME, FILE_WRITE);
  if(!sdFile)
  {
    Serial.println("Error to open file");
    while(1);
  }

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  digitalWrite(LED_RED, HIGH); 

/*
  ////////////////// Initialization of R0 //////////////////
  for(uint16_t i = 0; i < R_N; i++)
  {
    DetectInput = analogRead(R_DETECT);
    DetectVol = (DetectInput / 1023.0) * INPUT_VOLT;  //Voltage on R_D
    SensorR0 += VolToRes(DetectVol);
  }
  SensorR0 /= R_N;

  Serial.print("Initail R0 = "); Serial.println(SensorR0);
  */

  sdFile.println(TITLE);
  sdFile.println("Cycles\tStrain(%)\tR(kohm)\tR/R0"); 

  digitalWrite(RELAY1, HIGH);       // off 상태
  digitalWrite(RELAY2, LOW);

  delay(5*1000);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);    // 초록 LED를 켜서 외부전원을 ON 시켜도 된다는 신호를 줌

  while(analogRead(SIGNAL) < 1000) 
  { 
    //Serial.println(analogRead(SIGNAL)); 
  }
  
}

void loop() {
  while(Cycles < TOTAL_CYCLES)
  {
    if(Cycles == 0 && State == LOAD)
    {
      digitalWrite(RELAY1, HIGH);       
      digitalWrite(RELAY2, HIGH);
      //digitalWrite(RELAY2, LOW);
      //digitalWrite(RELAY1, LOW);   //NO 연결
    }
    
    Distance = 0;
    for(uint16_t i = 0; i < IR_N; i++)
    {
      IrInput = analogRead(IR_SENSOR);
      IrVol = (IrInput / 1023.0) * INPUT_VOLT;
      Distance += VoltoDis(IrVol);
      //Distance += IrVol;
    }
    Distance /= IR_N;

    if(State == LOAD && (NextPoint * Interval <= Distance - LENGTH_0))
    {
      SensorR = 0;
      for(uint16_t i = 0; i < R_N; i++)
      {
        DetectInput = analogRead(R_DETECT);
        DetectVol = (DetectInput / 1023.0) * INPUT_VOLT;  //Voltage on R_D
        SensorR += VolToRes(DetectVol);
      }
      SensorR /= R_N;

      if(NextPoint == 0) { SensorR0 = SensorR; }

      Serial.print(Cycles + 1); Serial.print(" cycles\t"); Serial.print(Distance); Serial.print(" mm\t"); Serial.print(NextPoint * STRAIN_INTERVAL * 100); Serial.print("%\t"); 
      Serial.print(DetectVol); Serial.print(" V\t"); Serial.print(SensorR * 0.001); Serial.print(" kohm\t"); Serial.println((float)SensorR / SensorR0); 
      sdFile.print(Cycles + 1); sdFile.print("\t"); sdFile.print(NextPoint * STRAIN_INTERVAL * 100); sdFile.print("\t"); sdFile.print(SensorR * 0.001); sdFile.print("\t"); sdFile.println((float)SensorR / SensorR0);  
      //Serial.print(Distance); Serial.println(" mm");
      if(NextPoint < DetectingPoints) { NextPoint++; }
    }
    else if(State == UNLOAD && (NextPoint * Interval >= Distance - LENGTH_0))
    {
      SensorR = 0;
      for(uint16_t i = 0; i < R_N; i++)
      {
        DetectInput = analogRead(R_DETECT);
        DetectVol = (DetectInput / 1023.0) * INPUT_VOLT;  //Voltage on R_D
        SensorR += VolToRes(DetectVol);
      }
      SensorR /= R_N;
      
      Serial.print(Cycles + 1); Serial.print(" cycles\t"); Serial.print(Distance); Serial.print(" mm\t"); Serial.print(NextPoint * STRAIN_INTERVAL * 100); Serial.print("%\t");
      Serial.print(DetectVol); Serial.print(" V\t"); Serial.print(SensorR * 0.001); Serial.print(" kohm\t"); Serial.println((float)SensorR / SensorR0); 
      sdFile.print(Cycles + 1); sdFile.print("\t"); sdFile.print(NextPoint * STRAIN_INTERVAL * 100); sdFile.print("\t"); sdFile.print(SensorR * 0.001); sdFile.print("\t"); sdFile.println((float)SensorR / SensorR0); 
      //Serial.print(Distance); Serial.println(" mm");
      if(NextPoint > 0) { NextPoint--; }
    }
    /*
    DetectVol = 0;
    SensorR = 0;
    for(uint16_t i = 0; i < R_N; i++)
    {
      DetectInput = analogRead(R_DETECT);
      DetectVol += (DetectInput / 1023.0) * INPUT_VOLT;  //Voltage on R_D
      //DetectVol += map(DetectInput, 0.0, 1023.0, 0.0, 5.0);
      SensorR += DetectInput;
    }
    DetectVol /= R_N;
    SensorR /= R_N;
    */
    //Serial.print(SensorR); Serial.print("\t"); Serial.print(DetectVol); Serial.print(" V\t"); Serial.print(VolToRes(DetectVol) * 0.001); Serial.println(" kohm\t");
    //Serial.print(Distance); Serial.println(" V");
    //Serial.print(Distance); Serial.println(" mm");
    
    if(State == LOAD && Distance >= MaxLength)
    {
      digitalWrite(RELAY2, LOW);
      digitalWrite(RELAY1, LOW);   //NO 연결
  
      State = UNLOAD;
      //Serial.println("UNLOAD");
    }
    else if(State == UNLOAD && Distance <= LENGTH_0)
    {
      digitalWrite(RELAY1, HIGH);    //2번부터
      digitalWrite(RELAY2, HIGH);    //NC 연결
    
      State = LOAD;
      //Serial.println("LOAD");
      Cycles++;
    }

    if(Cycles >= TOTAL_CYCLES)                            // 목표했던 사이클을 다 채웠을 때
    {
      digitalWrite(RELAY2, LOW);                          // 모터 양단을 전원의 COM에 연결 (off)
      digitalWrite(LED_GREEN, LOW);       
      sdFile.close();
      
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
    res = (-13.572 * pow(vol, 4)) + (115.27 * pow(vol, 3)) - (372.61 * pow(vol, 2)) + (576.34 * vol) - 307.37;    // 직접 측정 (4차)
  }
  else if(State == UNLOAD)
  {
    res = (-7.7105 * pow(vol, 4)) + (73.296 * pow(vol, 3)) - (267.28 * pow(vol, 2)) + (471.98 * vol) - 283.6;    // 직접 측정 (4차)
  }

  return res;
}

uint32_t VolToRes(float vol)
{
  
  if(State == LOAD)
  {
    vol *= 0.98;
  }
  else if(State == UNLOAD)
  {
    vol *= 0.9;
  }
  
  return (R_D * ((INPUT_VOLT - vol) / vol));
}
