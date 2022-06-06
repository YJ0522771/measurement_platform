#include <math.h>
#include <SD.h>
#include <SPI.h>
/*
 Using Arduino Uno
 Need to set some parameters : FILE_NAME, TOTAL_CYCLES, LENGTH_0, MAX_STRAIN, STRAIN_INTERVAL, R_D
 
*/

////////////////////////////// NEED TO SET PARAMETERS //////////////////////////////
#define FILE_NAME "07221#1B.txt"                                    // 저장 파일 이름 
#define TITLE "20722-1 Sample #1 (AuCNT, 20% interval)"                     // 파일 첫 줄에 들어갈 문구                      
#define TOTAL_CYCLES 12                                             // 반복 횟수
#define LENGTH_0 20.0                                               // 처음 길이 (mm)
#define MAX_STRAIN 3.0                                              // 최대 변형 (L/L0)
#define STRAIN_INTERVAL 0.2                                         // 측정 단위 ()
#define R_D 100000                                                  // Detect Resistance
////////////////////////////////////////////////////////////////////////////////////

#define DRIVER_IN1 9
#define DRIVER_IN2 8
#define DRIVER_ENA 10

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

  pinMode(DRIVER_IN1, OUTPUT);
  pinMode(DRIVER_IN2, OUTPUT);
  pinMode(DRIVER_ENA, OUTPUT);
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

  digitalWrite(DRIVER_IN1, LOW);       // off 상태
  digitalWrite(DRIVER_IN2, LOW);
  analogWrite(DRIVER_ENA, 200);

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
      digitalWrite(DRIVER_IN2, HIGH);
      //digitalWrite(DRIVER_IN1, HIGH);
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
    //Serial.print(Distance); Serial.println(" V");

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
      digitalWrite(DRIVER_IN2, LOW);
      delay(100);
      digitalWrite(DRIVER_IN1, HIGH);   //NO 연결
  
      State = UNLOAD;
      //Serial.println("UNLOAD");
    }
    else if(State == UNLOAD && Distance <= LENGTH_0)
    {
      digitalWrite(DRIVER_IN1, LOW);    //2번부터
      delay(100);
      digitalWrite(DRIVER_IN2, HIGH);    //NC 연결
    
      State = LOAD;
      //Serial.println("LOAD");
      Cycles++;
    }

    if(Cycles >= TOTAL_CYCLES)                            // 목표했던 사이클을 다 채웠을 때
    {
      digitalWrite(DRIVER_IN1, LOW);                          // 모터 양단을 전원의 COM에 연결 (off)
      digitalWrite(DRIVER_IN2, LOW); 
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
  
  res = (-6.9864 * pow(vol, 4)) + (64.246 * pow(vol, 3)) - (230.04 * pow(vol, 2)) + (408.56 * vol) - 238.34;    // 직접 측정 (4차)

  return res;
}

uint32_t VolToRes(float vol)
{
  
  vol *= 0.9927;
  vol -= 0.0464;
  
  return (R_D * ((INPUT_VOLT - vol) / vol));
}
