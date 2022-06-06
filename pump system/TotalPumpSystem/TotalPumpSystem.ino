#include <SD.h>
#include <SPI.h>
/*
 Using Arduino Mega 2560
 Need to set some parameters : FILE_NAME, TOTAL_CYCLES, MAX_VOL, INTERVAL, R_D, SENSOR_CH
 
*/

////////////////////////////// NEED TO SET PARAMETERS //////////////////////////////
#define FILE_NAME "test1116.txt"                                       // 저장 파일 이름
#define TITLE "201110-1 Sample #4 Volume test"                     // 파일 첫 줄에 들어갈 문구  
#define TOTAL_CYCLES 5                                              // 반복 횟수
#define MAX_VOL 300                                                 // 최대 부피 (mL)
#define INTERVAL 30                                                 // 측정 단위 (mL)
#define R_D 100000                                                  // Detect Resistance
#define SENSOR_CH 3                                                 // 센서 채널 수
////////////////////////////////////////////////////////////////////////////////////

#define IN 70
#define OUT 71

#define RELAY1 28                     // 유입 시 LOW
#define RELAY2 29                     // 배출 시 LOW
//릴레이 모듈 > LOW : NO에 연결, HIGH : NC에 연결, 평상시 : NC에 연결
#define LED_RED 24
#define LED_GREEN 25
#define TRIG A4                     // setup에서 loop로 넘어가기 위한 트리거 (파워서플라이 on)

#define FLOW_IN 22                    // 유량 센서 입력
#define FLOW_OUT 23

#define INPUT_VOLT 5.0    
#define R_N 200

/*  굳이 필요 없음
#define SD_MISO 50
#define SD_MOSI 51 
#define SD_SCK 52
#define SD_CS 47
*/

uint8_t State = IN;
uint8_t Cycles = 0;
float TotalVolume = 0;
float LastVolume = 0;
uint8_t DetectingPoints = MAX_VOL / INTERVAL;
uint8_t NextPoint = 0;

uint8_t DetectCh[SENSOR_CH];
uint16_t DetectInput[SENSOR_CH];
float DetectVol[SENSOR_CH];
uint32_t SensorR0[SENSOR_CH];
uint32_t SensorR[SENSOR_CH];

File sdFile;

//////////////////////////////////////////////////// Flow Sensor Reference Code ////////////////////////////////////////////////////
volatile uint16_t pulses = 0;               // count how many pulses! 
volatile uint8_t lastflowpinstate;          // track the state of the pulse pin 
volatile uint32_t lastflowratetimer = 0;    // you can try to keep time of how long it is between pulses

SIGNAL(TIMER0_COMPA_vect) 
{ 
  uint8_t x;
  if(State == IN)
  {
    x = digitalRead(FLOW_IN); 
  }
  else if(State == OUT)
  {
    x = digitalRead(FLOW_OUT); 
  }
    
  if (x == lastflowpinstate)
  { 
    lastflowratetimer++; 
    return; // nothing changed! 
  } 
    
  if (x == HIGH) 
  { 
    //low to high transition! 
    pulses++; 
  } 
  lastflowpinstate = x;     
  lastflowratetimer = 0; 
} 

void useInterrupt(boolean v); 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t VolToRes(float vol);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);


////////////////////////////////// SD 카드 //////////////////////////////////
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
////////////////////////////////////////////////////////////////////////////

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(FLOW_IN, INPUT);
  pinMode(FLOW_OUT, INPUT);  
  
  digitalWrite(LED_RED, HIGH); 

  for(uint8_t i = 0; i < SENSOR_CH; i++)
  {
    DetectCh[i] = SENSOR_CH - 1 - i;                                        // 아날로그 핀 (2부터)
    /*
    //Initialization of R0
    DetectInput[i] = analogRead(DetectCh[i]);                 
    DetectVol[i] = (DetectInput[i] / 1023.0) * INPUT_VOLT;  // Voltage on R_D
    SensorR0[i] = VolToRes(DetectVol[i]);
    */
  }

  sdFile.println(TITLE);
  sdFile.println("Cycles\tVolume(mL)\tCh.1 R(kohm)\tCh.2 R(kohm)\tCh.3 R(kohm)\tCh.1 R/R0\tCh.2 R/R0\tCh.3 R/R0"); 

  delay(5*1000);
  
  digitalWrite(RELAY1, HIGH);    //NC 연결 (모두 off)
  digitalWrite(RELAY2, HIGH);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);    // 초록 LED를 켜서 외부전원을 ON 시켜도 된다는 신호를 줌

  while(analogRead(TRIG) < 1000) 
  { 
    //Serial.println(analogRead(TRIG)); 
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
  while(Cycles < TOTAL_CYCLES)
  {
    if(Cycles == 0 && State == IN)
    {
      digitalWrite(FLOW_IN, HIGH); 
      lastflowpinstate = digitalRead(FLOW_IN); 
      useInterrupt(true);
      
      digitalWrite(RELAY1, LOW);
      //digitalWrite(RELAY2, LOW);
    }
/*
    ///////////////////////////////////////////////////////////////////////////////// 저항 조절
    for(uint8_t i = 0; i < SENSOR_CH; i++)
    {
      SensorR[i] = 0;
      for(uint8_t j = 0; j < R_N; j++)
      {
        DetectInput[i] = analogRead(DetectCh[i]);                 
        DetectVol[i] = (DetectInput[i] / 1023.0) * INPUT_VOLT;  // Voltage on R_D
        SensorR[i] += VolToRes(DetectVol[i]);
      }
      SensorR[i] /= R_N;
      //DetectVol[i] /= R_N;

      Serial.print(SensorR[i] * 0.001); Serial.print(" kohm \t");
      //Serial.print(DetectVol[i]); Serial.print(" V \t");
    }
    Serial.println();
    
    /////////////////////////////////////////////////////////////////////////////////
    */
    
    if(State == IN && TotalVolume >= MAX_VOL)             // 유입 > 배출 전환
    {
      Serial.println("stop");
      delay(3000);
      digitalWrite(RELAY1, HIGH);
      digitalWrite(RELAY2, LOW);

      digitalWrite(FLOW_IN, LOW); 
      digitalWrite(FLOW_OUT, HIGH); 
      lastflowpinstate = digitalRead(FLOW_OUT);
      useInterrupt(true);

      State = OUT;
      LastVolume = TotalVolume;
      pulses = 0;
    }
    else if(State == OUT && TotalVolume <= 0.0)                // 배출 > 유입 전환, 혹시 몰라 0으로 안 둠
    {
      Serial.println("stop");
      delay(3000);
      digitalWrite(RELAY2, HIGH);
      digitalWrite(RELAY1, LOW);

      digitalWrite(FLOW_OUT, LOW); 
      digitalWrite(FLOW_IN, HIGH); 
      lastflowpinstate = digitalRead(FLOW_IN);
      useInterrupt(true);

      State = IN;
      LastVolume = TotalVolume;
      pulses = 0;
      Cycles++;

      if(Cycles >= TOTAL_CYCLES)                            // 목표했던 사이클을 다 채웠을 때
      {
        digitalWrite(RELAY1, HIGH);                         // 모두 off
        digitalWrite(RELAY2, HIGH);
        digitalWrite(LED_GREEN, LOW);       
        sdFile.close();                                             
      }

      Serial.println("end trig");
      while(analogRead(TRIG) < 1000) 
      { 
        //Serial.println(analogRead(TRIG)); 
      }
    }

    if(Cycles >= TOTAL_CYCLES)                            // 목표했던 사이클을 다 채웠을 때
    {
      break;                                              // loop 탈출
    }
    
     delay(50);
     
    float liters = pulses;
    liters /= 7.5;  liters /= 60.0;
    
    if(State == IN)
    {
      liters /= 10.345;                                     // 값 보정///////////////////////////////////////////////////////////////////////
      TotalVolume = LastVolume + (liters * 1000.0);

      if(NextPoint * INTERVAL <= TotalVolume)             // 측정값 저장
      {
        Serial.print(Cycles + 1); Serial.print(" cycles \t"); Serial.print(TotalVolume); Serial.print(" mL \t");
        sdFile.print(Cycles + 1); sdFile.print(" cycles \t"); sdFile.print(NextPoint * INTERVAL); sdFile.print(" mL \t");
        
        for(uint8_t i = 0; i < SENSOR_CH; i++)
        {
          SensorR[i] = 0;
          for(uint8_t j = 0; j < R_N; j++)
          {
            DetectInput[i] = analogRead(DetectCh[i]);                 
            DetectVol[i] = (DetectInput[i] / 1023.0) * INPUT_VOLT;  // Voltage on R_D
            SensorR[i] += VolToRes(DetectVol[i]);
          }
          SensorR[i] /= R_N;

          Serial.print(SensorR[i] * 0.001); Serial.print(" kohm \t");
          sdFile.print(SensorR[i] * 0.001); sdFile.print(" kohm \t");

          if(State == IN && NextPoint == 0) { SensorR0[i] = SensorR[i]; }
          
          Serial.print((float)SensorR[i] / SensorR0[i]); Serial.print("\t");
          sdFile.print((float)SensorR[i] / SensorR0[i]); sdFile.print("\t");
       }
      Serial.println(); sdFile.println();
      
        if(NextPoint < DetectingPoints)
        {
          NextPoint++;
        }
      }
    }
    else if(State == OUT)
    {
      liters /= 10.35;                                     // 값 보정////////////////////////////////////////////////////////////////////////////////
      TotalVolume = LastVolume - (liters * 1000.0);

      if(NextPoint * INTERVAL >= TotalVolume)             // 측정값 저장
      {
        Serial.print(Cycles + 1); Serial.print(" cycles \t"); Serial.print(TotalVolume); Serial.print(" mL \t");
        sdFile.print(Cycles + 1); sdFile.print(" cycles \t"); sdFile.print(NextPoint * INTERVAL); sdFile.print(" mL \t");
        
        for(uint8_t i = 0; i < SENSOR_CH; i++)
        {
          SensorR[i] = 0;
          for(uint8_t j = 0; j < R_N; j++)
          {
            DetectInput[i] = analogRead(DetectCh[i]);                 
            DetectVol[i] = (DetectInput[i] / 1023.0) * INPUT_VOLT;  // Voltage on R_D
            SensorR[i] += VolToRes(DetectVol[i]);
          }
          SensorR[i] /= R_N;

          Serial.print(SensorR[i] * 0.001); Serial.print(" kohm \t");
          sdFile.print(SensorR[i] * 0.001); sdFile.print(" kohm \t");

          if(State == IN && NextPoint == 0) { SensorR0[i] = SensorR[i]; }
          
          Serial.println((float)SensorR[i] / SensorR0[i]); Serial.print("\t");
          sdFile.println((float)SensorR[i] / SensorR0[i]); sdFile.print("\t");
       }
       Serial.println(); sdFile.println();

        if(NextPoint > 0)
        {
          NextPoint--;
        }
      }
    }

    //Serial.print(liters * 1000.0); Serial.println(" mL");
    //Serial.print(Cycles); Serial.print(" cycles \t");
    //Serial.print(TotalVolume); Serial.println(" mL");
    //Serial.println(NextPoint);
    
  }

  digitalWrite(LED_RED, HIGH);                          // 끝났음을 알리기 위해 빨간 LED 점멸
  delay(1000);
  digitalWrite(LED_RED, LOW);
  delay(1000);

}

void useInterrupt(boolean v) {                                                // Flow Sensor Reference Function
  if (v) { 
    // Timer0 is already used for millis() - we'll just interrupt somewhere 
    // in the middle and call the "Compare A" function above 
    OCR0A = 0xAF; 
    TIMSK0 |= _BV(OCIE0A); 
  } else { 
    // do not call the interrupt function COMPA anymore 
    TIMSK0 &= ~_BV(OCIE0A); 
  } 
} 


uint32_t VolToRes(float vol)
{
  vol *= 0.9089;                                      // 저항 calibration
  vol -= 0.0226;

  return (R_D * ((INPUT_VOLT - vol) / vol));
}
