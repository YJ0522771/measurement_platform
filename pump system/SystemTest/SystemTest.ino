#include <SD.h>
#include <SPI.h>
/*
 Using Arduino Mega 2560
 Need to set some parameters : FILE_NAME, TOTAL_CYCLES, MAX_VOL
 
*/

////////////////////////////// NEED TO SET PARAMETERS //////////////////////////////
#define FILE_NAME "test2.txt"                                       // 저장 파일 이름
#define TOTAL_CYCLES 3                                              // 반복 횟수
#define MAX_VOL 500                                                 // 최대 부피 (mL)
#define INTERVAL 50                                                 // 측정 단위 (mL)
////////////////////////////////////////////////////////////////////////////////////

#define IN 70
#define OUT 71

#define RELAY1 28
#define RELAY2 29
//릴레이 모듈 > LOW : NO에 연결, HIGH : NC에 연결, 평상시 : NC에 연결
#define LED_RED 24
#define LED_GREEN 25

#define FLOW_IN 22
#define FLOW_OUT 23

#define SD_MISO 50
#define SD_MOSI 51 
#define SD_SCK 52
#define SD_CS 47

uint8_t State = IN;
uint8_t Cycles = 0;
float TotalVolume = 0;
float LastVolume = 0;
uint8_t DetectingPoints = MAX_VOL / INTERVAL;
uint8_t NextPoint = 0;

//File sdFile;

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
/*
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
  */

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(FLOW_IN, INPUT);
  pinMode(FLOW_OUT, INPUT);  
  
  digitalWrite(LED_RED, HIGH); 
  
  digitalWrite(RELAY1, HIGH);    //NC 연결
  digitalWrite(RELAY2, HIGH);

   delay(5*1000);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);    // 초록 LED를 켜서 외부전원을 ON 시켜도 된다는 신호를 줌

}

void loop() {
  // put your main code here, to run repeatedly:
  
  while(true)
  {
    

    if(Cycles == 0 && State == IN)
    {
      digitalWrite(FLOW_IN, HIGH); 
      lastflowpinstate = digitalRead(FLOW_IN); 
      useInterrupt(true);
      
      digitalWrite(RELAY2, LOW);
      //Serial.print("relay1 : ");  Serial.println(LOW); 
      //delay(10000);
    }

    //delay(10000);
    //digitalWrite(RELAY1, HIGH);
    //break;

    if(State == IN && TotalVolume >= MAX_VOL)             // 유입 > 배출 전환
    {
      delay(3000);
      digitalWrite(RELAY1, HIGH);
      //Serial.print("relay1 : ");  Serial.println(HIGH);
      digitalWrite(RELAY2, LOW);
      //Serial.print("relay2 : ");  Serial.println(LOW);
      //delay(10000);

      digitalWrite(FLOW_IN, LOW); 
      digitalWrite(FLOW_OUT, HIGH); 
      lastflowpinstate = digitalRead(FLOW_OUT);
      useInterrupt(true);

      State = OUT;
      LastVolume = TotalVolume;
      pulses = 0;
    }

    if(State == OUT && TotalVolume <= 0.0)                // 배출 > 유입 전환, 혹시 몰라 0으로 안 둠
    {
      delay(3000);
      digitalWrite(RELAY2, HIGH);
      //Serial.print("relay2 : ");  Serial.println(HIGH);
      digitalWrite(RELAY1, LOW);

      digitalWrite(FLOW_OUT, LOW); 
      digitalWrite(FLOW_IN, HIGH); 
      lastflowpinstate = digitalRead(FLOW_IN);
      useInterrupt(true);

      State = IN;
      LastVolume = TotalVolume;
      pulses = 0;
      Cycles++;
    }

    if(Cycles >= TOTAL_CYCLES)                            // 목표했던 사이클을 다 채웠을 때
    {
      digitalWrite(RELAY1, HIGH);                         // 모두 off
      digitalWrite(RELAY2, HIGH);
      digitalWrite(LED_GREEN, LOW);       
      //sdFile.close();
      
      break;                                              // loop 탈출
    }

    delay(100);
    
    float liters = pulses;
    liters /= 7.5;  liters /= 60.0;
    
    if(State == IN)
    {
      liters /= 10.345;                                     // 값 보정
      TotalVolume = LastVolume + (liters * 1000.0);

      if(NextPoint * INTERVAL <= TotalVolume)             // 측정값 저장
      {
        //Serial.print(Cycles); Serial.print(" cycles \t");
        //Serial.print(TotalVolume); Serial.println(" mL");
        //sdFile.print(Cycles); sdFile.print(" cycles \t");
        //sdFile.print(TotalVolume); sdFile.println(" mL");

        if(NextPoint < DetectingPoints)
        {
          NextPoint++;
        }
      }
    }
    else if(State == OUT)
    {
      liters /= 10.345;                                     // 값 보정
      TotalVolume = LastVolume - (liters * 1000.0);

      if(NextPoint * INTERVAL >= TotalVolume)             // 측정값 저장
      {
        //Serial.print(Cycles); Serial.print(" cycles \t");
        //Serial.print(TotalVolume); Serial.println(" mL");
        //sdFile.print(Cycles); sdFile.print(" cycles \t");
        //sdFile.print(TotalVolume); sdFile.println(" mL");

        if(NextPoint > 0)
        {
          NextPoint--;
        }
      }
    }

    
    
    //Serial.print(liters * 1000.0); Serial.println(" mL");
    Serial.print(Cycles); Serial.print(" cycles \t");
    Serial.print(TotalVolume); Serial.println(" mL");
    //Serial.println(NextPoint);
    
  }

  while(true)
  {
    digitalWrite(LED_RED, HIGH);                          // 끝났음을 알리기 위해 빨간 LED 점멸
    delay(1000);
    digitalWrite(LED_RED, LOW);
    delay(1000);
  }

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
