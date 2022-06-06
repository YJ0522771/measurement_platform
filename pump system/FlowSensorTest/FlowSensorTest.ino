/********************************************************** 
2 This is example code for using the Adafruit liquid flow meters.  
3  
4 Tested and works great with the Adafruit plastic and brass meters 
5     ------> http://www.adafruit.com/products/828 
6     ------> http://www.adafruit.com/products/833 
7  
8 Connect the red wire to +5V,  
9 the black wire to common ground  
10 and the yellow sensor wire to pin #2 
11  
12 Adafruit invests time and resources providing this open source code,  
13 please support Adafruit and open-source hardware by purchasing  
14 products from Adafruit! 
15  
16 Written by Limor Fried/Ladyada  for Adafruit Industries.   
17 BSD license, check license.txt for more information 
18 All text above must be included in any redistribution 
19 **********************************************************/ 
 
// which pin to use for reading the sensor? can use any pin! 
#define FLOW_SENSOR 8 

////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile uint16_t pulses = 0;               // count how many pulses! 
volatile uint8_t lastflowpinstate;          // track the state of the pulse pin 
volatile uint32_t lastflowratetimer = 0;    // you can try to keep time of how long it is between pulses 
volatile float flowrate;                    // and use that to calculate a flow rate  
// Interrupt is called once a millisecond, looks for any pulses from the sensor! 
SIGNAL(TIMER0_COMPA_vect) 
{ 
  uint8_t x = digitalRead(FLOW_SENSOR); 
    
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
  lastflowpinstate = x;    flowrate = 1000.0; 
  flowrate /= lastflowratetimer;  // in hertz 
  lastflowratetimer = 0; 
} 
 
 
void useInterrupt(boolean v); 
 
void setup() { 
   Serial.begin(9600); 
   //Serial.println("Flow sensor test!"); 
  
   pinMode(FLOW_SENSOR, INPUT); 
   digitalWrite(FLOW_SENSOR, HIGH); 
   lastflowpinstate = digitalRead(FLOW_SENSOR); 
   useInterrupt(true); 
} 

void loop()                     // run over and over again 
{  
  //Serial.print("Freq: "); Serial.println(flowrate); 
  //Serial.print("Pulses: "); Serial.println(pulses, DEC); 
   
  // if a plastic sensor use the following calculation 
  // Sensor Frequency (Hz) = 7.5 * Q (Liters/min) (YF-S201)
  // Liters = Q * time elapsed (seconds) / 60 (seconds/minute) 
  // Liters = (Frequency (Pulses/second) / 7.5) * time elapsed (seconds) / 60 
  // Liters = Pulses / (7.5 * 60) 
  float liters = pulses;  
  //liters /= 5.5;                              // YF-S401 (Freq. = 5.5 * Q - 3)
  //liters -= 3;
  //liters /= 60.0;
  //liters *= 1000.0;
  //Serial.print(mililiters); Serial.print(" ");
  liters /= 7.5;                            
  liters /= 12.0;                         // 수치 보정 
  //Serial.print(mililiters); Serial.print(" ");
  liters /= 60.0; 
  //Serial.println(mililiters);

 
/* 
  // if a brass sensor use the following calculation 
  float liters = pulses; 
  liters /= 8.1; 
  liters -= 6; 
  liters /= 60.0; 
*/ 
  //Serial.print(liters); Serial.println(" Liters");
  Serial.print(liters * 1000.0); Serial.println(" Mililiters"); 
 
  delay(100); 
} 

void useInterrupt(boolean v) { 
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
