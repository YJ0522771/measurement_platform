/*
 Using Arduino Mega 2560
  
 */

#define RELAY1 22
#define RELAY2 23
//릴레이 모듈 > LOW : NO에 연결, HIGH : NC에 연결, 평상시 : NC에 연결
#define LED_RED 24
#define LED_GREEN 25

#define FLOW_IN 26
#define FLOW_OUT 27

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  digitalWrite(LED_RED, HIGH);

  delay(5*1000);
  
  digitalWrite(RELAY1, HIGH);    //NC 연결
  digitalWrite(RELAY2, HIGH);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);    // 초록 LED를 켜서 외부전원을 ON 시켜도 된다는 신호

}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(RELAY1, LOW);
  Serial.print("relay1 : ");  Serial.println(LOW); 
  delay(10000);
  
  digitalWrite(RELAY1, HIGH);
  Serial.print("relay1 : ");  Serial.println(HIGH);
  delay(1000);
  
  digitalWrite(RELAY2, LOW);
  Serial.print("relay2 : ");  Serial.println(LOW);
  delay(10000);
  
  digitalWrite(RELAY2, HIGH);
  Serial.print("relay2 : ");  Serial.println(HIGH);
  delay(1000);

}
