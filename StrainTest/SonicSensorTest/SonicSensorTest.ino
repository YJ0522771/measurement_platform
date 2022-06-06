#define LED_RED 4             // 상태 알림 LED
#define LED_GREEN 5

#define RELAY1 3
#define RELAY2 2
//릴레이 모듈 > LOW : NO에 연결, HIGH : NC에 연결, 평상시 : NC에 연결

#define TRIG 8                // 초음파센서
#define ECHO 7

float distance = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //pinMode(LED_RED, OUTPUT);
  //pinMode(LED_GREEN, OUTPUT);
  //pinMode(RELAY1, OUTPUT);
  //pinMode(RELAY2, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  //digitalWrite(LED_RED, HIGH);

  //delay(5*1000);

  //digitalWrite(RELAY1, HIGH);    // 릴레이 NC 연결
  //digitalWrite(RELAY2, HIGH);

  

}

void loop() {
  // put your main code here, to run repeatedly:
  //digitalWrite(TRIG, HIGH);
  //delayMicroseconds(10);
  //digitalWrite(ECHO, LOW); 

  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  distance = (pulseIn(ECHO, HIGH) * 0.17);     // mm로 변환 (12 mm 보정)
  Serial.print(distance); Serial.println(" mm");

  delay(500);

}
