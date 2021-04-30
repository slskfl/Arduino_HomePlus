#include <SoftwareSerial.h>//블루투스
#include <Servo.h>
#include <DHT.h>
#include <DHT_U.h>

SoftwareSerial BTSerial(2,3); //블루투스 모듈과 교차

/*LED와 RGB*/
#define led_pin 8 //LED배열
#define redPin 9 //RGBLED
#define greenPin 10 //GREENLED
#define bluePin 11 //BLUELED
/*정온식 온도 센서*/
#define flame A1
int flame_val;
/*가스 센서*/
#define gas A0
int gas_pre, gas_cur;
/*서모모터*/
#define TRIG 6
#define ECHO 5
int pos=0;// 서보모터 각도 변수
Servo myServo;
/*온습도*/
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE); //핀번호, 타입
int h,t; //습도, 섭씨온도, 화씨온도
/*미세먼지*/
#define DUST A2
#define v_led 12
#define no_dust 0.35 // 미세 먼지 없을 때 초기 V 값 0.35
float vo_value=0; // 센서로 읽은 값 변수 선언
float sensor_voltage=0; // 센서로 읽은 값을 전압으로 측정 변수
float dust_density=0; //실제 미세먼지 밀도 값

String buffer="";

void setup() {
  Serial.begin(9600); //시리얼 통신 초기화
  BTSerial.begin(9600); //블루투스 통신 초기화
  pinMode(led_pin,OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(flame, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(v_led,OUTPUT);
  //가스 초기값
  gas_pre=analogRead(gas); 
  //서보모터 
  myServo.attach(7);
  //온도습도 센서 초기화
  dht.begin(); 
}

void loop() {
  /*가스 센서 값 출력*/
  gas_cur= analogRead(gas);  // 값 읽어오기
  if(gas_cur != gas_pre){ //가변저항 값이 바뀐 경우에만 전송
    gas_pre=gas_cur;  //가변저항을 돌려 값이 변했을시 이전 값에 현재 값을 전송 시킨다.
    Serial.println(String("가스 값 = " )+ gas_cur);
    delay(3000);
  }
  /*정온식 센서 값 */
  flame_val=analogRead(flame);
  Serial.println(String("정온식 온도 값 = " )+flame_val);
  
  String flame_val_s=String(flame_val);
  String gas_cur_s=String(gas_cur);

  /*서보모터 함수 호출*/
  servoRun();

  /*온/습도 함수 호출*/
  dhtRun();
  Serial.println(String("날씨(온도):")+t);
  Serial.println(String("날씨(습도):")+h);

  /*미세먼지 함수 호출*/
  dustRun();
    
  /*앱으로 데이터 전달하기 (정온식 온도센서 + 가스센서 + 날씨온도 + 날씨습도)*/
  char value[14]="            \n"; //총 12글자 + \n
  value[0]='F';
  value[1]=flame_val_s.substring(0,1).toInt()+'0';
  value[2]=flame_val_s.substring(1,2).toInt()+'0';
  value[3]=flame_val_s.substring(3).toInt()+'0';
  value[4]='G';
  value[5]=gas_cur_s.substring(0,1).toInt()+'0';
  value[6]=gas_cur_s.substring(1,2).toInt()+'0';
  value[7]=gas_cur_s.substring(3).toInt()+'0';
  value[8]=(t/10)+'0';
  value[9]=(t%10)+'0';
  value[10]=(h/10)+'0';
  value[11]=(h%10)+'0';
  
  Serial.println(String("온도 첫 글자")+(t/10));
  BTSerial.print(value); //온도값+가스값 앱으로 전달

  if(BTSerial.available()) { //수신 데이터가 존재 하는지 여부
     //데이터 읽어오기
     readData();
  }
}
void readData(){
   char data=BTSerial.read();// 데이터 값 읽기 보내기(안드로이드)
      if(data=='\n') {
        int state=buffer.substring(1).toInt(); 
        Serial.println(String("들어온 값")+state);
        switch(buffer[0]){
          case 'L': //거실 등
            digitalWrite(led_pin, state);
            break;
          case 'R': //RED led
            analogWrite(redPin, state);
            break;
          case 'G': //GREEN led 
            analogWrite(greenPin, state);
            break;
          case 'B': //BLUE led
            analogWrite(bluePin, state);
            break;
          case 'C':
            switch(buffer[1]){
                 switch(buffer[0]){
                  case 'H': //온습도
                    break; 
                  case 'T': //날짜 정보
                   break;
                  case 'D': //미세먼지 정보
                   break;  
                  case 'U': //사용자 입력 정보
                    break; 
                }
                break;
              }
            break;
       }
      buffer=""; 
      }else {
        buffer+=(char)data; //개행문자가 아닌 경우는 수신 버퍼에 저장
      }
}

/*서보모터 동작*/
void servoRun(){
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long distance=pulseIn(ECHO, HIGH)/58.2;
  Serial.println(distance+String("cm"));
  if(distance<10){
    if(pos!=120){ //for문의 무한 반복 방지
      for(pos=0; pos<120; pos++){
        myServo.write(pos);
        delay(15);
      }
    }
  } else{
    delay(500); //통과를 위해 차단기 닫힘을 2초 지연, 보통 뒤에도 초음파 센서를 작창하여 사용
    if(pos!=0){ //for문의 무한 반복 방지
      for(pos=120; pos>0; pos--){
        myServo.write(pos);
        delay(15);
      }
    }
  }
  delay(100);
}

/*온/습도 센서*/
void dhtRun(){
  delay(1000);
  h=dht.readHumidity(); //습도를 측정
  t=dht.readTemperature(); //섭씨 온도를 측정
  if(isnan(h) || isnan(t)){
    Serial.println("온도 습도 센서 작동 오류");
    return;
  }
}

/*미세먼지 센서 */
void dustRun(){
  // 미세 먼지 센서 동작
 digitalWrite(v_led,LOW); // 적외선 LED ON
 delayMicroseconds(280); // 280us동안 딜레이
 vo_value=analogRead(DUST); // 데이터를 읽음
 delayMicroseconds(40); // 320us - 280us
 digitalWrite(v_led,HIGH); // 적외선 LED OFF
 delayMicroseconds(9680); // 10ms(주기) -320us(펄스 폭) 한 값

 sensor_voltage=get_voltage(vo_value);
 dust_density=get_dust_density(sensor_voltage);

 Serial.print("value = ");
 Serial.println(vo_value);
 Serial.print("Voltage = ");
 Serial.print(sensor_voltage);
 Serial.println(" [V]");
 Serial.print("Dust Density = ");
 Serial.print(dust_density);
 Serial.println(" [ug/m^3]");
  
 delay(1000);
}
float get_voltage(float value){
 // 아날로그 값을 전압 값으로 바꿈
 float V= value * 5.0 / 1024; 
 return V;
}

float get_dust_density(float voltage){
 // 데이터 시트에 있는 미세 먼지 농도(ug) 공식 기준
 float dust=(voltage-no_dust) / 0.005;
 return dust;
}
