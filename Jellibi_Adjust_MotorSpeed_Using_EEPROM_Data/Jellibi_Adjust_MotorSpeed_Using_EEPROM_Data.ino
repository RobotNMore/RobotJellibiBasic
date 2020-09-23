#include <EEPROM.h>

#define pinBuzzer     3  // 부저 핀 번호

// 아래 젤리비 모델에 따라서 출발/정지 버튼으로 사용할 버튼을 지정
//#define pinButton     A3  // 젤리비 AGV는 왼쪽 측면 버튼 사용
#define pinButton     A0  // 젤리비 Macaron이나 Vision은 UP(위) 버튼

////////////////////  좌우 바퀴모터 속력 비율

// 전진 속력 보정비율 [0 .. 1], 대부분 0.94 ~ 1.00 사이로 설정됨
float Power1RatioF = 1.00;  // 왼쪽 모터
float Power2RatioF = 1.00;  // 오른쪽 모터

////////////////////  EEPROM 데이터 읽기

#define EEPROM_BASE           0
#define EEPROM_DATA_VER0      (EEPROM_BASE + 0)  // 'V'
#define EEPROM_DATA_VER1      (EEPROM_BASE + 1)  // '1'
#define EEPROM_DATA_VER2      (EEPROM_BASE + 2)  // '0'
#define EEPROM_DATA_VER3      (EEPROM_BASE + 3)  // '0'

#define EEPROM_PRODUCT_SN     (EEPROM_BASE + 4)  // Reserved
#define EEPROM_POWER_RATIOF   (EEPROM_BASE + 8)  // 1.00, 1.00
#define EEPROM_POWER_RATIOR   (EEPROM_BASE + 16) // Reserved
#define EEPROM_POWER_ADJUST   (EEPROM_BASE + 24) // Reserved
#define EEPROM_LT_THRESHOLD   (EEPROM_BASE + 28) // L=[..512..], R=[..512..]
#define EEPROM_SERVO_ADJUST   (EEPROM_BASE + 32) // Pan=[..512..], Tilt=[..512..]

boolean IsEepromDataValid = false;

boolean CheckEepromDataHeader()
{
  if( (EEPROM.read( EEPROM_DATA_VER0 ) == 'V') &&
      (EEPROM.read( EEPROM_DATA_VER1 ) == '1') &&
      (EEPROM.read( EEPROM_DATA_VER2 ) == '0') &&
      (EEPROM.read( EEPROM_DATA_VER3 ) == '0') )
      return  true;

  return  false;
}

void  ReadPowerRatio()  // [0.0 ~ 1.0] (최대 성능)
{
  EEPROM.get( EEPROM_POWER_RATIOF, Power1RatioF );
  EEPROM.get( EEPROM_POWER_RATIOF + 4, Power2RatioF );
}

////////////////////  모터 1번(왼쪽)과 2번(오른쪽)

#define pinDIR1   7 // 1번(왼쪽)모터 방향 지정용 연결 핀
#define pinPWM1   5 // 1번(왼쪽)모터 속력 지정용 연결 핀

#define pinDIR2   8 // 2번(오른쪽)모터 방향 지정용 연결 핀
#define pinPWM2   6 // 2번(오른쪽)모터 속력 지정용 연결 핀

////////////////////  모터 회전 동작

#define FORWARD   0 // 전진 방향
#define BACKWARD  1 // 후진 방향

void  drive(int dir1, int power1, int dir2, int power2)
{
  boolean dirHighLow1, dirHighLow2;
  int     p1, p2;

  if(dir1 == FORWARD)  // 1번(왼쪽)모터 방향
    dirHighLow1 = HIGH;
  else // BACKWARD
    dirHighLow1 = LOW;
  p1 = power1 * Power1RatioF;
  
  if(dir2 == FORWARD)  // 2번(오른쪽)모터
    dirHighLow2 = LOW;
  else // BACKWARD
    dirHighLow2 = HIGH;
  p2 = power2 * Power2RatioF;
  
  digitalWrite(pinDIR1, dirHighLow1);
  analogWrite(pinPWM1, p1);

  digitalWrite(pinDIR2, dirHighLow2);
  analogWrite(pinPWM2, p2);
}

void  Forward( int power )  // 전진
{
  drive(FORWARD, power, FORWARD, power);
}

void  Backward( int power )  // 후진
{
  drive(BACKWARD, power, BACKWARD, power);
}

void  TurnLeft( int power )  // 좌회전
{
  drive(BACKWARD, power, FORWARD, power);
}

void  TurnRight( int power )  // 우회전
{
  drive(FORWARD, power, BACKWARD, power);
}

void Stop()  // 정지
{
  analogWrite(pinPWM1, 0);
  analogWrite(pinPWM2, 0);
}

////////////////////////////////////////  setup()

void setup() 
{
  // 모터 제어 핀들을 모두 출력용으로 설정
  
  pinMode( pinDIR1, OUTPUT ); // 1번(왼쪽)모터 방향 핀
  pinMode( pinPWM1, OUTPUT ); // 1번(왼쪽)모터 속력 핀

  pinMode( pinDIR2, OUTPUT ); // 2번(오른쪽)모터 방향 핀
  pinMode( pinPWM2, OUTPUT ); // 2번(오른쪽)모터 속력 핀
 
  pinMode(pinBuzzer, OUTPUT); // 부저 핀을 출력 핀으로 설정
  
  pinMode(pinButton, INPUT);  // 버튼 핀을 입력용 핀으로 설정
  
  Stop(); // 정지

  // EEPROM에 저장되어 있는 데이터(좌우 모터와 바닥면의 IR 조정값) 읽기
  if( IsEepromDataValid = CheckEepromDataHeader() )
  {
    // 이전에 EEPROM에 저장된 올바른 데이터가 있음
    
    ReadPowerRatio(); // 좌우 모터 속도 조정 비율값 읽기
  }
  
  tone( pinBuzzer, 262 ); // 초기화가 끝났음을 "도미솔" 음 재생
  delay( 150 );
  tone( pinBuzzer, 330 );
  delay( 150 );
  tone( pinBuzzer, 392 ); 
  delay( 250 );
  noTone( pinBuzzer );  // 스피커/부저 끄기(음소거)
}

////////////////////////////////////////  loop()

bool  DoMove = false; // 주행(=1) 또는 정지(=0)

void loop()
{
  if( digitalRead(pinButton) == 0 ) // 버튼이 눌림
  {
    if( DoMove )  // 주행 중이면 정지시킴
    {
      Stop();
      
      tone( pinBuzzer, 330 ); // "미"
      delay( 100 );
      tone( pinBuzzer, 262 ); // "도"
      delay( 250 );
      noTone( pinBuzzer );
    }
      
    while( digitalRead(pinButton) == 0 ) // 버튼이 올려질 때 까지 대기
      delay( 10 );

    DoMove = ! DoMove;
    
    if( DoMove )
    {
      tone( pinBuzzer, 262 ); // "도"
      delay( 100 );
      tone( pinBuzzer, 330 ); // "미"
      delay( 250 );
      noTone( pinBuzzer );
      
      Forward( 80 ); // 주행 속력 (80= 약간 느리게, 120= 약간 빠르게)
    }
  }
}
