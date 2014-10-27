//write by tinnakon_za@hotmail.com
#include <Wire.h>
#include <Servo.h> 
#include <L3G4200D.h>
#include <LSM303.h>
L3G4200D gyro;
LSM303 compass;

#define RAD_TO_DEG 57.295779513082320876798154814105
//gain PID
#define kproll 4.212 //x 4.521 lamda
#define kiroll 0.0802 //x 0.0441
#define kdroll 0.4221 //x 0.421  k2

#define kpPitch 4.212//y 4.55 lamda
#define kiPitch 0.0802//y 0.0441
#define kdPitch 0.4221//y 0.421  k1

#define kpyaw 3.151  //z 2.251
#define kiyaw 0.0  //z

//remote
//#define kprate 0.352  //z 0.352
#define xp1 0.025 //remote
#define yp1 0.025
#define xd1 0.35 //remote
#define yd1 0.35
#define zd1 0.45

Servo moter1;  // create moter
Servo moter2;  // create moter
Servo moter3;  // create moter
Servo moter4;  // create moter

int16_t RxChannel1;		// ISR vars volatile 
int16_t RxChannel2;
int16_t RxChannel3;
int16_t RxChannel4;
int16_t RxChannel5;

uint16_t RxChannel1Start;		// ISR vars
uint16_t RxChannel2Start;
uint16_t RxChannel3Start;
uint16_t RxChannel4Start;
uint16_t RxChannel5Start;

float xAngle = 0.0;
float yAngle = 0.0;
float uiroll=0.0; //x
float uiPitch=0.0;//y
float uiyaw=0.0;  //z

float width11=1022;//filter
float width21=1510;//filter
float width31=1564;//filter
float width41=1533;//filter

//Used for timing
int timer=0;
int i=0;

void setup() {
  moter1.attach(9); 
  moter2.attach(10);
  moter3.attach(11);
  moter4.attach(12);

  Serial.begin(9600);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(8, INPUT);
  pinMode(A0, INPUT);
  
  	// pin change interrupt enables
	PCICR |= (1 << PCIE0);			// PCINT0..7	
        PCICR |= (1 << PCIE1);	
	PCICR |= (1 << PCIE2);			// PCINT16..23

	// pin change masks
	PCMSK0 |= (1 << PCINT0);		// PB0
        PCMSK1 |= (1 << PCINT8);		// PC0
	PCMSK2 |= (1 << PCINT20);		// PD4
	// external interrupts
	EICRA  = (1 << ISC00) | (1 << ISC10);	// Any change INT0, INT1
	EIMSK  = (1 << INT0) | (1 << INT1);		// External Interrupt Mask Register
	EIFR |= (1 << INTF0) | (1 << INTF1);
         Wire.begin();
  compass.init();
  compass.enableDefault();
  gyro.enableDefault();//250 dps 16 bit 65536
}

// the loop routine runs over and over again forever:
void loop() 
{
   int dt = 10;//roop run = 10 ms = 100 Hz
   int x = millis()/dt;
   int y = (millis() - (x*dt));
    if(y == 0)
    {
      int dtime = millis() - timer;
       gyro.read();
       compass.read();
       
      float gyroXrate = (gyro.g.x*0.00875) + 0.0;//-0.0091
      float gyroYrate = (gyro.g.y*0.00875) - 1.1;// 70 mdps/digit; 1 dps = 0.07 //FS = 250 dps 8.75
      float gyroZrate = (gyro.g.z*0.00875) + 0.3;
      float R = sqrt(pow(compass.a.x,2)+pow(compass.a.y,2)+pow(compass.a.z,2));
      float accXangle = (acos(compass.a.y/R)*RAD_TO_DEG - 90)*-1 - 2.8;
      float accYangle = (acos(compass.a.x/R)*RAD_TO_DEG - 90) - 2.1;
      xAngle = (0.99)*(xAngle + gyroXrate*0.01) + (0.01*accXangle);
      yAngle = (0.99)*(yAngle + gyroYrate*0.01) + (0.01*accYangle);
      xAngle = constrain(xAngle, -45, 45); 
      yAngle = constrain(yAngle, -45, 45);
      
  width11 = width11 + ((RxChannel1 - width11)*dt*0.03401);
  width21 = width21 + ((RxChannel2 - width21)*dt*0.03401);
  width31 = width31 + ((RxChannel3 - width31)*dt*0.03401);
  width41 = width41 + ((RxChannel4 - width41)*dt*0.03401);
  
  int throttle = map(width31, 1140, 1928, 980, 1920);
  
  float e1 = xAngle - ((width21-1476)*xp1); //roll
  float e2 = yAngle - ((width11-1468)*yp1); //pitch
  float e3 = gyroZrate - (width41 - 1460)*zd1; //rudder   //yaw
  
  float r1 = (kproll*e1) + (gyroXrate - (width21-1476)*xd1);
  float r2 = (kpPitch*e2) + (gyroYrate - (width11-1468)*yd1);
  
  float uyaw = (e3*kpyaw) + uiyaw;                              //z use PI control yaw
  float uroll =  (kdroll*r1*-1) - uiroll;      //x use adaptive PID control (0.0002125*gyroYrate*gyroZrate)
  float uPitch = (kdPitch*r2*-1) - uiPitch;   //y use adaptive PID control (0.0002125*gyroXrate*gyroZrate)

  int u1 = throttle + uroll + uyaw;      // R
  int u2 = throttle - uroll + uyaw;  // L
  int u3 = throttle + uPitch - uyaw;      // B
  int u4 = throttle - uPitch - uyaw;      // F

  u1 = constrain(u1, 1000, 1920);                 //pin D5  roll R   speed L (counter clockwise)
  u2 = constrain(u2, 1000, 1920);                 //pin D6  roll L   speed L (counter clockwise)
  u3 = constrain(u3, 1000, 1920);                 //pin D7  Pitch B  speed R (clockwise)
  u4 = constrain(u4, 1000, 1920);                 //pin D8  Pitch F  speed R (clockwise)
  
  if(throttle >= 1020)
  {
   i = i + 1;
  uiroll = uiroll + (kiroll*r1*dt*0.001);
  uiPitch = uiPitch + (kiPitch*r2*dt*0.001);
  uiyaw = uiyaw + (kiyaw*e3*dt*0.001);
  uiroll = constrain(uiroll, -100, 100);
  uiPitch  = constrain(uiPitch, -100, 100);
  uiyaw  = constrain(uiyaw, -30, 30);
  moter1.writeMicroseconds(u1); 
  moter2.writeMicroseconds(u2);
  moter3.writeMicroseconds(u3); 
  moter4.writeMicroseconds(u4); 
  }
  else
  {
    dtime = 10;
    i = i;
  uiroll = 0;
  uiPitch = 0;
  uiyaw = 0;
  moter1.writeMicroseconds(1000); 
  moter2.writeMicroseconds(1000);
  moter3.writeMicroseconds(1000); 
  moter4.writeMicroseconds(1000);
  }  
  
      if(i >= 20)
      {
        i=0;
        Serial.print(gyroXrate,1);Serial.print("\t");
        Serial.print(gyroYrate,1);Serial.print("\t");
        Serial.print(gyroZrate,1);Serial.print("\t");
        //Serial.print(xAngle,1);Serial.print("\t");
        //Serial.print(yAngle,1);Serial.print("\t");
        //Serial.print(zAngle,2);Serial.print("\t");
        
        // Serial.print(r1,1);Serial.print("\t");
         //Serial.print(r2,1);Serial.print("\t");
        // Serial.print(u1,1);Serial.print("\t");
         //Serial.print(u3,1);Serial.print("\t");
         
        //Serial.print(sensorValue);Serial.print("\t");
        //Serial.print(RxChannel1);Serial.print("\t");
        //Serial.print(RxChannel2);Serial.print("\t");
        //Serial.print(RxChannel3);Serial.print("\t");
        //Serial.print(RxChannel4);Serial.print("\t");
        //Serial.print(RxChannel5);Serial.print("\t");
        //Serial.print(dtime);Serial.print("\t");
        Serial.println(millis());
        //Serial.println(dtime);
       }
     timer = millis();
    } 
}

ISR(PCINT0_vect)
{
	if (digitalRead(8) == HIGH)			// rising
	{
		RxChannel4Start = micros();

	} else {				// falling
		RxChannel4 = micros() - RxChannel4Start;
                RxChannel4 = constrain(RxChannel4, 900, 2000);
	}
}

ISR(PCINT1_vect)
{
	if (digitalRead(A0) == HIGH)			// rising
	{
		RxChannel5Start = micros();

	} else {				// falling
		RxChannel5 = micros() - RxChannel5Start;
                RxChannel5 = constrain(RxChannel4, 900, 2000);
	}
}

ISR(PCINT2_vect)
{
	if (digitalRead(4) == HIGH)			// rising
	{
		RxChannel3Start = micros();

	} else {				// falling
		RxChannel3 = micros() - RxChannel3Start;
                RxChannel3 = constrain(RxChannel3, 900, 2000);
	}
}

ISR(INT0_vect)
{
	if (digitalRead(2) == HIGH)		
	{
		RxChannel2Start = micros();

	} else {				// falling
		RxChannel2 = micros() - RxChannel2Start;
                RxChannel2 = constrain(RxChannel2, 900, 2000);
	}
}

ISR(INT1_vect)
{
	if (digitalRead(3) == HIGH)		
	{
		RxChannel1Start = micros();

	} else {				// falling
		RxChannel1 = micros() - RxChannel1Start;
                RxChannel1 = constrain(RxChannel1, 900, 2000);
	}
}
