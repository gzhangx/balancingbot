

/* 
 *  Initial tries for a self balancing bot
 * Build on Lib: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 */

#include "I2Cdev.h"
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
//A5 => SCL
//A4 => SDA
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#define INTERRUPT_PIN 3  // use pin 2 on Arduino Uno & most boards
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

 

double setpoint= 172;    //bigger to empty side 
double Kp = 100;
double Kd = 0.3; 
double Ki = 350;

double input, output;
/******End of values setting*********/

int motorCounter = 0;
int motorCounterMax = 255;

int motorPins[][2] = {
  {8,9},
  {6, 7}
};
int motorSpeed[]={0,0};

int BOUND=20;

int potms[] = {A0, A1, A2};
const int POTSNUM = 3;


PID pid = PID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void serprintln(String s) {
  if (Serial) Serial.println(s);    
}
void setup() {
       
    Serial.begin(115200);
    serprintln("serial initialized");

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-motorCounterMax, motorCounterMax);  
    
    for (int i = 0; i < POTSNUM; i++) {
      pinMode(potms[i], INPUT);
    }
    
    for (int i = 0; i < 2; i++) {
      for(int k = 0; k < 2; k++) {
        int cur = motorPins[i][k];
        pinMode(cur, OUTPUT);
      }
    }
    
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        serprintln("wire.begin");
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
      
  // initialize device
    serprintln(F("Initializing I2C devices..."));
    
    mpu.initialize();
    //pinMode(INTERRUPT_PIN, INPUT);

     // verify connection
    serprintln(F("Testing device connections..."));
    serprintln(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(82);
    mpu.setYGyroOffset(-65);
    mpu.setZGyroOffset(-8);
    mpu.setXAccelOffset(402); 
    mpu.setYAccelOffset(838); 
    mpu.setZAccelOffset(1564); 

      // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        serprintln(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        serprintln(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        serprintln(F(")"));
    }

}


void loop() {
  //loop_simple();
  //loop_motor();
  loop_balance();
}
void loop_simple() {
  int who = 1;
   if (Serial.available()){
    char dbgstr[128];
    int c = Serial.read();
    if (c == 'F') motorControlDrive(who,'F');
    if (c == 'R') motorControlDrive(who,'R');
    if (c == 'S') motorControlDrive(who,'S');
 
    
  }
}
int dbgspeed = 0;
void loop_motor() {
  motorCounterMax = 5;
  motorCounter++;
  if (motorCounter>motorCounterMax)motorCounter=0;
  char dir = 'F';
  if (Serial.available()){
    char dbgstr[128];
    int c = Serial.read();
    if (c == 'F') dir = 'F';
    if (c == 'R') dir = 'R';
    if (c == 'W') dbgspeed++;
    if (c=='D') dbgspeed--;
    if (dbgspeed < 0) dbgspeed = 0;
    if (dbgspeed > motorCounterMax) dbgspeed = motorCounterMax;
    sprintf(dbgstr,"%c %i", c, dbgspeed);
    serprintln(dbgstr);
    int sp = dbgspeed;
  if (dir == 'R') sp = -sp;
  motorSpeed[0] = motorSpeed[1] = sp;
  }
  
  motorControlAll();  
}

void loop_balance() {
  if (!dmpReady) return;
  motorCounter++;
  if (motorCounter>motorCounterMax)motorCounter=0;
  int potVal[POTSNUM];
  for (int i = 0; i < POTSNUM; i++) {
    potVal[i] = analogRead(potms[i]);     
  }
  //Kp = potVal[0]+1;
  //Ki = potVal[1];
  //setpoint = map(potVal[2], 0, 1023, 180-BOUND, 180+BOUND);   
  //int pot_a2_val =  analogRead(POT_A2); //values 786 (or 642 for batt) to 0
       
  motorControlAll();    

  while (!mpuInterrupt && fifoCount < packetSize) 
  {  
     pid.Compute();           
     motorSpeed[0] = output;
     motorSpeed[1] = output;
        
     if (input<(setpoint - BOUND) || input> (setpoint+BOUND)){     
         motorSpeed[0] = motorSpeed[1] = 0;
     }
     if (mpuInterrupt){
        serprintln("int="+String(mpuInterrupt)+" i=" +String(input)+" o=" + String(output));
     }
        
        
        
      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();

      // get current FIFO count
      fifoCount = mpu.getFIFOCount();
      
      if ((mpuIntStatus & 0x10) || fifoCount == 1024)
      {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        serprintln("FIFO overflow!");   
      }
      else if (mpuIntStatus & 0x02)
      {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
        mpu.dmpGetGravity(&gravity, &q); //get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr
        input = ((ypr[1] * 180)/M_PI) + 180;
   }
}

void motorControlAll() {
  for (int i = 0; i < 2; i++)
    motorControl(i);  
}

void motorControl(int who) {
  int speed = motorSpeed[who];
  char dir = 'S';
  if (speed > 0) dir = 'F';
  if (speed < 0) {
    dir = 'R';
    speed = -speed;
  }  
  if (speed > motorCounter) {
    motorControlDrive(who, dir);
  }else {
    motorControlDrive(who, 'S');
  }
}
void motorControlDrive(int who, char dir){
  int pin1 = motorPins[who][0];
  int pin2 = motorPins[who][1];

  if (dir == 'R') {
    digitalWrite(pin1, 0);
    digitalWrite(pin2, 1);    
  }else if (dir == 'F') {
    digitalWrite(pin1, 1);
    digitalWrite(pin2, 0);    
  } else {
    digitalWrite(pin1, 1);
    digitalWrite(pin2, 1);
  }
}