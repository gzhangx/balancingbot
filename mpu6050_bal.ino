

/* 
 *  Initial tries for a self balancing bot
 * Build on Lib: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 */

#include "I2Cdev.h"
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

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

 

/*********Tune these 4 values for your BOT*********/
double setpoint= 172; //set the value when the bot is perpendicular to ground using serial monitor. 
//Read the project documentation on circuitdigest.com to learn how to set these values
double Kp = 21; //Set this first
double Kd = 0.8; //Set this secound
double Ki = 140; //Finally set this 
/******End of values setting*********/

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int EN_B = 12;      //Enable pin for first motor
int IN_B = 11;       //control pin for first motor
int IN_A = 10;
int EN_A = 9;

void setup() {
    pinMode(EN_B, OUTPUT);
    pinMode(IN_B, OUTPUT);  
    pinMode(EN_A, OUTPUT);
    pinMode(IN_A, OUTPUT);  
    
    Serial.begin(115200);
    while (!Serial);
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Serial.println("wire.begin");
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
      
  // initialize device
    Serial.println(F("Initializing I2C devices..."));
    
    mpu.initialize();
    //pinMode(INTERRUPT_PIN, INPUT);

     // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688); 

      // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        //setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}

char temps1[16];
char temps2[16];
char temps3[16];
 char debugline[256];

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available

        
        //no mpu data - performing PID calculations and output to motors     
        pid.Compute();   
        
        //Print the value of Input and Output on serial monitor to check how it is working.
        //Serial.print(input); Serial.print(" =>"); Serial.println(output);
        dtostrf(ypr[1], 4, 0, temps1);
        sprintf(debugline, "%3i %3i %3i  %5i=>%5i", (int)(ypr[0]*100), (int)(ypr[1]*100), (int)(ypr[2]*100), (int)input, (int)output);
        Serial.println(debugline);
        if (input>150 && input<200){//If the Bot is falling 
          
        if (output>0) //Falling towards front 
        Forward(); //Rotate the wheels forward 
        else if (output<0) //Falling towards back
        Reverse(); //Rotate the wheels backward 
        }
        else //If Bot not falling
        Stop(); //Hold the wheels still
        
        
   
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
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

        input = ypr[1] * 180/M_PI + 180;


        /*
Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            */
   }
}

void motor(char who, char op, int speed) {
    if (speed < 0) speed = -speed;
    speed = map(speed, 0, 255, 0, 255);
    int IN = who == 'A'?IN_A:IN_B;
    int EN = who == 'A'?EN_A:EN_B;
    int lh = op == 'L'?LOW:HIGH;
    digitalWrite(IN, lh);
    //digitalWrite(EN_A, HIGH);
    analogWrite(EN, speed);
}
void Forward() //Code to rotate the wheel forward 
{
    motor('A', 'H', output);
    motor('B', 'H', output);
    Serial.print("F"); //Debugging information 
}

void Reverse() //Code to rotate the wheel Backward  
{
    motor('A', 'L', output);
    motor('B', 'L', output);
    Serial.print("R");
}

void Stop() //Code to stop both the wheels
{
    motor('A', 'L', 0);
    motor('B', 'L', 0);
    Serial.print("S");
}
