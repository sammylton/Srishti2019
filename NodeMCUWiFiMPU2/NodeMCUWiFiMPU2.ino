// change your ssid and password accordingly

#include <ESP8266WiFi.h>

#include <ESP8266HTTPClient.h>
String me;
String  httpurl;
String  TheHiddenAnswerOfClient;
HTTPClient http;
//float a[2];
String mystring[2]; //defined a string array to transfer ypr data to pi since float values can't be transferred
//------------------------------------------------------------------------------------------DMP------------------------------------------------------------------------------------------
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL


#define INTERRUPT_PIN 15  // D8 on nodemcu                                                                      //INT
#define LED_PIN 16 // led_builtin on nodemcu
bool blinkState = false;
int p ,r;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU                                          //INT 
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprn[3];
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high                    //INT
void dmpDataReady() {
    mpuInterrupt = true;
}
//--------------------------------------------------------------------------------------DMP----------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------FUNCTIONS-------------------------------------------------------------------------------------------------

String gettheAnswer(String IPcache, String monmessagecache) {
httpurl = "http://";
httpurl+=IPcache;
httpurl+="/";
httpurl+=monmessagecache;
http.begin(httpurl);
http.GET();
TheHiddenAnswerOfClient = (http.getString());
http.end();
return TheHiddenAnswerOfClient;
}

void setup()
{

  pinMode(16,OUTPUT);
  Serial.begin(115200);
  WiFi.disconnect();
  delay(3000);
  Serial.println("START");
   WiFi.begin("sampc","sampcpassword");                            // to be checked each time
  while ((!(WiFi.status() == WL_CONNECTED))){
    delay(300);
    //Serial.print("..");

  }
  Serial.println("Connected");
  Serial.println("Your IP is");
  Serial.println((WiFi.localIP().toString()));
  //---------------------------------------------------------------------------DMP--------------------------
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();//use default I2C pins on NodeMCU 
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();                                                                                        
    pinMode(INTERRUPT_PIN, INPUT);                                                                            //INT 
 
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data                                                 // commented by Sameer for module to run automatically as soon as power is applied
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(31);
    mpu.setYGyroOffset(-50);
    mpu.setZGyroOffset(-15);
    mpu.setZAccelOffset(1582); // 1688 factory default for my test chip
                                                                                               // To be calculated by IMU example code in File under MPU6050
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
 //-------------------------------------------------------------------------DMP---------------------------------------------------------------------------------------------------- 
//16,5,4,0,2
}


void loop()
{// if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        //Serial.println(F("FIFO overflow!"));
        mystring[0]=String(yprn[0]);                                                                                  //this plus upcoming 5 line sare edited by Sameer
        mystring[1]=String(yprn[1]);
        mystring[2]=String(yprn[2]);
        String rpy = mystring[0]+","+mystring[1]+","+mystring[2];
        me=gettheAnswer("192.168.137.71:8080",rpy);              // ip to be changed accordingly
        Serial.println(me);

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.resetFIFO();                                                                       // Edited as per stackexchange
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        

        #ifdef OUTPUT_READABLE_YAWPITCHROLL                                  //currently defined
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            yprn[0]=ypr[0] * 180/M_PI;
            yprn[1]=ypr[1] * 180/M_PI;
            yprn[2]=ypr[2] * 180/M_PI;
            /*yprn[0]=map(yprn[0], -180.00, +180.00, 1000, 2000);
            yprn[1]=map(yprn[1], -180.00, +180.00, 1000, 2000);
            yprn[2]=map(yprn[2], -180.00, +180.00, 1000, 2000); */
            //Serial.print("ypr\t");
            /*Serial.print(yprn[0]);
            Serial.print("\t");
            Serial.print(yprn[1]);
            Serial.print("\t");
            Serial.println(yprn[2]);
            */
            //Serial.print("--\t");

            if(yprn[1]>-30 && yprn[1]<30) {p=0;} //Edited for pixhawk 
            else if(yprn[1]>30) { p=10; }
            else if(yprn[1]<-30) { p=-10; }
            

            if(yprn[2]>-30 && yprn[2]<30){r=0;}
            else if(yprn[2]>30) { r=10;}
            else if(yprn[2]<-30) { r=-10;}

            //Serial.print("\n");
        #endif

        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
 mystring[0]=String(p);
 mystring[1]=String(r);
 //mystring[2]=String(yprn[2]);
 String pr = mystring[0]+","+mystring[1];
 me=gettheAnswer("192.168.137.123:8080",pr);              // ip to be changed accordingly
 Serial.println(me);
//digitalWrite(16,HIGH);
//delay(3000); introduces FIFO overflow
 //me=gettheAnswer("192.168.137.71:8080",mystring[0]);
//digitalWrite(16,LOW);
//Serial.println(me);
 //delay(500);
}
