//SELFBALANCINGCODE


  //Libraries for MPU-6050

    #include "MPU6050_6Axis_MotionApps20.h"
    #include <Wire.h> 

    //PID Library and Cytron for motor driving
    #include <PID_v1.h>
    #include <CytronMotorDriver.h>

    //I2C -SCL SDA on arduino Mega
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"   
    #endif

    #define MIN_ABS_SPEED 5

    MPU6050 mpu;

    // MPU control Status Variables
    bool dmpReady = false; // set true if DMP init was successful
    uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
    uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount; // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q; // [w, x, y, z] quaternion container
    VectorFloat gravity; // [x, y, z] gravity vector
    float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

    // PID
    double initialSetpoint = 183.33;
    double setpoint = initialSetpoint;
    double movingAngleOffset = 1;
    double input, output;

    // Do multiple iterations to find out the best value
    double Kp = 21;//Set this first
    double Kd = 0.8;//Set this secound
    double Ki = 140; //Finally set this 
    PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

CytronMD lm(PWM_DIR, 6, 7); // Left motor driver
CytronMD rm(PWM_DIR, 8, 9); // Right motor driver


    volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

    void dmpDataReady()
    {
      mpuInterrupt = true;
    }

    void setup()
    {
      // join I2C bus (I2Cdev library doesn't do this automatically) SCL SDA Arduino Mega
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
    
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
    #endif

      Serial.begin(57600);
      mpu.initialize();

      devStatus = mpu.dmpInitialize();

    /*
      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(220);
      mpu.setYGyroOffset(76);
      mpu.setZGyroOffset(-85);
      mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    */
      // make sure it worked (returns 0 if so)
      if (devStatus == 0)
      {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection Digital Pin D2
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        // setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255,255);
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

  void loop()
    {
      // if programming failed, don't try to do anything
      if (!dmpReady) return;

      // wait for MPU interrupt or extra packet(s) available
      while (!mpuInterrupt && fifoCount < packetSize)
      {
        // no MPU data - performing PID calculations and output to motors
        pid.Compute();
        Serial.println(input);
        lm.setSpeed(static_cast<int16_t>(output));
        rm.setSpeed(static_cast<int16_t>(output));
      }

      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();

      // get current FIFO count
      fifoCount = mpu.getFIFOCount();

      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024)
      {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
      }
      else if (mpuIntStatus & 0x02)
      {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        Serial.println("entering else if loop");
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        input = ypr[1] * 180 / M_PI + 180;
        //Serial.println(input);
      }
    }
