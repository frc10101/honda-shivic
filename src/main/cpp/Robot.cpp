/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//#include <frc.h>
//#include <frc/stdint.h>

#include <frc/PWMVictorSPX.h>
#include <frc/TimedRobot.h>

#include <frc/Timer.h>
#include <frc/AnalogInput.h>

#include <frc/Servo.h>


#include "ctre/Phoenix.h"
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>


#include <frc/ADXRS450_Gyro.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

//#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/DriverStation.h>

#include <frc/PIDController.h>

/*
 * 2019 Robot Code Team 442
 * 
 */


//Blue robot or practice?  Arm Angle Offset affected, PWM motor drive (4 vs 2),
#define COMPETITION_CHASSIS



//#define USE_XBOX
//#define USE_BOTH_CONTROLLERS
#define USE_THRUSTMASTER
//#define USE_DUAL_THRUSTMASTERS

//#define JUST_TEST_SOLENOIDS


#ifdef USE_XBOX
#include "RRXboxController.h"

#define CONTROLLER_REF            RRXboxController xbox{0}
#define CONTROLLER_CALIBRATE      xbox.Calibrate()
#define CONTROLLER_SHIFTUP        xbox.OnRightBumper()
#define CONTROLLER_SHIFTDOWN      xbox.OnLeftBumper()

#define CONTROLLER_DRIVEFWD       xbox.GetLeftY()
#define CONTROLLER_DRIVELR        xbox.GetLeftX()

#define CONTROLLER_ON_HOLDANGLE   xbox.OnY()
#define CONTROLLER_HOLDANGLE      xbox.GetBtnY()

#define CONTROLLER_AUTOHATCH      xbox.GetBtnX()

#define CONTROLLER_ARM_CLICKUP    xbox.OnDPadUp()
#define CONTROLLER_ARM_CLICKDWN   xbox.OnDPadDown()

#define CONTROLLER_HATCHGRAB      xbox.OnB()
#define CONTROLLER_HATCHPUT       xbox.OnA()


#define CONTROLLER_CAMERAUP       xbox.GetBtnStart()
#define CONTROLLER_CAMERADOWN     xbox.GetBtnBack()

#define CONTROLLER_LIFT_UP        xbox.OnX()
#define CONTROLLER_LIFT_DOWN      xbox.OnB()

#define CONTROLLER_DEPLOY_RAMP    0

#define CONTROLLER_CARGOHANDLER_UP    (xbox.GetRightY() > 0.5)
#define CONTROLLER_CARGOHANDLER_DOWN  (xbox.GetRightY() < -0.5)
#define CONTROLLER_CARGOSPIN_FWD      xbox.GetBtnA()
#define CONTROLLER_CARGOSPIN_BKW      xbox.GetBtnY()



#define CONTROLLER_TESTARM_TEXT   "Read arm with X\nSet arm zero file with Y\nReset arm from file A\n"
#define CONTROLLER_TESTARM_READ   xbox.OnX()
#define CONTROLLER_TESTARM_WRITE  xbox.OnY()
#define CONTROLLER_TESTARM_LOAD   xbox.OnA()

#define CONTROLLER_SOLENOID_TEST_TEXT  "Solenoid tester...\nSelect sol:DPadRight, RightBumper\n"
#define CONTROLLER_SOLENOID_TEST_ON xbox.GetRightBumper()
#define CONTROLLER_SOLENOID_TEST_NEXT xbox.OnDPadRight() 


#endif
//#else

#ifdef USE_BOTH_CONTROLLERS

#include "Thrustmaster1600.h"
#include "RRXboxController.h"


#define CONTROLLER_REF            ThrustmasterController tm1600{0}; RRXboxController xbox{1}
#define CONTROLLER_CALIBRATE      tm1600.Calibrate()

//#define CONTROLLER_SHIFTUP        tm1600.OnBtn(3)
//#define CONTROLLER_SHIFTDOWN      tm1600.OnBtn(4)
#define CONTROLLER_THROTTLESPEED  tm1600.GetThrottle()


#define CONTROLLER_DRIVEFWD       tm1600.GetStickY()
#define CONTROLLER_DRIVELR        tm1600.GetStickX() + tm1600.GetStickZ()

#define CONTROLLER_ON_HOLDANGLE   tm1600.OnBtn(2) || xbox.OnY()
#define CONTROLLER_HOLDANGLE      tm1600.GetBtn(2) || xbox.GetBtnY()

#define CONTROLLER_AUTOHATCH      tm1600.GetBtn(1) || xbox.GetBtnX()

#define CONTROLLER_ARM_CLICKUP    tm1600.OnDPadUp() || xbox.OnDPadUp()
#define CONTROLLER_ARM_CLICKDWN   tm1600.OnDPadDown() || xbox.OnDPadDown()

#define CONTROLLER_HATCHGRAB      tm1600.OnBtn(8) || (xbox.GetRightTrigger()>0.5)
//xbox.OnA()
#define CONTROLLER_HATCHPUT       tm1600.OnBtn(7) || (xbox.GetLeftTrigger()>0.5)
//xbox.OnY()

#define CONTROLLER_CAMERAUP       tm1600.GetBtn(5) || (xbox.GetRightY() > 0.5)
#define CONTROLLER_CAMERADOWN     tm1600.GetBtn(10) || (xbox.GetRightY() < -0.5)

//#define CONTROLLER_LIFT_UP        tm1600.OnBtn(3) || xbox.OnX()
//#define CONTROLLER_LIFT_DOWN      tm1600.OnBtn(4) || xbox.OnB()

#define CONTROLLER_DEPLOY_RAMP    (tm1600.GetBtn(5) && tm1600.GetBtn(11)) || (xbox.GetBtnStart() && xbox.GetBtnBack())

// frc::DriverStation::GetInstance().GetMatchTime());  ???

#define CONTROLLER_TESTARM_TEXT   "Read arm with Rbuttons, TL\nSet arm zero file with TMiddle\nReset arm from file TopR\n"
#define CONTROLLER_TESTARM_READ   tm1600.OnBtn(13)
#define CONTROLLER_TESTARM_WRITE  tm1600.OnBtn(12)
#define CONTROLLER_TESTARM_LOAD   tm1600.OnBtn(11)


#endif



#ifdef USE_DUAL_THRUSTMASTERS

#include "Thrustmaster1600.h"

#define CONTROLLER_REF            ThrustmasterController tm1600{0};  ThrustmasterController tm1600b{1}
#define CONTROLLER_CALIBRATE      tm1600.Calibrate()

//#define CONTROLLER_SHIFTUP        tm1600.OnBtn(3)
//#define CONTROLLER_SHIFTDOWN      tm1600.OnBtn(4)
#define CONTROLLER_THROTTLESPEED  tm1600.GetThrottle()


#define CONTROLLER_DRIVEFWD       (tm1600.GetStickY() + tm1600b.GetStickY())
//tm1600.GetStickX() + tm1600b.GetStickX() +
#define CONTROLLER_DRIVELR         (tm1600.GetStickZ() + tm1600b.GetStickZ())  

#define CONTROLLER_ON_HOLDANGLE   tm1600.OnBtn(2) || tm1600b.OnBtn(2)
#define CONTROLLER_HOLDANGLE      tm1600.GetBtn(2) || tm1600b.GetBtn(2)

#define CONTROLLER_AUTOHATCH      tm1600.GetBtn(1) || tm1600b.GetBtn(1)

#define CONTROLLER_ARM_CLICKUP    tm1600.OnDPadUp() || tm1600b.OnDPadUp()
#define CONTROLLER_ARM_CLICKDWN   tm1600.OnDPadDown() || tm1600b.OnDPadDown()

#define CONTROLLER_HATCHGRAB      tm1600.OnBtn(8) || tm1600b.OnBtn(8)
#define CONTROLLER_HATCHPUT       tm1600.OnBtn(7) || tm1600b.OnBtn(7)

#define CONTROLLER_CAMERAUP   tm1600.GetBtn(5) || tm1600b.GetBtn(5)
#define CONTROLLER_CAMERADOWN   tm1600.GetBtn(10)  || tm1600b.GetBtn(10)

//#define CONTROLLER_LIFTTOGGLE     tm1600.OnBtn(3) || tm1600.OnBtn(4)

#define CONTROLLER_LIFT_UP        tm1600.OnBtn(3) || tm1600b.OnBtn(3)
#define CONTROLLER_LIFT_DOWN      tm1600.OnBtn(4) || tm1600b.OnBtn(4)

#define CONTROLLER_DEPLOY_RAMP    (tm1600.GetBtn(5) && tm1600.GetBtn(11)) || (tm1600b.GetBtn(5) && tm1600b.GetBtn(11))

#define CONTROLLER_CARGOHANDLER_UP    tm1600.GetDPadDown() || tm1600b.GetDPadDown()
#define CONTROLLER_CARGOHANDLER_DOWN  tm1600.GetDPadUp() || tm1600b.GetDPadUp()
#define CONTROLLER_CARGOSPIN_FWD      tm1600.GetBtn(3) || tm1600b.GetBtn(3)
#define CONTROLLER_CARGOSPIN_BKW      tm1600.GetBtn(4) || tm1600b.GetBtn(4)


#define CONTROLLER_TESTARM_TEXT   "Read arm with Rbuttons, TL\nSet arm zero file with TMiddle\nReset arm from file TopR\n"
#define CONTROLLER_TESTARM_READ   tm1600.OnBtn(13) || tm1600b.OnBtn(13)
#define CONTROLLER_TESTARM_WRITE  tm1600.OnBtn(12) || tm1600b.OnBtn(12)
#define CONTROLLER_TESTARM_LOAD   tm1600.OnBtn(11) || tm1600b.OnBtn(11)

#define CONTROLLER_SOLENOID_TEST_TEXT  "Solenoid tester...\nSelect sol:Button 14, Actuate:Button 15\n"
#define CONTROLLER_SOLENOID_TEST_ON tm1600.GetBtn(14) || tm1600b.GetBtn(14)
#define CONTROLLER_SOLENOID_TEST_NEXT tm1600.OnBtn(15) || tm1600b.OnBtn(15)


#endif


#ifdef USE_THRUSTMASTER

#include "Thrustmaster1600.h"

#define CONTROLLER_REF            ThrustmasterController tm1600{0}
#define CONTROLLER_CALIBRATE      tm1600.Calibrate()

//#define CONTROLLER_SHIFTUP        tm1600.OnBtn(3)
//#define CONTROLLER_SHIFTDOWN      tm1600.OnBtn(4)
#define CONTROLLER_THROTTLESPEED  tm1600.GetThrottle()


#define CONTROLLER_DRIVEFWD       tm1600.GetStickY()
#define CONTROLLER_DRIVELR        tm1600.GetStickX() + tm1600.GetStickZ()  

#define CONTROLLER_ON_HOLDANGLE   tm1600.OnBtn(2)
#define CONTROLLER_HOLDANGLE      tm1600.GetBtn(2)

#define CONTROLLER_AUTOHATCH      tm1600.GetBtn(1)

#define CONTROLLER_ARM_CLICKUP    tm1600.OnDPadUp()
#define CONTROLLER_ARM_CLICKDWN   tm1600.OnDPadDown()

#define CONTROLLER_HATCHGRAB      tm1600.OnBtn(8)
#define CONTROLLER_HATCHPUT       tm1600.OnBtn(7)

#define CONTROLLER_CAMERAUP   tm1600.GetBtn(5)
#define CONTROLLER_CAMERADOWN   tm1600.GetBtn(10)

#define CONTROLLER_LIFTTOGGLE     tm1600.OnBtn(3) || tm1600.OnBtn(4)

#define CONTROLLER_LIFT_UP        tm1600.OnBtn(3)
#define CONTROLLER_LIFT_DOWN      tm1600.OnBtn(4)

#define CONTROLLER_DEPLOY_RAMP    tm1600.GetBtn(5) && tm1600.GetBtn(11)

#define CONTROLLER_CARGOHANDLER_UP    tm1600.GetDPadDown()
#define CONTROLLER_CARGOHANDLER_DOWN  tm1600.GetDPadUp()
#define CONTROLLER_CARGOSPIN_FWD      tm1600.GetBtn(1)
#define CONTROLLER_CARGOSPIN_BKW      tm1600.GetBtn(2)


#define CONTROLLER_TESTARM_TEXT   "Read arm with Rbuttons, TL\nSet arm zero file with TMiddle\nReset arm from file TopR\n"
#define CONTROLLER_TESTARM_READ   tm1600.OnBtn(13)
#define CONTROLLER_TESTARM_WRITE  tm1600.OnBtn(12)
#define CONTROLLER_TESTARM_LOAD   tm1600.OnBtn(11)

#define CONTROLLER_SOLENOID_TEST_TEXT  "Solenoid tester...\nSelect sol:Button 14, Actuate:Button 15\n"
#define CONTROLLER_SOLENOID_TEST_ON tm1600.GetBtn(14)
#define CONTROLLER_SOLENOID_TEST_NEXT tm1600.OnBtn(15)


#endif


class Robot : public frc::TimedRobot, public frc::PIDOutput {
 public:

  Robot()
  {
		auto inst = nt::NetworkTableInstance::GetDefault();
		table = inst.GetTable("GRIP");

		PIDtable = inst.GetTable("PIDTUNE");
     
		anglePID = new frc::PIDController(anglePID_P, anglePID_I, anglePID_D, 0.0, &gyro, this);
		anglePID->Enable();

		PIDtable->PutNumber("P", anglePID->GetP());
		PIDtable->PutNumber("I", anglePID->GetI());
		PIDtable->PutNumber("D", anglePID->GetD());

    SetArmZero();
  }

  void PIDWrite(double output)
	{
		anglePIDoutput = output;
	}

  ~Robot()
  {
    delete anglePID;

  }


  void SetArmZero()
  {

		int abspos = talon1.GetSensorCollection().GetPulseWidthPosition() & 0xfff;
		printf("abspos: %d\n", abspos);


#ifdef COMPETITION_CHASSIS
    //float angleOffset = 37.1;
    int absReadingToBeZero = (int)(4096/360.0*37.1);
    printf("Competition Chassis Setup!!!\n");
#else
    
    int absReadingToBeZero = 2200;
    printf("NON-Competition Chassis Setup!!!\n");
#endif

#define ARM_ZERO_FILE   "/home/lvuser/arm_zero.dat"
    {
      FILE * f = NULL;

      f = fopen(ARM_ZERO_FILE, "rb");
      if(f)
      {
        int zero;

        if(fread(&zero, sizeof(int), 1, f) == 1)
        {
          absReadingToBeZero = zero;
          printf("arm zero set from file: %d\n", absReadingToBeZero);
        }  
        fclose(f);
      }
    }

    double pos = abspos - absReadingToBeZero;

    while(pos>1000) pos -= 4096;

    
	//	talon1.GetSensorCollection().SetQuadraturePosition(pos, kTimeoutMs);  //.SetPulseWidthPosition(pos, defaulttimeout);
		printf("before adjust getpos: %d\n", talon1.GetSelectedSensorPosition(kPIDLoopIdx));

    //Or use the below with pos var instead of 0?
    talon1.SetSelectedSensorPosition(pos, kPIDLoopIdx, kTimeoutMs);
    printf("Set sensor pos to %f\n", pos);

	  talon1.SetSensorPhase(false);

		//printf("afterset getpos: %d\n", talon1.GetSensorCollection().GetQuadraturePosition());
printf("after adjust getpos: %d\n", talon1.GetSelectedSensorPosition(kPIDLoopIdx));


		  abspos = talon1.GetSensorCollection().GetPulseWidthPosition() & 0xfff;


		printf("afterset abspos: %d\n", abspos);

  }

  void RobotInit() override
  {
    printf("RobotInit\n");

    DriveMode = DriveMode_Hatch;


    gyro.ClearError();
    
    gyro.Calibrate();
    


    talon1.ClearStickyFaults();

    talon1.ConfigFactoryDefault();

    talon1.SetNeutralMode(NeutralMode::Brake);



		talon1.ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,
				kTimeoutMs);


		/* set the peak and nominal outputs, 12V means full */
		talon1.ConfigNominalOutputForward(0, kTimeoutMs);
		talon1.ConfigNominalOutputReverse(0, kTimeoutMs);
		talon1.ConfigPeakOutputForward(1, kTimeoutMs);
		talon1.ConfigPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 */
		talon1.Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		talon1.Config_kP(kPIDLoopIdx, 2.0, kTimeoutMs);
		talon1.Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		talon1.Config_kD(kPIDLoopIdx, 512.0, kTimeoutMs);    


    //talon2.ClearStickyFaults();

    //talon2.ConfigFactoryDefault();




#ifndef JUST_TEST_SOLENOIDS


    compressor.ClearAllPCMStickyFaults();


 

    SuctionSol.Set(frc::DoubleSolenoid::kReverse);
    HalfChubPusher.Set(frc::DoubleSolenoid::kReverse);

    SuctionPlate.Set(frc::DoubleSolenoid::kReverse);
    RampSolB.Set(frc::DoubleSolenoid::kOff);
    
#endif //JUST_TEST_SOLENOIDS

     auto inst = nt::NetworkTableInstance::GetDefault();
		
    //Setup networktable stuff for raspi ChickenVision (!!! Credit to Screaming Chickens 3997 !!!)
    CVtable = inst.GetTable("ChickenVision");
    CVtable->PutBoolean("Driver", true);
    CVtable->PutBoolean("Tape", true);
    CVtable->PutBoolean("FlipVideo", true);
    
    //point to raspi camera servers for shuffleboard
    //put this in raspi code eventually...
  //  static final String PI_ADDRESS = "";
  //  static final int PORT = 1181; 
  //  inst.GetDefault()
  //    .GetEntry("/CameraPublisher/PiCamera/streams")
  //    .SetStringArray(new String )



    std::string streamNames[1];
    streamNames[0] = "mjpeg:http://10.4.42.214:1181/stream.mjpg";
    inst.GetEntry("/CameraPublisher/PiCamera1/streams").SetStringArray(streamNames);

    std::string streamNames2[1];

    streamNames2[0] = "mjpeg:http://10.4.42.214:1182/stream.mjpg";
    inst.GetEntry("/CameraPublisher/PiCamera2/streams").SetStringArray(streamNames2);

		table = inst.GetTable("ROBOT");

   // table->AddTableListener(&robotTableListener);


   cameraServo.SetAngle(90);


  }

  

  void TeleopInit() override
  {
  
    printf("TeleopInit\n");
    
    CONTROLLER_CALIBRATE;



  }


	void
	RobotPeriodic(void)
	{
		gyroangle = gyro.GetAngle();  //in degrees

		table->PutNumber("desiredAngle", desiredAngle);
		table->PutNumber("gyroAngle", gyroangle);

    table->PutNumber("ArmPosition", currentArmPos = talon1.GetSelectedSensorPosition(kPIDLoopIdx));

    table->PutNumber("DSBattery", frc::DriverStation::GetInstance().GetBatteryVoltage());

    table->PutNumber("anglePIDOutput", anglePIDoutput);

#define TUNE_THE_PID 1
#ifdef TUNE_THE_PID
				//REMOVE THIS AFTER TEST
				{
					float newP, newI, newD;

					newP = PIDtable->GetNumber("P", 1.0);
					newI = PIDtable->GetNumber("I", 0.0);
					newD = PIDtable->GetNumber("D", 0.0);

					if(newP != anglePID_P ||
						newI != anglePID_I ||
						newD != anglePID_D)
					{
						anglePID_P = newP;
						anglePID_I = newI;
						anglePID_D = newD;

						anglePID->SetPID(anglePID_P,anglePID_I,anglePID_D);
					}
				}
#endif


  }

  void DisabledInit()
  {
    printf("DisabledInit.\n");
    talon1.Set(ControlMode::PercentOutput, -0.0);
  }

  void TeleopPeriodic() override 
  {

#ifndef JUST_TEST_SOLENOIDS


    //Speed stuff...
    float motorSpeedMultiplier = 1.0;  //will be set below...
    static int speedSelect = 2;  //start out in middle
    static int lastSpeed = -1;
    #ifdef CONTROLLER_SHIFTUP 

    bool shiftUp = CONTROLLER_SHIFTUP;
    bool shiftDown = CONTROLLER_SHIFTDOWN;

      motorSpeedMultiplier = 0.2 + ((1.0 - CONTROLLER_THROTTLESPEED) / 2.0) * 0.8;
      table->PutNumber("MotorMult", motorSpeedMultiplier);
    /*if(shiftUp || shiftDown)
    {
      if(shiftUp) speedSelect++;
      if(shiftDown) speedSelect--;

      if(speedSelect<0) speedSelect = 0;
      if(speedSelect>4) speedSelect = 4;
      
      const float speeds[] = {0.20, 0.33, 0.5, 0.75, 1.0};

      motorSpeedMultiplier = speeds[speedSelect];

      table->PutNumber("Gear", speedSelect);
      table->PutNumber("MotorMult", motorSpeedMultiplier);

    }
    #else

      motorSpeedMultiplier = 0.2 + ((1.0 - CONTROLLER_THROTTLESPEED) / 2.0) * 0.8;
      table->PutNumber("MotorMult", motorSpeedMultiplier);
*/
    #endif

    
//#define TANKDRIVE
#ifdef  TANKDRIVE  

    DONT DO THIS...
    R_motor.Set(xbox.GetRightY() * motorSpeedMultiplier); 
    L_motor.Set(-xbox.GetLeftY() * motorSpeedMultiplier);

#else  //not TANKDRIVE

    float fwd = -(fabs(CONTROLLER_DRIVEFWD) * CONTROLLER_DRIVEFWD);
    float lr = -(fabs(CONTROLLER_DRIVELR) * 0.5 * CONTROLLER_DRIVELR);

    if(CONTROLLER_ON_HOLDANGLE)  //HOLD ANGLE!
    {
      desiredAngle = gyroangle;
      //anglePID->Reset();
      
      anglePID->SetSetpoint(desiredAngle);

    }
    if(CONTROLLER_HOLDANGLE)  //Use PID with gyro to hold angle...
    {
      //desiredAngle
      //tweak lr with anglePIDOutput...
      lr = anglePIDoutput;
    }


    
    {
      
      static bool needs_init = true;

      static float cameraHatchAngle = 25;
      static float cameraCargoAngle = 155;

      static float cameraHatchAngleTweak = 0;
      static float cameraHatchAngleDelta = 0.3;
      
      static float cameraCargoAngleTweak = 0;
      static float cameraCargoAngleDelta = 0.3;
      
      

      if(CONTROLLER_CAMERAUP || needs_init) 
      {
        compressor.Start();

        cameraHatchAngleTweak += cameraHatchAngleDelta;
        if(fabs(cameraHatchAngleTweak) > 25.0) cameraHatchAngleDelta =-cameraHatchAngleDelta;

        DriveMode = DriveMode_Hatch;
        CVtable->PutBoolean("FlipVideo", true );
        CVtable->PutBoolean("Tape", true);
        cameraServo.SetAngle(cameraHatchAngle + cameraHatchAngleTweak);
        needs_init = false;
      }
      if(CONTROLLER_CAMERADOWN) 
      {
        compressor.Stop();

        cameraCargoAngleTweak += cameraCargoAngleDelta;
        if(fabs(cameraCargoAngleTweak) > 25.0) cameraCargoAngleDelta =-cameraCargoAngleDelta;

        DriveMode = DriveMode_Cargo;
        CVtable->PutBoolean("FlipVideo", false );
        CVtable->PutBoolean("Tape", false);
        cameraServo.SetAngle(cameraCargoAngle + cameraCargoAngleTweak);
      }



    }


    if(CONTROLLER_AUTOHATCH && (DriveMode == DriveMode_Hatch) ) // use ChickenVision angle...
    {
      //Set ChickenVision NTable...
      CVtable->PutBoolean("Driver", false);  //sets brightness, etc for targeting...

      static int lasttimestamp = -9999;
      int currenttimestamp = CVtable->GetNumber("VideoTimestamp", -66666);
      if(currenttimestamp == lasttimestamp)  // NOT UPDATING!>>!
      {
          //printf("VideoDown\n");
      }
      else
      {
        bool tapeDetected = CVtable->GetBoolean("tapeDetected", false);
        if(tapeDetected)
        {
          float cvangle = CVtable->GetNumber("tapeYaw", 0.0);
          desiredAngle = gyroangle + cvangle;
          anglePID->SetSetpoint(desiredAngle);
          lr = anglePIDoutput;
        }
      }
    }
    else
    {
      //Set ChickenVision NTable...
      CVtable->PutBoolean("Driver", true);  //sets brightness, etc for driving...
    }
    


//#define RIGHT_STICK_FIELD_CENTRIC
#ifdef RIGHT_STICK_FIELD_CENTRIC
    float rStickX = xbox.GetRightX();
    float rStickY = xbox.GetRightY();
    float rStickAngle = atan2(rStickX, -rStickY) * 360.0/(2*3.1415926);
    float rStickMag = sqrtf(rStickX * rStickX + rStickY * rStickY);

    table->PutNumber("RStickAngle", rStickAngle);
    table->PutNumber("RStickMag", rStickMag);

    if(rStickMag > 0.5)  //override with field centric
    {
      desiredAngle = rStickAngle;
      anglePID->SetSetpoint(desiredAngle);
      lr = anglePIDoutput;
      fwd = xbox.GetRightTrigger();

    }
#endif //RIGHT_STICK_FIELD_CENTRIC



    if(DriveMode == DriveMode_Cargo) fwd = -fwd;//Rear is now the front...

    float lvalue = fwd - lr;
    float rvalue = fwd + lr;


#ifdef COMPETITION_CHASSIS
    L1_motor.Set(lvalue * motorSpeedMultiplier); 
    L2_motor.Set(lvalue * motorSpeedMultiplier); 
    R1_motor.Set(-rvalue * motorSpeedMultiplier);
    R2_motor.Set(-rvalue * motorSpeedMultiplier);
    CargoMotor1.Set(motorSpeedMultiplier);
#else    
    L_motor.Set(lvalue * motorSpeedMultiplier); 
    R_motor.Set(-rvalue * motorSpeedMultiplier);
#endif

#endif  //not TANKDRIVE





  //We will need to find the values we want to target.  Use Phoenix Tuner plot and manually adjust to positions
  //once we know those, use them as target positions (cycle through an array if more than 2)
  //ASSUMPTIONS: statting position is going to be fixed and can be "zero" - so we don't have to use absolute mag

  const int nPositions = 10;
  const int targetPositions[] = { 0, -500, -800, -1200, -1400, -2200, -2400, -2745, -3200, -4000};
  static int currentPosIndex = 2;
  if (CONTROLLER_ARM_CLICKUP) currentPosIndex++;

  if (CONTROLLER_ARM_CLICKDWN) currentPosIndex--;


//Code for timing a get or put of hatch panel...

//OLD WAY:
/*
    if(CONTROLLER_HATCHGRAB)  
    {
      ballPullerTimer.Reset();
      ballPullerTimer2.Reset();

      ballPullerOn = true;
      ballPullerTimer.Start();
      ballPullerTimer2.Start();

      printf("Timer Started!\n");
      SuctionSol.Set(frc::DoubleSolenoid::kForward);

      currentPosIndex += 2; //2 clicks forward before suction...
      //motor on
    }

    if(ballPullerOn)
    {
      if(ballPullerTimer2.HasPeriodPassed(0.70))
      {
        printf("Solenoid OFF\n");
        SuctionSol.Set(frc::DoubleSolenoid::kReverse);  //suction...
        ballPullerTimer2.Stop();
      }

      if(ballPullerTimer.HasPeriodPassed(1.0)) //times up
      {
        ballPullerOn = false;
        ballPullerTimer.Stop();
        printf("Times UP!\n");
        //motor off

        currentPosIndex -= 2;  //2 clicks back
      }
    }
*/


//NEW way:
   enum { DO_NOTHING, 
          START_PICKUP, PICKUP_STAGE1, PICKUP_STAGE2, PICKUP_STAGE3, PICKUP_STAGE4,
          START_PLACEMENT, PLACEMENT_STAGE1, PLACEMENT_STAGE2, PLACEMENT_STAGE3, PLACEMENT_STAGE4,
          START_RAMP_DEPLOY, RAMP_STAGE1, RAMP_STAGE2
        };
    static int semiautostate = DO_NOTHING;

    static frc::Timer semiautotimer;

    static bool rampState = 1;

    if( semiautostate == DO_NOTHING )
    {
      switch(DriveMode)  //since buttons could be shared between hatch vs cargo mode...
      {
        default:
        case DriveMode_Hatch:

          if(CONTROLLER_HATCHGRAB)  semiautostate = START_PICKUP;
          if(CONTROLLER_HATCHPUT)  semiautostate = START_PLACEMENT;
          if(CONTROLLER_DEPLOY_RAMP)  semiautostate = START_RAMP_DEPLOY;

          talon1.Set(ControlMode::PercentOutput, 0.08);

        break;
        case DriveMode_Cargo:

          if(CONTROLLER_CARGOHANDLER_UP)
          {
            talon1.Set(ControlMode::PercentOutput, 0.5);
          }
          else
          if(CONTROLLER_CARGOHANDLER_DOWN)
          {
            talon1.Set(ControlMode::PercentOutput, -0.3);
          }
          else
          {
            talon1.Set(ControlMode::PercentOutput, 0.08);
          }
          
          if(CONTROLLER_CARGOSPIN_FWD)  //intake...
          {
            CargoMotor1.Set(0.6);  //3
            CargoMotor2.Set(-0.6);  //5
          }
          else
          if(CONTROLLER_CARGOSPIN_BKW)
          {

            CargoMotor1.Set(-1.0 * motorSpeedMultiplier);  //1
            CargoMotor2.Set(1.0 * motorSpeedMultiplier); //-1
          }  
          else
          {
            CargoMotor1.Set(0.0);
            CargoMotor2.Set(0.0); 
          }
          

        break;
      }
     }


    table->PutNumber("AutoState", semiautostate);

    switch(semiautostate){

      case START_PICKUP:
        semiautotimer.Reset();
        semiautotimer.Start();
        semiautostate++;          //need to always move on to next...  or you'll reset timer...
        currentPosIndex += 2; //2 clicks forward before suction...


      //break;  /// INTENTIONALLY FALL THROUGH HERE TO NOT DELAY SOLENOID COMMAND
      case PICKUP_STAGE1:
        if(semiautotimer.HasPeriodPassed(0.700)) semiautostate++;

        SuctionSol.Set(frc::DoubleSolenoid::kForward);
        SuctionPlate.Set(frc::DoubleSolenoid::kForward);

      break;
      case PICKUP_STAGE2:
        if(semiautotimer.HasPeriodPassed(0.300)) semiautostate++;

        SuctionSol.Set(frc::DoubleSolenoid::kReverse);  //suction...

      break;
      case PICKUP_STAGE3:
 
        currentPosIndex -= 2;  //2 clicks back        
        SuctionPlate.Set(frc::DoubleSolenoid::kReverse);
        semiautostate = DO_NOTHING;
        //if(semiautotimer.HasPeriodPassed(0.300)) semiautostate++;

      break;
      //case PICKUP_STAGE4:

      case START_PLACEMENT:
        semiautotimer.Reset();
        semiautotimer.Start();
        semiautostate++;          //need to always move on to next...  or you'll reset timer...
        currentPosIndex += 2; //2 clicks forward and wait to place...

        SuctionPlate.Set(frc::DoubleSolenoid::kForward);

        //FALL THROUGH INTENTIONAL...
      case PLACEMENT_STAGE1:
        if(semiautotimer.HasPeriodPassed(0.700)) semiautostate++;

      break;
      case PLACEMENT_STAGE2:

        SuctionSol.Set(frc::DoubleSolenoid::kForward);  
        HalfChubPusher.Set( frc::DoubleSolenoid::kForward );
        
        if(semiautotimer.HasPeriodPassed(0.300)) semiautostate++;
      break;
      case PLACEMENT_STAGE3:
        
        SuctionPlate.Set(frc::DoubleSolenoid::kReverse);
        currentPosIndex -= 2;  //2 clicks back        
        //HalfChubPusher.Set( frc::DoubleSolenoid::kReverse );
        if(semiautotimer.HasPeriodPassed(0.500)) semiautostate++;
        
      break;

      case PLACEMENT_STAGE4:
        HalfChubPusher.Set( frc::DoubleSolenoid::kReverse );
        semiautostate = DO_NOTHING;
        break;

      case START_RAMP_DEPLOY:
        semiautotimer.Reset();
        semiautotimer.Start();
                

        rampState = !rampState;

        semiautostate++; 
        //FALL THROUGH INTENTIONAL...
      case RAMP_STAGE1:


        //SuctionPlate.Set(rampState ? frc::DoubleSolenoid::kForward : frc::DoubleSolenoid::kReverse);
        //RampSolB.Set(rampState ? frc::DoubleSolenoid::kForward : frc::DoubleSolenoid::kReverse);

        if(semiautotimer.HasPeriodPassed(1.0)) semiautostate++;
       
        break;
      case RAMP_STAGE2:

        //?????
        semiautostate = DO_NOTHING;

        break;


      case DO_NOTHING:
        break;
      default:
        semiautostate = DO_NOTHING;
    }



    if(CONTROLLER_LIFT_UP)
    {
      //LiftSol.Set( frc::DoubleSolenoid::kForward );  //toggle up/down
    }
    if(CONTROLLER_LIFT_DOWN)
    {
      //LiftSol.Set( frc::DoubleSolenoid::kReverse );  //toggle up/down
    }







//Final check for positions at end of everything...
  if(currentPosIndex > nPositions-1) currentPosIndex = nPositions - 1;
  if(currentPosIndex < 0) currentPosIndex = 0;


  //talon1.Set(ControlMode::Position, targetPositions[currentPosIndex]);

  //talon2.Set(ControlMode::Follower, talon1ID);  //WILL THIS WORK AT ALL???
  //OR?
  //talon2.Set(ControlMode::PercentOutput, talon1.GetMotorOutputPercent() );
  

  

//if(nPositions>0 && nPositions<5) //check for 'hit'
{  
  float deltapos = talon1.GetSelectedSensorPosition(kPIDLoopIdx) - (float)targetPositions[currentPosIndex];
  float vel = talon1.GetSelectedSensorVelocity(kPIDLoopIdx);


  table->PutNumber("pos", deltapos);
  table->PutNumber("vel", vel);

  if(deltapos > 50 && vel > 10)
  {
    table->PutBoolean("HIT", true);
    printf("Hit!\n");
  }
  else
    {
      table->PutBoolean("HIT", false);
    }
}








#endif //JUST_TEST_SOLENOIDS
     
  }

  void AutonomousPeriodic() override
  {

    TeleopPeriodic();
    

    /*
    float fwd = 0.0;
    float lr = 0.0;
    float motorSpeedMultiplier = 0.8;


//     use ChickenVision angle...
    {
      static int lasttimestamp = -9999;
      int currenttimestamp = CVtable->GetNumber("VideoTimestamp", -66666);
      if(currenttimestamp == lasttimestamp)  // NOT UPDATING!>>!
      {
          //printf("VideoDown\n");
      }
      else
      {
        bool tapeDetected = CVtable->GetBoolean("tapeDetected", false);
        if(tapeDetected)
        {
          float cvangle = CVtable->GetNumber("tapeYaw", 0.0);
          desiredAngle = gyroangle + cvangle;
          anglePID->SetSetpoint(desiredAngle);
          lr = anglePIDoutput;
          fwd = 0.3;
        }
      }
    }



    float lvalue = fwd - lr;
    float rvalue = fwd + lr;

    


#ifdef COMPETITION_CHASSIS
    L1_motor.Set(lvalue * motorSpeedMultiplier); 
    L2_motor.Set(lvalue * motorSpeedMultiplier); 
    R1_motor.Set(-rvalue * motorSpeedMultiplier);
    R2_motor.Set(-rvalue * motorSpeedMultiplier);
#else    
    L_motor.Set(lvalue * motorSpeedMultiplier); 
    R_motor.Set(-rvalue * motorSpeedMultiplier);
#endif

*/

  }

  void DisabledPeriodic() override
  {
    //do nothing...
  }

  void TestInit() override
  {
    //printf(CONTROLLER_TESTARM_TEXT);
    //compressor.Stop();
  }


  void TestPeriodic() override
  {
    //Turn off arm motor...
    talon1.Set(ControlMode::PercentOutput, 0.0);
    static int zeroStep = 0;
    static int zeroReading = 0;

    if(CONTROLLER_TESTARM_READ)
    {
      
    	int abspos = talon1.GetSensorCollection().GetPulseWidthPosition() & 0xfff;
		  int relpos = talon1.GetSelectedSensorPosition(kPIDLoopIdx);
      printf("abspos: %d  relpos : %d angle: %f\n", abspos, relpos, abspos*360.0/4096.0);
      zeroReading = abspos;
      zeroStep = 1;
    }

    if(CONTROLLER_TESTARM_WRITE)
    {
      if(zeroStep > 0) //reading acquired...
      {
        zeroStep = 0;

        {
          FILE * f = NULL;

          f = fopen(ARM_ZERO_FILE, "wb");
          if(f)
          {
            int zero = zeroReading;

            if(fwrite(&zero, sizeof(int), 1, f) == 1)
            {
              
              printf("arm zero WROTE to file: %d", zero);
            } 
            else printf("error writing to angle file\n");


            fclose(f);

            printf("RELOAD/SET with btnA");

          }
          else
          {
            printf("failed to open angle file\n");
          }
          
        }

      } else printf("Read with X first!");

    }

    if(CONTROLLER_TESTARM_LOAD)
    {
      SetArmZero();
    }



//#define TEST_CONTROLLER
#ifdef  TEST_CONTROLLER



#define PRINTSTATE(x)  if(x) printf(#x);

PRINTSTATE ( CONTROLLER_CALIBRATE )
//PRINTSTATE( CONTROLLER_SHIFTUP )
//PRINTSTATE( CONTROLLER_SHIFTDOWN )

PRINTSTATE( CONTROLLER_DRIVEFWD )
PRINTSTATE( CONTROLLER_DRIVELR )

PRINTSTATE( CONTROLLER_ON_HOLDANGLE )
PRINTSTATE( CONTROLLER_HOLDANGLE )

PRINTSTATE( CONTROLLER_AUTOHATCH )

PRINTSTATE( CONTROLLER_ARM_CLICKUP )
PRINTSTATE( CONTROLLER_ARM_CLICKDWN )

PRINTSTATE( CONTROLLER_HATCHGRAB )
PRINTSTATE( CONTROLLER_HATCHPUT )    


PRINTSTATE( CONTROLLER_TESTARM_READ )
PRINTSTATE( CONTROLLER_TESTARM_WRITE )
PRINTSTATE( CONTROLLER_TESTARM_LOAD )


#endif
    

    //Solenoid Test Section...
    {
      static frc::Solenoid * aSolenoid = NULL;
      static int solPort = 0;
      static int solPCM = 0;
      const int solPCMCount = 1;
      static int oneTime = 1;

      if(oneTime)
      {
        oneTime = 0;
        printf(CONTROLLER_SOLENOID_TEST_TEXT);
      }

      if( CONTROLLER_SOLENOID_TEST_NEXT )
      {
        printf("Controlling PCM %d, solenoid %d...\n", solPCM, solPort);
        if(aSolenoid) free(aSolenoid);
        aSolenoid = new frc::Solenoid(solPCM, solPort);

        solPort++;
        if(solPort>7) 
        {
          solPCM++;
          if(solPCM >= solPCMCount) solPCM = 0;

          solPort = 0;
        }

      }
      if(aSolenoid) aSolenoid->Set( CONTROLLER_SOLENOID_TEST_ON );
    }
    //End Solenoid Test Section...



  }

 private:
  CONTROLLER_REF;      //RR X b o x C o n t r o l l e r   x b o x { 0 };
  

  enum {DriveMode_Hatch, DriveMode_Cargo};
  int DriveMode;  //use states above...

#ifdef COMPETITION_CHASSIS
  frc::PWMVictorSPX L1_motor{0};   //PWM port #
  frc::PWMVictorSPX L2_motor{1};  
  frc::PWMVictorSPX R1_motor{2};   
  frc::PWMVictorSPX R2_motor{3};  
#else  
  frc::PWMVictorSPX L_motor{1};   //PWM port #
  frc::PWMVictorSPX R_motor{2};  
#endif

  frc::Servo  cameraServo{4};  

  frc::AnalogInput analog{0};  //analog input #

  bool ballPullerOn = false;

  //frc::Timer  ballPullerTimer;
  //frc::Timer  ballPullerTimer2;

  frc::PWMVictorSPX CargoMotor1{7};   
  frc::PWMVictorSPX CargoMotor2{8};  

  //talons on canbus
  const int talon1ID = 20;
  TalonSRX talon1{talon1ID};  //canbus ID
  int kPIDLoopIdx = 0;
  int kTimeoutMs = 30;

  const int talon2ID = 21;
  //TalonSRX talon2{talon2ID};  



#ifndef JUST_TEST_SOLENOIDS

  frc::DoubleSolenoid HalfChubPusher {2,3};

  frc::DoubleSolenoid SuctionPlate {1,0};
  frc::DoubleSolenoid RampSolB {5,4};

  frc::DoubleSolenoid SuctionSol {6,7};
  frc::Compressor compressor {0};

#endif
  
  frc::ADXRS450_Gyro gyro{frc::SPI::kOnboardCS0};
  float gyroangle=0;


  std::shared_ptr<NetworkTable> table;

  std::shared_ptr<NetworkTable> CVtable;

  float currentArmPos = 0;
  float lastArmPos = 0;

 // frc::DriverStation& driverstation;

  //for driving straigt?
	float desiredAngle = 0.0;  //for PID control of angle...
	//bool holdAngle = false;
	frc::PIDController * anglePID;
	double anglePIDoutput = 0.0;
	std::shared_ptr<NetworkTable> PIDtable;
	//THESE NEED TO BE TWEAKED...
	float anglePID_P = -0.02;
	float anglePID_I = 0.0;//0.008;
	float anglePID_D = -0.01;  


};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
