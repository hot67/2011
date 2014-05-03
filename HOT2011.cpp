#include "WPILib.h"
#include "Gamepad.h"
#include "Math.h"
#include "Jaysif.h"
#include "HotPID.h"



class HotBot2011 : public IterativeRobot
{
	//Declare system for Drive and Lift
	Jaguar *m_lDrive1;
	Jaguar *m_rDrive1;

	Jaguar *m_lDrive2;
	Jaguar *m_rDrive2;
	
	Jaguar *m_Lift1;
	Jaguar *m_Lift2;
	
	//Declare Arm motors
	Victor *m_Arm1;
	Victor *m_Arm2;
	
	//Declare minibot deploy
	Servo *m_Mini1;
	Servo *m_Mini2;
	
	//Declare relays
	Relay *m_Gripper;
	Relay *m_Shifter;
	
	//Declare light sensors
	DigitalInput *m_Left;
	DigitalInput *m_Right;
	DigitalInput *m_Middle;
	
	//??????????????
	DigitalInput *m_MinibotSwitch;

	//Declare Potentiometer
	AnalogChannel *m_armPot;
	
	Gyro *m_HotGyro;
	
	//Declare drive system
	RobotDrive *m_robotDrive;		
	
	// Declare a variable to use to access the driver station object
	DriverStation *m_ds;						// driver station object
	DriverStationLCD *m_dsLCD;
	
	UINT32 m_priorPacketNumber;					// keep track of the most recent packet number from the DS
	UINT8 m_dsPacketsReceivedInCurrentSecond;	// keep track of the ds packets received in the current second
	
	// Declare variables for the two joysticks being used
	Gamepad *m_Gamepad1;
	Gamepad *m_Gamepad2;
	
	//Declare xbox controllers
	//Joystick *EcksBawks1;
	//Joystick *EcksBawks2;
	
	//Declare PIDs
	HotPID *m_liftPosition;
	HotPID *m_armPosition;
	HotPID *m_gyroTurn;
	
	//Declare encoders
	Encoder *m_Encoder1; 
	Encoder *m_Encoder2;
	Encoder *m_EncoderLift;
	
	
	//Declare misc. variables
	float m_JayTest;
	float m_TurnMani;
	float m_ArmMani;
	float m_LiftMani;
	float m_shiftcounter;
	float m_shiftreversecounter;
	float m_ArmPotVolt;
	float m_Encoder1Distance;
	float m_Encoder2Distance;
	float m_EncoderLiftDistance;
	int m_autonomousCase;
	float m_ININOUTOUT;
	int m_case3delaycounter;
	int m_LeftValue;
	int m_MidValue;
	int m_RightValue;
	int m_TotalValue;
	int m_InceptionCase;
	int m_Blindness;
	int m_FOIRK;
	int m_SwerveCount;
	int m_Counter;
	int m_OtherCounter;
	int m_GripperCounter;
	int m_shiftflag;
	int m_TMGT;
	int m_TopGyro;
	float m_LiftPosition;
	float m_BlindCount;
	int m_POKECOUNTER;
	int m_ULTRACOUNTER;
	int m_GREATCOUNTER;
	int m_MASTERCOUNTER;
	float m_Tester;
	signed int m_Negative;
	float m_GyroAngle;
	float m_TurtleCount;
	float m_IIE;
	float m_OldGyroValue;
	int m_dumbflag;
	int m_dumbflag2;
	int m_dumbflag3;
	int m_dumbflag4;
	int m_dumbflag5;
	int m_dumbflag6;
	int m_dumbflag7;
	int m_Position;
	int m_SwitchFlag;
	float m_randomflag;
	int m_MiniCount;
	float m_MiniWatch;
	float m_FlagFlag;
	float m_Pj;
	float m_Ij;
	float m_Dj;
	
	float m_Du1;
	float m_Du2;
	float m_Du3;
	float m_Du4;
	float m_Du5;
	float m_Du6;
	
	/*float LeftXAxis;
	float LeftYAxis;
	float RightXAxis;
	float RightXAxis;
	float TrigLXAxis;
	float LeftXAxis;//see table below for axis numbers
	bool Button01;*/
	
	//Joystick.getRawAxis(1);
	//Joystick.getRawButton(1);
	
	/*Axis indexes:
	1 - LeftX
	2 - LeftY
	3 - Triggers (Each trigger = 0 to 1, axis value = right - left)
	4 - RightX
	5 - RightY
	6 - DPad Left/Right*/
	
	// Declare variables for each of the eight solenoid outputs
	
	static const int NUM_SOLENOIDS = 8;
	Solenoid *m_solenoids[(NUM_SOLENOIDS+1)];

	enum {							// drive mode selection
		UNINITIALIZED_DRIVE = 0,
		ARCADE_DRIVE = 1,
		TANK_DRIVE = 2
	} m_driveMode;
	
	// Local variables to count the number of periodic loops performed
	UINT32 m_aukTOPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
		
public:
/**
 * Constructor for this "BuiltinDefaultCode" Class.
 * 
 * The constructor creates all of the objects used for the different inputs and outputs of
 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
 * providing named objects for each of the robot interfaces. 
 */
	HotBot2011(void)	{
		printf("BuiltinDefaultCode Constructor Started\n");

		//Initialize system for Drive and Lift
		m_lDrive1 = new Jaguar(1);
		m_rDrive1 = new Jaguar(2);
		m_lDrive2 = new Jaguar(3);
		m_rDrive2 = new Jaguar(4);
		
		m_Lift1 = new Jaguar(5);
		m_Lift2 = new Jaguar(6);
		
		//Initialize Arm motors
		m_Arm1 = new Victor(7);
		m_Arm2 = new Victor(8);
		
		//Initialize Minibot deploy servos
		m_Mini1 = new Servo(9);
		m_Mini2 = new Servo(10);
		
		//Initialize relays
		m_Gripper = new Relay(1);
		m_Shifter = new Relay(2);
		
		//?????????
		m_MinibotSwitch = new DigitalInput(7);
		
		//Initialize Potentionmeter
		m_armPot = new AnalogChannel(1);
		m_HotGyro = new Gyro(2);
		
		//Initialize Light Sensors
		m_Left = new DigitalInput(8);
		m_Right = new DigitalInput(10);
		m_Middle = new DigitalInput(9);
		
		//Initialize PIDs
		m_liftPosition = new HotPID(0.6,0.04,0.00,0.00);
		m_armPosition = new HotPID(0.95,0.01,0.00,0.02);
		m_gyroTurn = new HotPID(.047,11.51,0.0,0.022);
		
		//Set up drive system
		m_robotDrive = new RobotDrive(m_lDrive1, m_lDrive2, m_rDrive1, m_rDrive2); //This is where we invert
		
		// Acquire the Driver Station object
		m_ds = DriverStation::GetInstance();
		m_dsLCD = DriverStationLCD::GetInstance();
		m_priorPacketNumber = 0;
		m_dsPacketsReceivedInCurrentSecond = 0;

		// Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
		m_Gamepad1 = new Gamepad(1);
		m_Gamepad2 = new Gamepad(2);

		//Initialize encoders
		m_Encoder1 = new Encoder(1,2, true);
		m_Encoder1 -> SetDistancePerPulse((10.0*3.14)/360);
		m_Encoder1 -> SetMaxPeriod(1.0);
		m_Encoder1 -> Start();
		
		m_Encoder2 = new Encoder(3,4, false); 
		m_Encoder2 -> SetDistancePerPulse((10.0*3.14)/360);
		m_Encoder2 ->SetMaxPeriod(1.0);
		m_Encoder2 -> Start();
		
		m_EncoderLift = new Encoder(5,6, false);
		m_EncoderLift -> SetDistancePerPulse(1);
		m_EncoderLift -> SetMaxPeriod(1.0);
		m_EncoderLift -> Start();
		
		m_HotGyro -> Reset();
		
		//Initialize xbox controllers
		//EcksBawks1 = new Joystick(1);
		//EcksBawks2 = new Joystick(2);
		m_dumbflag = 0;
		m_dumbflag2 = 0;
		m_dumbflag3 = 0;
		m_dumbflag4 = 0;
		m_dumbflag5 = 0;
		m_dumbflag6 = 0;
		m_dumbflag7 = 0;
		m_randomflag = 0;
		m_TurtleCount = 0;
		m_GyroAngle = 0;
		m_ArmMani = 0;
		m_LiftMani = 0;
		m_TurnMani = 0;
		m_ArmPotVolt = 0;
		m_LeftValue = 2;
		m_MidValue = 2;
		m_RightValue = 2;
		m_TotalValue = 2;
		m_Negative = 1;
		m_autonomousCase = 1;
		m_InceptionCase = 1;
		m_Blindness = 1;
		m_TMGT = 1;
		m_TopGyro = 1;
		m_Tester = 0;
		m_FOIRK = 4; //4=Line Follow, 3=Low Gyro, 2=2 Tube, 1=High Gyro
		m_LiftPosition = 0;
		m_shiftflag = 0;
		m_ININOUTOUT = 0;
		m_BlindCount = 0;
		m_SwerveCount = 0;
		m_MiniCount = 0;
		m_MiniWatch = 0;
		m_FlagFlag = 2;
		m_Position = 0;
		m_SwitchFlag = 0;
		m_Counter = 0;
		m_OtherCounter = 0;
		m_GripperCounter = 0;
		m_shiftcounter = 0;
		m_shiftreversecounter = 0;
		m_OldGyroValue = 0;
		m_POKECOUNTER = 0;
		m_ULTRACOUNTER = 0;
		m_GREATCOUNTER = 0;
		m_MASTERCOUNTER = 0;
		m_Du1 = 0;
		m_Du2 = 0;
		m_Du3 = 0;
		m_Du4 = 0;
		m_Du5 = 0;
		m_Du6 = 0;
		m_IIE = 0;
		m_Pj = 0.067;
		m_Ij = 11.5;
		m_Dj = 0.0;
		m_JayTest = 0;
		
		
				// Set drive mode to uninitialized
		m_driveMode = UNINITIALIZED_DRIVE;
		//But not really
		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_aukTOPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;

		printf("BuiltinDefaultCode Constructor Completed\n");
	}

	/********************************** Init Routines *************************************/

	void RobotInit(void) 
	{
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.
		
		printf("RobotInit() completed.\n");
	}
	
	void DisabledInit(void) 
	{
		m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
		
		// Move the cursor down a few, since we'll move it back up in periodic.
		printf("\x1b[2B");
		
		//Set up PIDs
		m_liftPosition->Reset();
		m_liftPosition->SetSPLimits(+2000.0, -5.0);	// desired position (inches)
		m_liftPosition->SetPVLimits(+2000.0, -5.0);	// actual position (inches)
		m_liftPosition->SetMVLimits(+0.6, -0.6);	// Manipulated Variable, Jaguar drive
		m_liftPosition->Enable();
		
		m_armPosition->Reset();
		m_armPosition->SetSPLimits(+4.0, +1.8);	// desired position (inches)
		m_armPosition->SetPVLimits(+4.0, +1.8);	// actual position (inches)
		m_armPosition->SetMVLimits(+1.0, -1.0);	// Manipulated Variable, Jaguar drive
		m_armPosition->Enable();
		
		m_gyroTurn->Reset();
		m_gyroTurn->SetSPLimits(+90.0, -90.0);	// desired position (volts)
		m_gyroTurn->SetPVLimits(+90.0, -90.0);	// actual position (volts)
		m_gyroTurn->SetMVLimits(+1.0, -1.0);	// Manipulated Variable, Jaguar drive
		m_gyroTurn->Enable();
	}

	void AutonomousInit(void) 
	{
		RESETEVERYTHING();
		m_aukTOPeriodicLoops = 0; // Reset the loop counter for autonomous mode
		m_Encoder1Distance = 0;
		m_Encoder2Distance = 0;
		m_EncoderLiftDistance = 0;
		m_case3delaycounter = 0;
		
		
		m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "                     ");
		m_dsLCD->UpdateLCD();

		m_robotDrive->TankDrive(m_Gamepad1->GetLeftX(), m_Gamepad1->GetLeftX());
		
		//Reset encoders
		m_Encoder1->Reset();
		m_Encoder2->Reset();
		m_EncoderLift->Reset();
		m_armPosition->Reset();
		
		//Set up PIDs
		m_liftPosition->Reset();
		m_liftPosition->SetSPLimits(+2000.0, -5.0);	// desired position (inches)
		m_liftPosition->SetPVLimits(+2000.0, -5.0);	// actual position (inches)
		m_liftPosition->SetMVLimits(+0.6, -0.6);	// Manipulated Variable, Jaguar drive
		m_liftPosition->Enable();
		
		m_armPosition->Reset();
		m_armPosition->SetSPLimits(+4.0, +1.8);	// desired position (inches)
		m_armPosition->SetPVLimits(+4.0, +1.8);	// actual position (inches)
		m_armPosition->SetMVLimits(+1.0, -1.0);	// Manipulated Variable, Jaguar drive
		m_armPosition->Enable();
		
		m_gyroTurn->Reset();
		m_gyroTurn->SetSPLimits(+90.0, -90.0);	// desired position (volts)
		m_gyroTurn->SetPVLimits(+90.0, -90.0);	// actual position (volts)
		m_gyroTurn->SetMVLimits(+1.0, -1.0);	// Manipulated Variable, Jaguar drive
		m_gyroTurn->Enable();
	}

	void TeleopInit(void) 
	{
		RESETEVERYTHING();
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0;	// Reset the number of dsPackets in current second
		m_driveMode = UNINITIALIZED_DRIVE;		// Set drive mode to uninitialized
		
		//Reset encoders
		m_Encoder1->Reset();
		m_Encoder2->Reset();
		m_EncoderLift-> Reset();
		
		m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "                     ");
		m_dsLCD->UpdateLCD();
		
		//Set up PIDs
		m_liftPosition->Reset();
		m_liftPosition->SetSPLimits(+2000.0, -5.0);	// desired position (inches)
		m_liftPosition->SetPVLimits(+2000.0, -5.0);	// actual position (inches)
		m_liftPosition->SetMVLimits(+0.6, -0.6);	// Manipulated Variable, Jaguar drive
		m_liftPosition->Enable();
		
		m_armPosition->Reset();
		m_armPosition->SetSPLimits(+4.0, +1.8);	// desired position (inches)
		m_armPosition->SetPVLimits(+4.0, +1.8);	// actual position (inches)
		m_armPosition->SetMVLimits(+1.0, -1.0);	// Manipulated Variable, Jaguar drive
		m_armPosition->Enable();
		
		m_gyroTurn->Reset();
		m_gyroTurn->SetSPLimits(+90.0, -90.0);	// desired position (volts)
		m_gyroTurn->SetPVLimits(+90.0, -90.0);	// actual position (volts)
		m_gyroTurn->SetMVLimits(+1.0, -1.0);	// Manipulated Variable, Jaguar drive
		m_gyroTurn->Enable();
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  
	{
		// increment the number of disabled periodic loops completed
		m_disabledPeriodicLoops++;
		
		//Decide which Autonomous to run
		if(m_Gamepad1 -> GetButton01())
		{
			m_FOIRK = 1;//High Gyro
		}
		if(m_Gamepad1 -> GetButton02())
		{
			m_FOIRK = 2;//2 Tube
		}
		if(m_Gamepad1 -> GetButton03())
		{
			m_FOIRK = 3;//Low Gyro
		}
		if(m_Gamepad1 -> GetButton04())
		{
			m_FOIRK = 4;//Line Follow
		}
		if(m_Gamepad1 -> GetButton06() && m_SwitchFlag == 0)
		{
			m_Position = 1;
			m_SwitchFlag = 1;
		}
		else if (m_Gamepad1 -> GetButton05() && m_SwitchFlag == 1)
		{
			m_Position = 0;
			m_SwitchFlag = 0;
		}
		if(m_FOIRK == 1)
		{
			m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton: High Peg Gyro     ");
		}
		else if(m_FOIRK == 2)
		{
			m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton: 2 Tube            ");
		}
		else if(m_FOIRK == 3)
		{
			m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton: Low Peg Gyro      ");
		}
		else if(m_FOIRK == 4)
		{
			m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton: Line Follow      ");
		}
		if(m_Position == 0)
		{
			m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "POSITION: LEFT                                 ");
		}
		else if (m_Position == 1)
		{
			m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "POSITION:     RIGHT");
		}
		m_dsLCD->UpdateLCD();
	}

	void AutonomousPeriodic(void) 
	{	
		m_Mini1->Set(0);
		m_Mini2->Set(0);
		//Update Light Sensors
		m_LeftValue = m_Left -> Get() ? 0:1;
		m_MidValue = m_Middle->Get() ? 0:1;
		m_RightValue = m_Right->Get() ? 0:1;
		m_TotalValue = ((m_LeftValue * 4) + (m_MidValue * 2) + (m_RightValue * 1));
		
		m_aukTOPeriodicLoops++;
		
		//Update encoder distance
		m_Encoder1Distance = m_Encoder1 -> GetDistance();
		m_Encoder2Distance = m_Encoder2 -> GetDistance();
		m_EncoderLiftDistance = m_EncoderLift -> GetDistance();
		m_GyroAngle = m_HotGyro-> GetAngle();
		m_ArmPotVolt = m_armPot->GetVoltage();
		m_BlindCount = m_Blindness;
		m_TurtleCount = m_TMGT;
		m_IIE = m_HotGyro -> GetAngle();
		//Update Driver Station
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "AutoC:%f   ", m_TurtleCount);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Gyro:%f     ", m_GyroAngle);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "DIS1:%f     ",m_Encoder1Distance);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 7, "LIFT?:%f     ",m_randomflag);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "IIE:%f     ", m_EncoderLiftDistance);
	
		
		m_dsLCD->UpdateLCD(); 
		
#ifdef LUKAS
		
 	switch(m_autonomousCase)
	{
		case 1:
			m_Roller -> Set(Relay::kReverse);
			
			if(m_aukTOPeriodicLoops == 200)
			{
				m_Roller -> Set(Relay::kOff);
				m_autonomousCase = 2;
			}
		break;
		case 2:
			if (m_Encoder1Distance >= 0 && m_Encoder1Distance <= 219)
			{	
				m_robotDrive->Drive(0.45, 0.0);
			}
			else
			{
				m_robotDrive->Drive(0.0, 0.0);
				m_autonomousCase = 3;
			}
		
			if (m_Encoder1Distance >= 60 && m_Encoder1Distance <= 217)
			{
				m_Roller -> Set(Relay::kForward);
			}
			else
			{
				m_Roller -> Set(Relay::kOff);
			}
		break;
		case 3:
			m_case3delaycounter++;
			if (m_case3delaycounter == 150)
			{
				m_autonomousCase = 4;
			}
			else
			{
				m_robotDrive->Drive(0.0, 0.0);
				m_Roller->Set(Relay::kReverse);
			}
		break;
		case 4:
			if (m_Encoder1Distance >= 114 && m_Encoder1Distance <= 240)
			{	
				m_Roller->Set(Relay::kOff);
				m_robotDrive->Drive(-0.45, 0.0);
			}
			else
			{
				m_robotDrive->Drive(0.0,0.0);
			}
		break;
		}
#endif
 	
#ifdef LOLJAY	
 	if(m_FOIRK == 4)
 	{
#ifdef NewBot
		switch(m_autonomousCase)
		{
			case 1:
				
				m_ULTRACOUNTER++;
				if(m_ULTRACOUNTER < 15)
				{
					m_Gripper->Set(Relay::kForward);
				}
				else
				{
					m_Gripper->Set(Relay::kOff);
					ArmPosition(2.9);
				}
				if(m_Encoder1Distance < -10 )
				{
					m_robotDrive -> TankDrive(0.0,0.0);
				}
				if (m_Encoder1Distance < 175)
				{	
					DriveStraightForward(.6);
				}
				else
				{
					m_autonomousCase = 3;
				}
				if (m_Encoder1Distance > 30)
				{
					
					ManualLiftV2A(TOP, .11);
					
				}
				else					//CHERRY CHERRIES
				{						//JERRY
					ManualLiftV2A(FLOOR, 0); //LUKEJERIES MADE ME PUT FLOOR
				}
			break;
			case 3:
				DriveStraightForwardSlow();
				if (m_Encoder1Distance > 190)
				{
					m_autonomousCase = 4;
				}
				ManualLiftV2A(TOP, .11);
				ArmPosition(2.9);
			break;
			case 4:
				m_GripperCounter++;
				m_robotDrive->Drive(0.0, 0.0);
				ManualLiftV2A(TOP, .11);
				ArmPosition(2.9);
				m_Gripper->Set(Relay::kReverse);
				if (m_GripperCounter > 15)
				{
					m_Gripper->Set(Relay::kOff);
					m_autonomousCase = 5;
				}
			break;
			case 5:
				m_GripperCounter++;
				if (m_Encoder1Distance > 125 && m_Encoder1Distance < 240)
				{
					m_robotDrive -> TankDrive(-.5, -.5);
					if (m_GripperCounter > 157.5)
					{
						m_Lift1 -> Set(0);
						m_Lift2 -> Set(0);
						ArmPosition(2.5);
					}
					else
					{
						ManualLiftV2A(TOP, .11);
						ArmPosition(2.9);
					}
				}
				else
				{
					m_robotDrive->TankDrive(0.0,0.0);
				}
			break;
		}
 	}
 	else if(m_FOIRK == 1)
		{
 		switch(m_TopGyro)
 				{
 					case 1:
 						if(m_Encoder1Distance < -10 )
						{
							m_robotDrive -> TankDrive(0.0,0.0);
						}
 						m_ULTRACOUNTER++;
 						if(m_ULTRACOUNTER < 15)
 						{
 							m_Gripper->Set(Relay::kForward);
 						}
 						else
 						{
 							m_Gripper->Set(Relay::kOff);
 							ArmPosition(2.9);
 						}
 						
 						if(m_GyroAngle > 3 || m_GyroAngle < -3)
						{
							m_HotGyro -> Reset();
						}
 						
 						if (m_Encoder1Distance < 175)
 						{	
 							DriveDirectionGyro(0, .6, .1, 1);;
 						}
 						else
 						{
 							m_TopGyro = 3;
 						}
 						if (m_Encoder1Distance > 30)
 						{
 							
 							ManualLiftV2A(TOP, .11);
 							
 						}
 						else					//CHERRY CHERRIES
 						{						//JERRY
 							ManualLiftV2A(FLOOR, 0); //LUKEJERIES MADE ME PUT FLOOR
 						}
 					break;
 					case 3:
 						DriveDirectionGyro(0, .4, .1, 1);
 						if (m_Encoder1Distance > 190)
 						{
 							m_TopGyro = 4;
 						}
 						ManualLiftV2A(TOP, .11);
 						ArmPosition(2.9);
 					break;
 					case 4:
 						m_GripperCounter++;
 						m_robotDrive->Drive(0.0, 0.0);
 						ManualLiftV2A(TOP, .11);
 						ArmPosition(2.9);
 						m_Gripper->Set(Relay::kReverse);
 						if (m_GripperCounter > 15)
 						{
 							m_Gripper->Set(Relay::kOff);
 							m_TopGyro = 5;
 						}
 					break;
 					case 5:
 						m_GripperCounter++;
 						if (m_Encoder1Distance > 125 && m_Encoder1Distance < 240)
 						{
 							m_robotDrive -> TankDrive(-.5, -.5);
 							if (m_GripperCounter > 157.5)
 							{
 								m_Lift1 -> Set(0);
 								m_Lift2 -> Set(0);
 								ArmPosition(2.5);
 							}
 							else
 							{
 								ManualLiftV2A(TOP, .11);
 								ArmPosition(2.9);
 							}
 						}
 						else
 						{
 							m_robotDrive->TankDrive(0.0,0.0);
 						}
 					break;
 				}
	}
#endif 
	
#ifdef Blindness
 	else if (m_FOIRK == 3)
 	{
 		switch(m_Blindness)
 		{
 		case 1:		
 			if(m_Encoder1Distance < -10 )
			{
				m_robotDrive -> TankDrive(0.0,0.0);
			}
			m_ULTRACOUNTER++;
			if(m_ULTRACOUNTER < 15)
			{
				m_Gripper->Set(Relay::kForward);
			}
			else
			{
				m_Gripper->Set(Relay::kOff);
				ArmPosition(2.75);
			}
			
			if(m_GyroAngle > 3 || m_GyroAngle < -3)
			{
				m_HotGyro -> Reset();
			}
			
			if (m_Encoder1Distance > 30)
			{
				
				ManualLiftV2A(TOP, .11);
				
			}
			else					
			{						
				ManualLiftV2A(FLOOR, 0); 
			}
			
			if (m_Encoder1Distance < 160)
			{	
				DriveDirectionGyro(0, .6, .1, 1);
				
				if (m_Encoder1Distance > 180)
				{
					m_robotDrive -> TankDrive(0.0,0.0);
				}
			// This is my favorite part of the code
			}
			else
			{
				m_Blindness = 2;
			}
			
			
		break;
 		case 2:
 		/*	if (m_TotalValue == 0)
				{						*/
 			DriveDirectionGyro(0, .6, .1, 1);
															
					if (m_Encoder1Distance > 183)
					{
						m_robotDrive -> TankDrive(0.0,0.0);
						m_Blindness = 4;
					}
		/*			
				}
				else
				{
					m_Blindness = 4;
				}
 			ManualLiftV2A(TOP, .2);
 			ArmPosition(2.75);			*/
 		break;							
		case 3:
			DriveDirectionGyro(0, .6, .1, 1);
			if (m_Encoder1Distance > 183)
			{
				m_Blindness = 4;
			}
			ManualLiftV2A(TOP, .2);
			ArmPosition(2.75);
		break;
		case 4:
			m_GripperCounter++;
			m_robotDrive->Drive(0.0, 0.0);
			ManualLiftV2A(TOP, .11);
			ArmPosition(2.75);
			m_Gripper->Set(Relay::kReverse);
			if (m_GripperCounter > 15)
			{
				m_Gripper->Set(Relay::kOff);
				m_Blindness = 5;
			}
		break;
		case 5:
			m_GripperCounter++;
			if (m_Encoder1Distance > 120 && m_Encoder1Distance < 240)
			{
				DriveDirectionGyro(0, -.6, .1, 1);
				if(m_GyroAngle > 3 || m_GyroAngle < -3)
				{
					m_HotGyro ->Reset();
				}
				if (m_GripperCounter > 157.5)
				{
					ManualLiftV2T(MID, .11);
					ArmPosition(2.5);
				}
				else
				{
					ManualLiftV2A(TOP, .11);
					ArmPosition(2.75);
				}
			}
			else
			{
				m_Blindness = 6;
			}
		break;
		/*case 6:
			DriveDirectionGyro(54, 0.0, .8, 0);
			ArmPosition(ROUT);
			if(m_GyroAngle >= 54)
			{
				m_Encoder1->Reset();
				m_Blindness = 7;
			}
		break;
		case 7:
			DriveDirectionGyro(54, .6, .1, 1);
			if(m_Encoder1Distance > 26.5)
			{
				m_robotDrive->TankDrive(0.0,0.0);
				m_Blindness = 8;
			}
		break;
		case 8:
			m_GREATCOUNTER++;
			if(m_GREATCOUNTER < 20)
			{
				m_Gripper->Set(Relay::kForward);
			}
			else
			{
				m_Gripper->Set(Relay::kOff);
				ArmPosition(2.9);
			}
		break;		*/
			
 		}
 	}
#endif
#ifdef TMGT //DirDTS
 	else if (m_FOIRK == 2 && m_Position == 1)
 	{
 	switch(m_TMGT)
 			{
 				case 1:
 					m_ULTRACOUNTER++;
 					if(m_ULTRACOUNTER < 15)
 					{
 						m_Gripper->Set(Relay::kForward);
 					}
 					else
 					{
 						m_Gripper->Set(Relay::kOff);
 						ArmPosition(2.9);
 					}
 					
 					if(m_GyroAngle > 3 || m_GyroAngle < -3)
					{
						m_HotGyro -> Reset();
					}
 					
 					if (m_Encoder1Distance < 161)
 					{	
 						DriveDirectionGyro(0, .75, .1, 1);
 					}
 					else
 					{
 						m_TMGT = 3;
 					}
 					
 					if (m_Encoder1Distance > 15)
 					{
 						ManualLiftV2A(TOP, .11);
 					}
 					else				
 					{						
 						ManualLiftV2A(FLOOR, 0);
 					}
 					
 				break;
 				case 3:
 					DriveDirectionGyro(0, .75, .1, 1);
 					if (m_Encoder1Distance > 159)
 					{
 						m_TMGT = 4;
 					}
 					ManualLiftV2A(TOP, .11);
 					ArmPosition(2.9);
 					if(m_GyroAngle > 3 || m_GyroAngle < -3)
					{
						m_HotGyro ->Reset();
					}
 				break;
 				case 4:
 					if(m_dumbflag == 0)
 					{
 						m_HotGyro->Reset();
 					}
 					m_dumbflag=1;
 					m_GripperCounter++;
 					m_robotDrive->Drive(0.0, 0.0);
 					if(m_GripperCounter >= 20)
 					{
	 					m_Gripper->Set(Relay::kReverse);
	 					if (m_GripperCounter > 35)
	 					{
	 						m_Gripper->Set(Relay::kOff);
	 						m_TMGT = 5;
	 						m_dumbflag = 0;
	 					}
 					}
 					ManualLiftV2A(TOP, .11);
 					ArmPosition(2.9);
 					if(m_GyroAngle > 3 || m_GyroAngle < -3)
					{
						m_HotGyro ->Reset();
					}
 				break;
 				case 5:
 					/*if(m_dumbflag == 0)
					{
						m_HotGyro->Reset();
						m_dumbflag = 1;
					}*/
					m_GripperCounter++;
					if (m_Encoder1Distance > 10)
					{						
						DriveDirectionGyro(0, -.75, .1, 1);
						ArmPosition(2.9);
						if(m_GyroAngle > 3 || m_GyroAngle < -3)
						{
							m_HotGyro ->Reset();
						}
						if(m_Encoder1Distance < 145)
						{
							ManualLiftV2T(0.0 , 0.0);
							ArmPosition(2.5);
							m_randomflag++;
						}
					}	
					
					if (m_Encoder1Distance > -26 && m_Encoder1Distance < 10)
					{
						DriveDirectionGyro(0, -.4, .1, 1);
						ArmPosition(ROUT);
						if(m_GyroAngle > 3 || m_GyroAngle < -3)
						{
							m_HotGyro -> Reset();
						}
						ManualLiftV2T(FLOOR, 0.0);
					}
					else if(m_Encoder1Distance < -26)
					{
						m_TMGT = 6;
					}
 				break;
 				case 6:
 					if(m_dumbflag3 == 0)
 					{
 						m_HotGyro->Reset();
 					}
 					m_dumbflag3 = 1;
 					m_POKECOUNTER++;
 					if(m_POKECOUNTER > 33)
 					{
	 					//DriveDirectionGyro(35, 0.0, .95, 0);
 						GyroTurn(44);
	 					ManualLiftV2T(FLOOR, 0.0);
	 					ArmPosition(ROUT);
	 					if(m_gyroTurn -> OnTarget())
	 					{
	 						m_Encoder1->Reset();
	 						m_TMGT = 7;
	 					}
 					}
 					else
 					{
 						m_robotDrive->Drive(0.0, 0.0);
 					}
 					
 				break;
 				case 7:
 					if(m_dumbflag4 == 0)
 					{
 						m_HotGyro->Reset();
 					}
 					m_dumbflag4 = 1;
 					DriveDirectionGyro(0, .6, .1, 1);
 					/*if(m_GyroAngle > 3 || m_GyroAngle < -3)
					{
						m_HotGyro ->Reset();
					}*/
 					if(m_Encoder1Distance > 42)
 					{
 						m_robotDrive->TankDrive(0.0,0.0);
 						m_TMGT = 8;
 					}
 				break;
 				case 8:
 					if(m_dumbflag5 == 0)
 					{
 						m_HotGyro->Reset();
 					}
 					m_dumbflag5 = 1;
 					m_GREATCOUNTER++;
 					if(m_GREATCOUNTER < 25)
 					{
 						m_Gripper->Set(Relay::kForward);
 					}
 					else
 					{
 						m_Gripper->Set(Relay::kOff);
 						ArmPosition(2.75);
 						m_TMGT = 9;
 					}
 				break;
 				case 9:
 					if(m_dumbflag6 == 0)
 					{
 						m_HotGyro->Reset();
 						m_gyroTurn->SetGains(0.047, 11.51, 0.0);
 					}
 					m_dumbflag6 = 1;
 					m_GREATCOUNTER++;
 					if(m_GREATCOUNTER > 58)
 					{
	 					//DriveDirectionGyro(-32, 0.0, .95, 0);
 						GyroTurn(-44);
	 					ArmPosition(2.75);	
	 					if(m_gyroTurn -> OnTarget())
	 					{
	 						m_Encoder1->Reset();
	 						m_TMGT = 10;
	 					}
 					}
 				break; // I HATE BREAKS SO MUCH, EVERY CASE
 				case 10:
 					if(m_dumbflag7 == 0)
 					{
 						m_HotGyro->Reset();
 					}
 					m_dumbflag7 = 1;
 					DriveDirectionGyro(0, .75, .1, 1);
 					ArmPosition(2.75);
 					if(m_Encoder1Distance > 151)
 					{
 						m_TMGT = 11;
 					}
 					if (m_Encoder1Distance > 0)
 					{
 						
 						ManualLiftV2A(TOP, .11);
 						
 					}
 					else					//CHERRY CHERRIES
 					{						//JERRY
 						ManualLiftV2A(FLOOR, 0); //LUKEJERIES MADE ME PUT FLOOR
 					}
 					if(m_GyroAngle > 3 || m_GyroAngle < -3)
					{
						m_HotGyro ->Reset();
					}
 				break;
 				case 11:
 					m_MASTERCOUNTER++;
 					m_robotDrive->Drive(0.0, 0.0);
 					ArmPosition(2.75);
 					ManualLiftV2A(TOP, .11);
 					m_Gripper->Set(Relay::kReverse);
 					if (m_MASTERCOUNTER > 15)
 					{
 						m_Gripper->Set(Relay::kOff);
 						m_TMGT = 12;
 					}
 				break;
 				case 12:
 					m_MASTERCOUNTER++;
 					if(m_Encoder1Distance < 165)
 					{
 						m_robotDrive -> TankDrive(0.0, 0.0);
 					}
 					else
 					{
 						DriveDirectionGyro(0, -.6, .1, 1);
 					}
 					if (m_MASTERCOUNTER > 157.5)
					{
						ManualLiftV2T(0, .00);
						ArmPosition(2.5);
					}
					else
					{
						ManualLiftV2A(TOP, .11);
						ArmPosition(2.75);
					}
 			}
 	}
 	else if (m_FOIRK == 2 && m_Position == 0)
 	 	{
 	 	switch(m_TMGT)
 	 			{
 	 				case 1:
 	 					m_ULTRACOUNTER++;
 	 					if(m_ULTRACOUNTER < 15)
 	 					{
 	 						m_Gripper->Set(Relay::kForward);
 	 					}
 	 					else
 	 					{
 	 						m_Gripper->Set(Relay::kOff);
 	 						ArmPosition(2.9);
 	 					}
 	 					
 	 					if(m_GyroAngle > 3 || m_GyroAngle < -3)
 						{
 							m_HotGyro -> Reset();
 						}
 	 					
 	 					if (m_Encoder1Distance < 161)
 	 					{	
 	 						DriveDirectionGyro(0, .75, .1, 1);
 	 					}
 	 					else
 	 					{
 	 						m_TMGT = 3;
 	 					}
 	 					if (m_Encoder1Distance > 15)
 	 					{
 	 						
 	 						ManualLiftV2A(TOP, .11);
 	 						
 	 					}
 	 					else				
 	 					{						
 	 						ManualLiftV2A(FLOOR, 0);
 	 					}
 	 				break;
 	 				case 3:
 	 					DriveDirectionGyro(0, .75, .1, 1);
 	 					if (m_Encoder1Distance > 159)
 	 					{
 	 						m_TMGT = 4;
 	 					}
 	 					ManualLiftV2A(TOP, .11);
 	 					ArmPosition(2.9);
 	 					if(m_GyroAngle > 3 || m_GyroAngle < -3)
 						{
 							m_HotGyro ->Reset();
 						}
 	 				break;
 	 				case 4:
 	 					if(m_dumbflag == 0)
 	 					{
 	 						m_HotGyro->Reset();
 	 					}
 	 					m_dumbflag=1;
 	 					m_GripperCounter++;
 	 					m_robotDrive->Drive(0.0, 0.0);
 	 					if(m_GripperCounter >= 20)
 	 					{
 		 					m_Gripper->Set(Relay::kReverse);
 		 					if (m_GripperCounter > 35)
 		 					{
 		 						m_Gripper->Set(Relay::kOff);
 		 						m_TMGT = 5;
 		 						m_dumbflag = 0;
 		 					}
 	 					}
 	 					ManualLiftV2A(TOP, .11);
 	 					ArmPosition(2.9);
 	 					if(m_GyroAngle > 3 || m_GyroAngle < -3)
 						{
 							m_HotGyro ->Reset();
 						}
 	 				break;
 	 				case 5:
 	 					if(m_dumbflag == 0)
 						{
 							m_HotGyro->Reset();
 							m_dumbflag = 1;
 						}
 						m_GripperCounter++;
 						if (m_Encoder1Distance > 10)
 						{
 							
 							DriveDirectionGyro(0, -.75, .1, 1);
 							ArmPosition(2.9);
 							if(m_GyroAngle > 3 || m_GyroAngle < -3)
 							{
 								m_HotGyro ->Reset();
 							}
 							if(m_Encoder1Distance < 145)
 							{
 								ManualLiftV2T(0.0 , 0.0);
 								ArmPosition(2.5);
 								m_randomflag++;
 							}
 						}	
 						
 						if (m_Encoder1Distance > -26 && m_Encoder1Distance < 10)
 						{
 							DriveDirectionGyro(0, -.4, .1, 1);
 							ArmPosition(ROUT);
 							if(m_GyroAngle > 3 || m_GyroAngle < -3)
 							{
 								m_HotGyro -> Reset();
 							}
 							ManualLiftV2T(FLOOR, 0.0);
 						}
 						else if(m_Encoder1Distance < -26)
 						{
 							m_TMGT = 6;
 						}
 	 				break;
 	 				case 6:
 	 					if(m_dumbflag3 == 0)
 	 					{
 	 						m_HotGyro->Reset();
 	 					}
 	 					m_dumbflag3 = 1;
 	 					m_POKECOUNTER++;
 	 					if(m_POKECOUNTER > 33)
 	 					{
 		 					//DriveDirectionGyro(35, 0.0, .95, 0);
 	 						GyroTurn(-47);
 		 					ManualLiftV2T(FLOOR, 0.0);
 		 					ArmPosition(ROUT);
 		 					if(m_gyroTurn -> OnTarget())
 		 					{
 		 						m_Encoder1->Reset();
 		 						m_TMGT = 7;
 		 					}
 	 					}
 	 					else
 	 					{
 	 						m_robotDrive->Drive(0.0, 0.0);
 	 					}
 	 					
 	 				break;
 	 				case 7:
 	 					if(m_dumbflag4 == 0)
 	 					{
 	 						m_HotGyro->Reset();
 	 					}
 	 					m_dumbflag4 = 1;
 	 					DriveDirectionGyro(0, .6, .1, 1);
 	 					if(m_GyroAngle > 3 || m_GyroAngle < -3)
 						{
 							m_HotGyro ->Reset();
 						}
 	 					if(m_Encoder1Distance > 42)
 	 					{
 	 						m_robotDrive->TankDrive(0.0,0.0);
 	 						m_TMGT = 8;
 	 					}
 	 				break;
 	 				case 8:
 	 					if(m_dumbflag5 == 0)
 	 					{
 	 						m_HotGyro->Reset();
 	 					}
 	 					m_dumbflag5 = 1;
 	 					m_GREATCOUNTER++;
 	 					if(m_GREATCOUNTER < 25)
 	 					{
 	 						m_Gripper->Set(Relay::kForward);
 	 					}
 	 					else
 	 					{
 	 						m_Gripper->Set(Relay::kOff);
 	 						ArmPosition(2.75);
 	 						m_TMGT = 9;
 	 					}
 	 				break;
 	 				case 9:
 	 					if(m_dumbflag6 == 0)
 	 					{
 	 						m_HotGyro->Reset();
 	 						m_gyroTurn->SetGains(0.071, 11.51, 0.0);
 	 					}
 	 					m_dumbflag6 = 1;
 	 					m_GREATCOUNTER++;
 	 					if(m_GREATCOUNTER > 58)
 	 					{
 		 					//DriveDirectionGyro(-32, 0.0, .95, 0);
 	 						GyroTurn(45);
 		 					ArmPosition(2.75);	
 		 					if(m_gyroTurn -> OnTarget())
 		 					{
 		 						m_Encoder1->Reset();
 		 						m_TMGT = 10;
 		 					}
 	 					}
 	 				break; // I HATE BREAKS SO MUCH, EVERY CASE
 	 				case 10:
 	 					if(m_dumbflag7 == 0)
 	 					{
 	 						m_HotGyro->Reset();
 	 					}
 	 					m_dumbflag7 = 1;
 	 					DriveDirectionGyro(0, .75, .1, 1);
 	 					ArmPosition(2.75);
 	 					if(m_Encoder1Distance > 151)
 	 					{
 	 						m_TMGT = 11;
 	 					}
 	 					if (m_Encoder1Distance > 0)
 	 					{
 	 						
 	 						ManualLiftV2A(TOP, .11);
 	 						
 	 					}
 	 					else					//CHERRY CHERRIES
 	 					{						//JERRY
 	 						ManualLiftV2A(FLOOR, 0); //LUKEJERIES MADE ME PUT FLOOR
 	 					}
 	 					if(m_GyroAngle > 3 || m_GyroAngle < -3)
 						{
 							m_HotGyro ->Reset();
 						}
 	 				break;
 	 				case 11:
 	 					m_MASTERCOUNTER++;
 	 					m_robotDrive->Drive(0.0, 0.0);
 	 					ArmPosition(2.75);
 	 					ManualLiftV2A(TOP, .11);
 	 					m_Gripper->Set(Relay::kReverse);
 	 					if (m_MASTERCOUNTER > 15)
 	 					{
 	 						m_Gripper->Set(Relay::kOff);
 	 						m_TMGT = 12;
 	 					}
 	 				break;
 	 				case 12:
 	 					m_MASTERCOUNTER++;
 	 					if(m_Encoder1Distance < 165)
 	 					{
 	 						m_robotDrive -> TankDrive(0.0, 0.0);
 	 					}
 	 					else
 	 					{
 	 						DriveDirectionGyro(0, -.6, .1, 1);
 	 					}
 	 					if (m_MASTERCOUNTER > 157.5)
 						{
 							ManualLiftV2T(0, .00);
 							ArmPosition(2.5);
 						}
 						else
 						{
 							ManualLiftV2A(TOP, .11);
 							ArmPosition(2.75);
 						}
 	 			}
 	 	
	}
 	
#endif
#endif
	};
	
	void TeleopPeriodic(void) 
	{
		m_Mini1->Set(0);
		m_Mini2->Set(0);
		
		if(m_Gamepad1 -> GetDpadY() == 1)
		{
			m_JayTest = 1;
		}
		else if(m_Gamepad1 -> GetDpadY() == -1)
		{
			m_JayTest = 0;
		}
		
		m_telePeriodicLoops++; // increment the number of teleop periodic loops completed
		m_dsPacketsReceivedInCurrentSecond++;	// increment DS packets received
		if(m_Gamepad1 -> GetButton08() && m_Gamepad1 ->GetButton07()) //Start Diagnostic Mode
		{
			
			Diag();
		}
		else if(m_JayTest == 1)
		{
			if(m_Gamepad1 -> GetButton07())
			{
				m_gyroTurn->SetGains(m_Pj, m_Ij, m_Dj);
				m_FlagFlag = 1;
			}
			if(m_Gamepad1 -> GetButton08())
			{
				m_FlagFlag = 0;
			}
				
			SensorUpdator();
			Shift();
			TeleopDrive();
			JeleopDriverStationUpdate();
			TeleopGripper();
			TeleopLiftPosition();
			MiniDeploy();
			if(m_Gamepad1 -> GetButton01())
			{
				m_Pj = m_Pj +.01;
			}
			if(m_Gamepad1 -> GetButton02())
			{
				m_Pj = m_Pj -.01;
			}
			if(m_Gamepad1 -> GetButton03())
			{
				m_Ij = m_Ij +.01;
			}
			if(m_Gamepad1 -> GetButton04())
			{
				m_Ij = m_Ij -.01;
			}
			if(m_Gamepad1 -> GetButton05())
			{
				m_Dj = m_Dj +.01;
			}
			if(m_Gamepad1 -> GetButton06())
			{
				m_Dj = m_Dj -.01;
			}
			
			
		}
		else
		{		
		SensorUpdator();
		Shift();
		TeleopDrive();
		TeleopDriverStationUpdate();
		TeleopGripper();
		TeleopLiftPosition();
		MiniDeploy();
		/*
		}  // if (m_ds->GetPacketNumber()...
		*/
		}
	}; // TeleopPeriodic(void)
	
	void TeleopDrive(void)
	{
		if(m_Gamepad1 -> GetButton09())//WaltSpeed(sloooooooow)
		{
			GyroTurn(50);
			//m_robotDrive->ArcadeDrive(m_Gamepad1 -> GetLeftY(), (m_Gamepad1 -> GetRightY()));
			
		}
		else if(m_Gamepad1-> GetButton10())				
		{
			GyroTurn(0);
			//m_robotDrive->TankDrive(m_Gamepad1 -> GetLeftY(), -(m_Gamepad1 -> GetLeftY()));
		}
		else
		{
			
			
			m_robotDrive->ArcadeDrive(m_Gamepad1 -> GetLeftY(), (m_Gamepad1 -> GetRightY()));
			
			
		}
	}
	
	void SensorUpdator (void)
	{
		m_ININOUTOUT = m_Gamepad1 -> GetLeftY();
		//Set Variables for Encoder distances and Potentiometer voltage
		m_ArmPotVolt = m_armPot -> GetVoltage();
		m_Encoder1Distance = m_Encoder1 -> GetDistance();
		m_Encoder2Distance = m_Encoder2 -> GetDistance();
		m_EncoderLiftDistance = m_EncoderLift->GetDistance();
		m_GyroAngle = m_HotGyro->GetAngle();
		m_MiniWatch = m_MiniCount;
	}

	void TeleopDriverStationUpdate(void)
	{
		m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "DIS1:%f     ",m_Encoder1Distance);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 7, "Watch:%f     ",m_TurnMani);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "ArmPot:%f   ", m_ArmPotVolt);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "FF:%f     ", m_FlagFlag);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Gyro:%f     ", m_GyroAngle);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "BlackAndYellow: %f", m_ININOUTOUT);
		m_dsLCD->UpdateLCD(); 
	}
	void JeleopDriverStationUpdate(void)
	{
		m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "P:%f     ",m_Pj);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "I:%f     ",m_Ij);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "D:%f   ", m_Dj);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Gyro:%f     ", m_GyroAngle);
		m_dsLCD->UpdateLCD(); 
	}
	
	void TeleopGripper(void)
	{
		if(m_Gamepad2 -> GetRightX() < 0)
		{
			m_Gripper -> Set(Relay::kForward);
		}
		else if (m_Gamepad2 -> GetRightX() > 0)
		{
			m_Gripper -> Set(Relay::kReverse);
		}
		else
		{
			m_Gripper -> Set(Relay::kOff);
		}
	}
	void TeleopLiftPositionV2(void)
	{	
		if(m_Gamepad2 -> GetButton08())
		{
			m_EncoderLift->Reset();
		}
		
		m_Lift1 -> Set(-(m_Gamepad2 -> GetDpadX())); 	//Lift on Right Stick, Gamepad 2
		m_Lift2 -> Set(-(m_Gamepad2 -> GetDpadX()));
		
		if(m_Gamepad2 -> GetDpadY() == 1)
		{
			ArmPosition(2.8);
		}
		else if(m_Gamepad2 -> GetDpadY() == -1)
		{
			ArmPosition(1.95);
		}
		else 
		{
			m_Arm1 -> Set(-(m_Gamepad2 -> GetLeftY())/2);	//Arm on Left Stick, Gamepad 2
			m_Arm2 -> Set(-(m_Gamepad2 -> GetLeftY())/2);
		}
		
	}
	void TeleopLiftPosition (void)
	{	
		if(m_Gamepad2-> GetButton08())
		{
			m_EncoderLift->Reset();
		}
		//Call preset lift positions
		if (m_Gamepad2->GetButton03())
		{
			ManualLiftV3(0, 0);
			if(m_EncoderLiftDistance < 80)
			{
				m_EncoderLift-> Reset();
			}
		}
		else if (m_Gamepad2->GetButton01())
		{
			if(m_Gamepad2->GetButton06())
			{
				ManualLiftV3(LOW + 134, .11);
			}
			else 
			{
				ManualLiftV3(LOW, .11);
			}
		}
		else if (m_Gamepad2->GetButton02())
		{
			if(m_Gamepad2->GetButton06())
			{
				ManualLiftV3(MID + 134, .11);
			}
			else 
			{
				ManualLiftV3(MID, .11);
			}
		}
		else if (m_Gamepad2->GetButton04())
		{
			ManualLiftV3(TOP, .11);
		}
		else
		{
			m_Arm1 -> Set(-(m_Gamepad2 -> GetLeftY()));	//Arm on Left Stick, Gamepad 2
			m_Arm2 -> Set(-(m_Gamepad2 -> GetLeftY()));
					
			m_Lift1 -> Set(-(m_Gamepad2 -> GetDpadX())); 	//Lift on Right Stick, Gamepad 2
			m_Lift2 -> Set(-(m_Gamepad2 -> GetDpadX()));
		}
	}
	
	void DriveStraightForward(float Speed)
	{
		switch(m_TotalValue)
		{
		case 0:
			m_robotDrive -> TankDrive(0.0, 0.0); 
			/*
			 * SwerveCounter++;
			 * if(SwerveCounter < 40);
			 * {
			 * 		m_robotDrive -> ArcadeDrive(0.0, .4);
			 * }
			 * else if(SwerveCounter >= 40 && SwerveCounter <= 120)
			 * {
			 * 		m_robotDrive -> ArcadeDrive(0.0, -.4);
			 * }
			 * else
			 * {
			 * 		m_robotDrive -> TankDrive(0.0, 0.0);
			 * }
			 *
			 */
		break;
		case 1:
			m_robotDrive ->TankDrive (Speed + .04 , Speed - .08);
		break;
		case 2:
			m_robotDrive -> TankDrive(Speed, Speed);
		break;
		case 3:
			m_robotDrive ->TankDrive(Speed + .04, Speed - .08);
		break;
		case 4:
			m_robotDrive -> TankDrive (Speed - .08, Speed + .04);
		break;
		case 5:
			m_robotDrive -> TankDrive (0.0, 0.0);
		break;
		case 6:
			m_robotDrive -> TankDrive(Speed - .08, Speed + .04);
		break;
		case 7:
			m_robotDrive -> TankDrive (0.0, 0.0);
		break;
		}
	}
	
	void DriveStraightForwardSlow(void)
	{
		switch(m_TotalValue)
		{
		case 0:
			m_robotDrive ->TankDrive(0.0,0.0);
		break;
		case 1:
			m_robotDrive ->TankDrive (0.5 , 0.4);
		break;
		case 2:
			m_robotDrive -> TankDrive(.45, .45);
		break;
		case 3:
			m_robotDrive ->TankDrive(.60, .35);
		break;
		case 4:
			m_robotDrive -> TankDrive (0.4, 0.5);
		break;
		case 5:
			m_robotDrive -> TankDrive (0.35, 0.60);
		break;
		case 6:
			m_robotDrive -> TankDrive(.35, .60);
		break;
		case 7:
			m_robotDrive -> TankDrive (0.0, 0.0);
		break;
		}
	}

	void DriveStraightBackwards(void)
	{
		switch(m_TotalValue)
		{
		case 0:
			m_robotDrive ->Drive(0,0);
		break;
		case 1:
			m_robotDrive ->TankDrive (-.65 , -.50);
		break;
		case 2:
			m_robotDrive -> TankDrive(-.55, -.55);
		break;
		case 3:
			m_robotDrive ->TankDrive(-.65, -.50);
		break;
		case 4:
			m_robotDrive -> TankDrive (-.50, -.65);
		break;
		case 5:
			m_robotDrive -> TankDrive (0.0, 0.0);
		break;
		case 6:
			m_robotDrive -> TankDrive(-.50,-.65);
		break;
		case 7:
			m_robotDrive -> TankDrive (-.45, -.45);
		break;
		}
	}
	/*void SuiicideMissionNoCopyrights(void) //KeepSake, with no mass effect 2 rights infringed
	{
			switch(m_TotalValue)
			{
			case 0:
				m_robotDrive ->ArcadeDrive(.0,-1.0);
			break;
			case 1:
				m_robotDrive ->TankDrive(0.0,0.0);
			break;
			case 2:
				m_robotDrive ->TankDrive(0.0,0.0);
			break;
			case 3:
				m_robotDrive ->TankDrive(0.0,0.0);
			break;
			case 4:
				m_robotDrive ->TankDrive(0.0,0.0);
			break;
			case 5:
				m_robotDrive ->TankDrive(0.0,0.0);
			break;
			case 6:
				m_robotDrive ->TankDrive(0.0,0.0);
			break;
			case 7:
				m_robotDrive ->TankDrive(0.0,0.0);
			break;
			}	
		}*/
	void LiftPositionPID(float LiftPosition)
	{
		m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "DIS1:%f     ", LiftPosition);
		m_dsLCD->UpdateLCD();
		if(LiftPosition == (m_EncoderLift->GetDistance()))
		{
			m_Lift1 -> Set(0.0);
			m_Lift2 -> Set(0.0);
		}
		else
		{
		// Set the lift motors to move the lift to a position
		m_Lift1->Set(m_liftPosition->GetMV(LiftPosition,m_EncoderLift->GetDistance()));
		m_Lift2->Set(m_liftPosition->GetMV(LiftPosition,m_EncoderLift->GetDistance()));
		}
	}
	
	void ArmPosition (float ArmPosition)
	{
		JIDArm(ArmPosition);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "DIS1:%f     ", ArmPosition);
		m_dsLCD->UpdateLCD();
		m_ArmPotVolt = m_armPot -> GetVoltage();
		// Set the arm motors to move the arm to a position
		m_Arm1->Set(m_armPosition->GetMV(ArmPosition,m_armPot->GetVoltage()));
		m_Arm2->Set(m_armPosition->GetMV(ArmPosition,m_armPot->GetVoltage()));
		
	//	m_Arm1->Set(m_ArmMani);
	//	m_Arm2->Set(m_ArmMani);
		
	/*	if(m_EncoderLiftDistance < 0)
		{
			m_EncoderLift-> Reset();
		}
		if (m_ArmPotVolt > ArmPosition + .07)//Set lift
			{
				m_Arm1 -> Set(-0.3);
				m_Arm2 -> Set(-0.3);
			}
			else if (m_ArmPotVolt < ArmPosition - .07)
			{
				m_Arm1 -> Set(0.3);
				m_Arm2 -> Set(0.3);
			}
			else
			{
				m_Arm1 -> Set(0.0);
				m_Arm2 -> Set(0.0);		
			}		*/
			
	}
	
	void ManualLiftV3(float LiftV, float TAS)
	{
		//JIDLift(LiftV);
		
		if(m_EncoderLiftDistance < 0)
		{
			m_EncoderLift-> Reset();
		}
		if (m_EncoderLiftDistance > LiftV+90)//Set lift
		{
			m_Lift1 -> Set(-.65);
			m_Lift2 -> Set(-.65);
			ArmPosition(NIN);
		}
		else if (m_EncoderLiftDistance < LiftV-45)
		{
			m_Lift1 -> Set(1.0);
			m_Lift2 -> Set(1.0);
			ArmPosition(NIN);
		}
		else
		{
			m_Lift1 -> Set(TAS);
			m_Lift2 -> Set(TAS);
			
			if(m_EncoderLiftDistance < 80)
			{
				ArmPosition(ROUT);
			}
			else if(m_Gamepad2->GetButton05())
			{
				ArmPosition(2.8);
			}
			else if((m_Gamepad2->GetButton06()) && (m_Gamepad2 -> GetButton04()))
			{
				ArmPosition(NIN);
			}
			else
			{
				ArmPosition(OUT);
			}
		}
		
	}
	
	void ManualLiftV2T(float LiftV, float TAS)
	{
		//JIDLift(LiftV);
		
		if(m_EncoderLiftDistance < 0)
		{
			m_EncoderLift-> Reset();
		}
		if (m_EncoderLiftDistance > LiftV+90)//Set lift
			{
				m_Lift1 -> Set(-.6);
				m_Lift2 -> Set(-.6);
			}
			else if (m_EncoderLiftDistance < LiftV-45)
			{
				m_Lift1 -> Set(.95);
				m_Lift2 -> Set(.95);
			}
			else
			{
				m_Lift1 -> Set(TAS);
				m_Lift2 -> Set(TAS);
			}
	}
	
	void ManualLiftV2A(float LiftV, float TAS)
	{
		//JIDLift(LiftV);
		
	if(m_EncoderLiftDistance < 0)
		{
			m_EncoderLift -> Reset();
		}
		
		if (m_EncoderLiftDistance > LiftV + 900)//Set lift
			{
				m_Lift1 -> Set(-.3);
				m_Lift2 -> Set(-.3);
			}
			else if (m_EncoderLiftDistance < LiftV-45)
			{
				m_Lift1 -> Set(.92);
				m_Lift2 -> Set(.92);
			}
			else
			{
				m_Lift1 -> Set(TAS);
				m_Lift2 -> Set(TAS);
			}
	}
	
	void Diag (void)
	{
		if(m_Gamepad2 -> GetButton07())//Reverse direction
		{
			m_Negative = -1;
			if( (m_Gamepad2 ->GetDpadY()) == 1 )//Run gripper reversed
			{
				m_Gripper -> Set(Relay::kReverse);
			}
			else
			{
				m_Gripper -> Set(Relay::kOff);
			}
			
			if( (m_Gamepad2 ->GetDpadY()) == -1 )//Run shifter reversed
			{
				m_Shifter -> Set(Relay::kReverse);
			}
			else
			{
				m_Shifter -> Set(Relay::kOff);
			}
		}
		else
		{
			m_Negative = 1;
			if( (m_Gamepad2 ->GetDpadY()) == 1 )//Run gripper forward
			{
				m_Gripper -> Set(Relay::kForward);
			}
			else
			{
				m_Gripper -> Set(Relay::kOff);
			}
			
			if( (m_Gamepad2 ->GetDpadY()) == -1 )//Run shifter forward
			{
				m_Shifter -> Set(Relay::kForward);
			}
			else
			{
				m_Shifter -> Set(Relay::kOff);
			}
		}
		
		
		if(m_Gamepad2 -> GetButton05())//Run LeftDrive1
		{
			m_lDrive1 -> Set(1.0 * m_Negative);
		}
		else
		{
			m_lDrive1 -> Set(0.0 * m_Negative);
		}
		
		if(m_Gamepad2 -> GetButton06())//Run RightDrive1
		{
			m_rDrive1 -> Set(1.0 * m_Negative);
		}
		else
		{
			m_rDrive1 -> Set(0.0 * m_Negative);
		}
		
		if((m_Gamepad2 -> GetRightX()) > .1)//Run LeftDrive2
		{
			m_lDrive2 -> Set(1.0 * m_Negative);
		}
		else
		{
			m_lDrive2 -> Set(0.0 * m_Negative);
		}
		
		if((m_Gamepad2 -> GetRightX()) < -.1)//Run RightDrive2
		{
			m_rDrive2 -> Set(1.0 * m_Negative);
		}
		else
		{
			m_rDrive2 -> Set(0.0 * m_Negative);
		}
		
		if(m_Gamepad2 -> GetButton01())//Run Lift1
		{
			m_Lift1 -> Set(0.75 * m_Negative);
		}
		else
		{
			m_Lift1 -> Set(0.0 * m_Negative);
		}
		
		if(m_Gamepad2 -> GetButton02())//Run Lift2
		{
			m_Lift2 -> Set(0.75 * m_Negative);
		}
		else
		{
			m_Lift2 -> Set(0.0 * m_Negative);
		}
		
		if(m_Gamepad2 -> GetButton03())//Run Arm1
		{
			m_Arm1 -> Set(1.0 * m_Negative);
		}
		else
		{
			m_Arm1 -> Set(0.0 * m_Negative);
		}
		
		if(m_Gamepad2 -> GetButton04())//Run Arm2
		{
			m_Arm2 -> Set(1.0 * m_Negative);
		}
		else
		{
			m_Arm2 -> Set(0.0 * m_Negative);
		}
		
		if(m_Gamepad2 -> GetButton10())//Run minibot deploy 1
		{
			m_Mini1 -> Set(45.0);
		}
		else
		{
			m_Mini1 -> Set(0.0);
		}
			
		if(m_Gamepad2 ->GetButton09())//Run minibot deploy 2
		{
			m_Mini2 -> Set(45.0);
		}
		else
		{
			m_Mini2 -> Set(0.0);
		}
		m_Du1 = m_Encoder1 -> GetDistance();
		m_Du2 = m_HotGyro -> GetAngle();
		m_Du3 = m_Arm1 -> Get();
		m_Du4 = m_Arm1 -> Get();
		m_Du5 = m_Arm1 -> Get();
		m_Du6 = m_Arm1 -> Get();
		
		m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Du1:%f     ", m_Du1);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Du2:%f     ", m_Du2);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Du3:%f   ", m_Du3);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Du4:%f     ", m_Du4);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Du5:%f   ", m_Du5);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Du6:%f     ", m_Du6);
		m_dsLCD->UpdateLCD(); 
	}
void Shift (void)
{
	
	if(m_Gamepad1-> GetRightX() < 0)
	{
		m_Shifter -> Set (Relay::kForward); //into high
		
	}											//Left Low, Right High
	else if(m_Gamepad1-> GetRightX() > 0)
	{
			m_Shifter -> Set (Relay::kReverse); //into low
			
	}
	else
	{
		m_Shifter -> Set(Relay::kOff);
	}
	/*if((m_Gamepad1 -> GetRightX()) < 0)//If holding button 8, shift
	{
		m_shiftcounter++;
		m_shiftreversecounter = 0;
		if(m_shiftcounter > 50)
		{
			m_Shifter -> Set(Relay::kOff);
			
		}
		else
		{
			m_Shifter -> Set(Relay::kForward);
		}
		m_shiftflag = 1;
	}
	else if(m_shiftflag = 1)//Unshift after letting go of button 8
	{
		m_shiftreversecounter++;
		m_shiftcounter = 0;
		m_shiftflag = 0;
		if(m_shiftreversecounter > 50)
		{
			m_Shifter -> Set(Relay::kOff);
		}
		else
		{
			m_Shifter -> Set(Relay::kReverse);
		}
	}*/
}
void MiniDeploy (void)
{
	if(m_Gamepad1 -> GetButton06())
	{
		m_MiniCount++;
		if(m_MiniCount > 25)
		{
			m_Mini2 -> SetAngle(110);
		}
	}
	
	if(m_Gamepad1 -> GetButton05())
	{
		m_Mini1 -> SetAngle(90);
	}
	if(m_Gamepad1 -> GetButton02())
	{
		m_Mini2 -> SetAngle(110);
	}
}
void JIDArm (float SetPositionA)
{
	float ActualPosition = m_armPot->GetVoltage();
	m_ArmMani  = (ActualPosition - SetPositionA)/(SetPositionA);
}
void JIDLift (float SetPositionL)
{
	float ActualPosition = m_EncoderLift->GetDistance();
	m_LiftMani  = pow(((ActualPosition - SetPositionL)/(SetPositionL)), 1/3);
}
void JIDTurn (float SetPositionT)
{
	float ActualPosition = m_HotGyro -> GetAngle();
	m_TurnMani = pow(((ActualPosition - SetPositionT)/(SetPositionT)), 1/3);
}
void GyroTurn(float DesiredDirection)
{
	double turnspeed;
	turnspeed = -(m_gyroTurn->GetMV(DesiredDirection,m_GyroAngle));
	m_robotDrive->TankDrive(-turnspeed,turnspeed);
	//if(m_GyroAngle == DesiredDirection) m_robotDrive -> TankDrive(0.0,0.0);
	if(m_gyroTurn->OnTarget()) m_robotDrive -> TankDrive(0.0, 0.0);
	m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 8, "Watch:%f     ",m_TurnMani);
	m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 8, "FF:%f     ",m_FlagFlag);
	m_dsLCD->UpdateLCD();
	
	
	/*
	if(m_GyroAngle < DesiredDirection+1)
	{
		m_TurnMani = ((m_GyroAngle - DesiredDirection)/(DesiredDirection));
			if(m_TurnMani > .85)
			{
				m_robotDrive -> TankDrive(-m_TurnMani, m_TurnMani);
				m_FlagFlag = 1;
			}
			else
			{
				m_FlagFlag = 0;
				m_robotDrive -> TankDrive(.85, -.85);
			}
		
	}
	else if(m_GyroAngle > DesiredDirection-1)
	{
		m_TurnMani = ((m_GyroAngle - DesiredDirection)/(DesiredDirection));
		
			if(m_TurnMani < -.85)
			{
				m_robotDrive -> TankDrive(m_TurnMani, -m_TurnMani);
				m_FlagFlag = 1;
			}
			else
			{
				m_FlagFlag = 0;
				m_robotDrive -> TankDrive(-.85, .85);
			}
	}	
	else
	{
		m_robotDrive -> TankDrive(0.0,0.0);
	}
	*/
}
void DriveStraightEncoder(float Speed)
{
	float Encoder1Distance = m_Encoder1->GetDistance();
	float Encoder2Distance = m_Encoder2->GetDistance();
	if (Encoder1Distance+.1 > Encoder2Distance)
	{
		m_robotDrive -> TankDrive(Speed, Speed+.05);
	}
	else if (Encoder1Distance < Encoder2Distance+.1)
	{
		m_robotDrive -> TankDrive(Speed+.05, Speed);
	}
	else 
	{
		m_robotDrive -> TankDrive(Speed, Speed);
	}
}
void DriveDirectionGyro(float Direction, float Speed, float Turn, int Spin)
{	
	//1 = Nospin
	if(Spin == 1)
	{
		m_GyroAngle = m_HotGyro -> GetAngle();
		//float m_Direction = Direction;
		//m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Direc:%f     ", m_Direction);
		if (m_GyroAngle > Direction + .1)
		{
			m_robotDrive -> TankDrive(Speed, Speed + Turn);
		}
		else if (m_GyroAngle < Direction - .1)
		{
			m_robotDrive -> TankDrive(Speed + Turn, Speed);
		}
		else
		{
			m_robotDrive -> TankDrive(Speed, Speed);
		}
	}
	else
	{
		
		if (m_GyroAngle > Direction + .7)
		{
			m_robotDrive -> ArcadeDrive(0, 0 + Turn);
		}
		else if (m_GyroAngle < Direction - .7)
		{
			m_robotDrive -> ArcadeDrive(0, 0 - Turn);
		}
		else
		{
			m_robotDrive -> TankDrive(Speed, Speed);
		}
	}
}
void RESETEVERYTHING(void)
{
	m_HotGyro->SetSensitivity(.0033);
	m_ArmMani = 0;
	m_LiftMani = 0;
	m_ArmPotVolt = 0;
	m_LeftValue = 2;
	m_MidValue = 2;
	m_RightValue = 2;
	m_TotalValue = 2;
	m_Negative = 1;
	m_autonomousCase = 1;
	m_InceptionCase = 1;
	m_Blindness = 1;
	m_TMGT = 1;
	m_TopGyro = 1;
	m_Tester = 0;
	m_LiftPosition = 0;
	m_shiftflag = 0;
	m_ININOUTOUT = 0;
	m_Counter = 0;
	m_OtherCounter = 0;
	m_GripperCounter = 0;
	m_shiftcounter = 0;
	m_shiftreversecounter = 0;
	m_ULTRACOUNTER = 0;
	m_GREATCOUNTER = 0;
	m_MASTERCOUNTER = 0;
	m_Du1 = 0;
	m_Du2 = 0;
	m_Du3 = 0;
	m_Du4 = 0;
	m_Du5 = 0;
	m_Du6 = 0;
	m_dumbflag = 0;
	m_dumbflag2 = 0;
	m_dumbflag3 = 0;
	m_dumbflag4 = 0;
	m_dumbflag5 = 0;
	m_dumbflag6 = 0;
	m_dumbflag7 = 0;
	m_HotGyro -> Reset();
	m_MiniCount = 0;
}
		//Patch 11.1.2.2
};

START_ROBOT_CLASS(HotBot2011);
