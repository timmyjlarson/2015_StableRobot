#include "WPILib.h"
#include<string.h>
#include <time.h>
#include<iostream>

/**
 * 2501's 2015 Robot Code
 * Written by:
 * Tyler Seiford
 * Tim Larson
 * Jacob Ozel
 * Captain:
 * Kyle Ronsberg
 */

class Robot: public SampleRobot
{
private:
	enum ANALOG_IN
	{
		ANALOG_EMPTY_0 = 0,
		ANALOG_EMPTY_1,
		ANALOG_EMPTY_2,
		ANALOG_EMPTY_3
	};
	enum PWM_OUT
	{
		PWM_TALON_RIGHT_FRONT = 0,
		PWM_TALON_LEFT_FRONT,
		PWM_TALON_RIGHT_REAR,
		PWM_TALON_LEFT_REAR,
		PWM_EMPTY_4,
		PWM_TALON_ELEVATOR,
		PWM_TALON_ARMS,
		PWN_EMPTY_7,
		PWM_SERVO_CAMERA_PAN,
		PWM_SERVO_CAMERA_TILT
	};
	enum CAN
	{
		CAN_PNUEMATIC_CONTROL_MODULE = 0,
		CAN_TALON_RIGHT_FRONT = 1,
		CAN_TALON_LEFT_FRONT,
		CAN_TALON_RIGHT_REAR,
		CAN_TALON_LEFT_REAR
	};
	enum RELAY_OUT
	{
		RELAY_EMPTY_0 = 0,
		RELAY_EMPTY_1,
		RELAY_EMPTY_2,
		RELAY_EMPTY_3
	};
	enum DIO_PORTS
	{
		DIO_LIMIT_SWITCH_ELEVATOR_MAX = 0,
		DIO_LIMIT_SWITCH_ELEVATOR_MIN,
		DIO_LIMIT_SWITCH_ARMS_MAX,
		DIO_LIMIT_SWITCH_ARMS_MIN,
		DIO_EMPTY_4,
		DIO_EMPTY_5,
		DIO_EMPTY_6,
		DIO_EMPTY_7,
		DIO_EMPTY_8,
		DIO_EMPTY_9
	};
	enum PCM_SOLENOID_PORTS
	{
		PCM_EMPTY_0 = 0,
		PCM_DOUBLE_SOLENOID1,
		PCM_DOUBLE_SOLENOID2,
		PCM_EMPTY_3,
		PCM_EMPTY_4,
		PCM_EMPTY_5,
		PCM_EMPTY_6,
		PCM_EMPTY_7
	};
	enum JOYSTICK_ID
	{
		JOYSTICK_1 = 0,
		JOYSTICK_2,
		JOYSTICK_3
	};
	struct JoyStickState
	{								//	Control				Driver			Driver2
		bool TriggerPressed;		//------------------------------------------------------
		bool Button2Pressed;		//
		bool Button3Pressed;		//
		bool Button4Pressed;		//
		bool Button5Pressed;		//
		bool Button6Pressed;		//
		bool Button7Pressed;		//	Camera Pan
		bool Button8Pressed;		//	Camera Pan
		bool Button9Pressed;		//	Camera Tilt Down
		bool Button10Pressed;		//	Camera Tilt	Up		Solenoids
		bool Button11Pressed;		//	Elevator?			Solenoids
		bool Button12Pressed;		//	Elevator?			Solenoids
		bool YForwardPressed;		//	Elevator Up
		bool YReversePressed;		//	Elevator Down
	};

	RobotDrive driveSystem;
	CANTalon rightFrontDriveTalon, leftFrontDriveTalon, rightRearDriveTalon, leftRearDriveTalon;

	Compressor compressor;

	DoubleSolenoid solenoids;

	Joystick driverStick;
	Joystick driverStick2;
	Joystick controlStick;

	JoyStickState driverState;
	JoyStickState driverState2;
	JoyStickState controlState;

	Talon elevator;
	bool elevatorIsRunningUp;
	bool elevatorIsRunningDown;

	Talon arms;
	bool armsIsRunningUp;
	bool armsIsRunningDown;

	DigitalInput elevatorMaxLimitSwitch;
	bool elevatorMaxLimitSwitchIsOpen;

	DigitalInput elevatorMinLimitSwitch;
	bool elevatorMinLimitSwitchIsOpen;

	int elevatorMaxOldLimit, elevatorMaxNewLimit, elevatorMinNewLimit, elevatorMinOldLimit;	//UINT32

	const float SPIN_SPEED = 0.5;
	const float ELEVATOR_SPEED_UP = 0.5;
	const float ELEVATOR_SPEED_DOWN = -0.25;
	const float ARMS_SPEED = 0.5;
	const float ELEVATOR_NEUTRAL = 0.05;
	const float DRIVE_SPEED = 1.0;													//Only Change for kids :P

	const float AUTO_ELEVATOR_UP = -0.5;

	const float AJ_AUTO_FORWARD = 2.0;

	const int autoCode = 3;//				0 = Nothing		1 = Drive Forward		2 = Use below numbers for ADVANCED_AUTO		3 = AntiJerk_AUTO

	//                                      ADVANCED_AUTO
	//---------------------------------------------------------------------------------------------------------------------
	//										Open Arms & Drive Forward
	const float ADV_AUTO_FORWARD =				1.0;
	//										Stop, Close Arms
	const float ADV_AUTO_ARMS_OPEN = 			1.0;
	//										Move Elevator Up
	const float ADV_AUTO_ELEVATOR =				1.0;
	//										Turn											Tune
	const float ADV_AUTO_TURN =					0.6;
	//										Pause for a sec
	const float ADV_AUTO_WAIT =					0.5;
	//										Drive Forward into Auto Zone					Tune
	const float ADV_AUTO_FORWARD_2 =			1.0;

public:
	Robot() :
		driveSystem(leftFrontDriveTalon, leftRearDriveTalon, rightFrontDriveTalon, rightRearDriveTalon),
		rightFrontDriveTalon(CAN_TALON_RIGHT_FRONT),
		leftFrontDriveTalon(CAN_TALON_LEFT_FRONT),
		rightRearDriveTalon(CAN_TALON_RIGHT_REAR),
		leftRearDriveTalon(CAN_TALON_LEFT_REAR),
		compressor(CAN_PNUEMATIC_CONTROL_MODULE),
		solenoids(PCM_DOUBLE_SOLENOID1, PCM_DOUBLE_SOLENOID2),
		driverStick(JOYSTICK_1),
		driverStick2(JOYSTICK_2),
		controlStick(JOYSTICK_3),
		elevator(PWM_TALON_ELEVATOR),
		arms(PWM_TALON_ARMS),
		elevatorMaxLimitSwitch(DIO_LIMIT_SWITCH_ELEVATOR_MAX),
		elevatorMinLimitSwitch(DIO_LIMIT_SWITCH_ELEVATOR_MIN)
{
		driveSystem.SetExpiration(0.1);
		driveSystem.SetSafetyEnabled(false);
		driveSystem.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		driveSystem.SetInvertedMotor(RobotDrive::kRearRightMotor, true);

		CameraServer::GetInstance()->SetQuality(50);
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");
	}
	void initSolenoids()
	{
		compressor.SetClosedLoopControl(true);
	}
	void checkSolenoidButtons()
	{
		if (controlStick.GetRawButton(1))
		{
			if (!controlState.TriggerPressed)
			{
				controlState.TriggerPressed = true;
				solenoids.Set(DoubleSolenoid::kForward);
			}
		}
		else
		{
			if (controlState.TriggerPressed)
			{
				controlState.TriggerPressed = false;
			}
		}

		if (controlStick.GetRawButton(3))
		{
			if (controlState.Button3Pressed == false)
			{
				controlState.Button3Pressed = true;
				solenoids.Set(DoubleSolenoid::kOff);
			}
		}
		else
		{
			if (controlState.Button3Pressed)
			{
				controlState.Button3Pressed = false;
			}
		}

		if (controlStick.GetRawButton(2))
		{
			if (controlState.Button2Pressed == false)
			{
				controlState.Button2Pressed = true;
				solenoids.Set(DoubleSolenoid::kReverse);
			}
		}
		else
		{
			if (controlState.Button2Pressed)
			{
				controlState.Button2Pressed = false;
			}
		}
	}
	void initElevator()
	{
		elevatorIsRunningUp = 0;
		elevatorIsRunningDown = 0;
		elevator.Set(0.0);
	}
	float SignElevator(float f)
	{
		if (f < 0)
		{
			return (0.5 * f * f) + ELEVATOR_NEUTRAL;
		}
		else
		{
			return (-1 * f * f) + ELEVATOR_NEUTRAL;
		}
	}
	void checkElevatorAxis()
	{
		float elevatorNum = SignElevator(controlStick.GetY());
		elevator.Set(elevatorNum);

		if (elevatorNum > 0.04)
		{
			if (!controlState.YForwardPressed)
			{
				controlState.YForwardPressed = true;

				if (!elevatorIsRunningUp)
				{
					elevatorIsRunningUp = true;
				}
			}
		}
		else
		{
			if (controlState.YForwardPressed)
			{
				controlState.YForwardPressed = false;
				if (elevatorIsRunningUp)
				{
					elevatorIsRunningUp = false;
				}
			}
		}
		if (elevatorNum < -0.04)
		{
			if (!controlState.YReversePressed)
			{
				controlState.YReversePressed = true;

				if (!elevatorIsRunningDown)
				{
					elevatorIsRunningDown = true;
				}
			}
		}
		else
		{
			if (controlState.YReversePressed)
			{
				controlState.YReversePressed = false;
				if (elevatorIsRunningDown)
				{
					elevatorIsRunningDown = false;
				}
			}
		}
	}
	void debug() {}
	void initLimits()
	{
		elevatorMaxNewLimit = elevatorMaxLimitSwitch.Get();
		elevatorMaxOldLimit = elevatorMaxNewLimit;

		elevatorMinNewLimit = elevatorMinLimitSwitch.Get();
		elevatorMinOldLimit = elevatorMinNewLimit;
	}
	void checkLimits()
	{
		elevatorMaxNewLimit = elevatorMaxLimitSwitch.Get();
		if(elevatorMaxOldLimit == 0 && elevatorMaxNewLimit == 1)
		{
			elevatorIsRunningUp = false;
			elevatorIsRunningDown = false;
			elevator.Set(0.0);
		}
		elevatorMaxOldLimit = elevatorMaxNewLimit;

		elevatorMinNewLimit = elevatorMinLimitSwitch.Get();
		if(elevatorMinOldLimit == 0 && elevatorMinNewLimit == 1)
		{
			elevatorIsRunningUp = false;
			elevatorIsRunningDown = false;
			elevator.Set(0.0);
		}
		elevatorMinOldLimit = elevatorMinNewLimit;
	}
	float SignSquare(float f)
	{
		if (f < 0)
		{
			return -1.0 * f * f * DRIVE_SPEED;
		}
		else
		{
			return f * f * DRIVE_SPEED;
		}
	}
	float SignSpin(float f)
	{
		if (f < 0)
		{
			return -1.0 * f * f * SPIN_SPEED * DRIVE_SPEED;
		}
		else
		{
			return f * f * SPIN_SPEED * DRIVE_SPEED;
		}
	}
	void Autonomous()
	{
		driveSystem.SetSafetyEnabled(false);
		Wait(0.005);

		float auto_time = 0.005;

		driveSystem.MecanumDrive_Cartesian(0,0,0);
		if(autoCode == 0)
		{
			SmartDashboard::PutString("DB/String 0", "AUTO_CODE: 0");
			Wait(15.0);
			auto_time += 15;
		}
		else if(autoCode == 1)
		{
			SmartDashboard::PutString("DB/String 0", "AUTO_CODE: 1");
			driveSystem.MecanumDrive_Cartesian(0,-0.5,0);
			Wait(ADV_AUTO_FORWARD_2);
			auto_time += ADV_AUTO_FORWARD_2;
		}
		else if(autoCode == 2)
		{
			SmartDashboard::PutString("DB/String 0", "ADVANCED_AUTO");
			SmartDashboard::PutString("DB/String 1", "ACTIVE");

			driveSystem.MecanumDrive_Cartesian(0,-0.5,0);							//Forward + Open Arms
			solenoids.Set(DoubleSolenoid::kReverse);
			Wait(ADV_AUTO_FORWARD);
			auto_time += ADV_AUTO_FORWARD;

			driveSystem.MecanumDrive_Cartesian(0,0,0);								//Stop + Close Arms
			solenoids.Set(DoubleSolenoid::kForward);
			Wait(ADV_AUTO_ARMS_OPEN);
			auto_time += ADV_AUTO_ARMS_OPEN;

			elevator.Set(AUTO_ELEVATOR_UP);											//Elevator up a bit
			Wait(ADV_AUTO_ELEVATOR);
			auto_time += ADV_AUTO_ELEVATOR;

			elevator.Set(ELEVATOR_NEUTRAL);											//Set Elevator to Neutral
			driveSystem.MecanumDrive_Cartesian(0,0,0.5);							//Turn to face the AUTO_ZONE
			Wait(ADV_AUTO_TURN);
			auto_time += ADV_AUTO_TURN;

			driveSystem.MecanumDrive_Cartesian(0,0,0);								//Pause for a sec
			Wait(ADV_AUTO_WAIT);
			auto_time += ADV_AUTO_WAIT;

			driveSystem.MecanumDrive_Cartesian(0,-0.5,0);							//Drive into AUTO_ZONE
			Wait(ADV_AUTO_FORWARD_2);
			auto_time += ADV_AUTO_FORWARD_2;

			driveSystem.MecanumDrive_Cartesian(0,0,0);								//Stop in AUTO_ZONE
		}
		else if(autoCode == 3)
		{
			SmartDashboard::PutString("DB/String 0", "AntiJerk_AUTO");
			SmartDashboard::PutString("DB/String 1", "ACTIVE");

			elevator.Set(-0.5);
			Wait(0.25);
			auto_time += 0.25;
			elevator.Set(-0.07);

			solenoids.Set(DoubleSolenoid::kForward);
			Wait(1.0);
			auto_time += 1;

			elevator.Set(-0.5);
			Wait(1.5);
			auto_time += 1.5;
			elevator.Set(-0.07);

			driveSystem.MecanumDrive_Cartesian(0,0.75,0);							//Drive into AUTO_ZONE
			Wait(AJ_AUTO_FORWARD);
			auto_time += AJ_AUTO_FORWARD;

			driveSystem.MecanumDrive_Cartesian(0,0,0);								//Pause for a sec
			Wait(0.5);
			auto_time += 0.5;

			driveSystem.MecanumDrive_Cartesian(0,0,0.3);							//Turn to face the AUTO_ZONE
			Wait(1.2);
			auto_time += 1.2;
		}
		else
		{
			SmartDashboard::PutString("DB/String 0", "Invalid AUTO_CODE");
		}

		driveSystem.MecanumDrive_Cartesian(0,0,0);

		while (IsAutonomous() && IsEnabled() && auto_time < 15.0)					//Wait for the remainder of AUTONOMOUS
		{
			Wait(1.0);
			auto_time += 1;
		}

	}
	void OperatorControl()
	{
		driveSystem.SetSafetyEnabled(false);
		Wait(0.005);

		initElevator();
		initSolenoids();
		initLimits();

		while (IsOperatorControl() && IsEnabled())
		{
			float x = SignSquare(driverStick.GetX());
			float y = SignSquare(driverStick.GetY());
			float rotate = SignSpin(driverStick2.GetX());
			driveSystem.MecanumDrive_Cartesian(x,y,rotate);

			Wait(0.005);

			checkElevatorAxis();
			checkSolenoidButtons();
			checkLimits();
		}
	}
};
START_ROBOT_CLASS(Robot);
//THIS IS CODE SMART ONE
