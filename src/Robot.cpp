#include "WPILib.h"
#include <time.h>

/**
 * 2501's 2015 Robot Code
 * Written by:
 * Tyler Seiford
 * Tim Larson
 * Jacob Ozel
 * Edited by: Kyle Ronsberg
 */

class Robot: public SampleRobot
{
private:
	enum ANALOG_IN
	{
		ANALOG_EMPTY_1 = 1,
		ANALOG_EMPTY_2,
		ANALOG_EMPTY_3,
		ANALOG_EMPTY_4,
		ANALOG_EMPTY_5,
		ANALOG_EMPTY_6,
		ANALOG_EMPTY_7,
		ANALOG_EMPTY_8
	};
	enum PWM_OUT
	{
		PWM_TALON_RIGHT_FRONT = 1,
		PWM_TALON_LEFT_FRONT,
		PWM_TALON_RIGHT_REAR,
		PWM_TALON_LEFT_REAR,
		PWM_TALON_ELEVATOR,
		PWM_TALON_ARMS,
		PWM_EMPTY_7,
		PWM_SERVO_CAMERA_PAN,
		PWM_SERVO_CAMERA_TILT,
		PWN_EMPTY_10,
	};
	enum CAN
	{
		CAN_TALON_RIGHT_FRONT = 1,
		CAN_TALON_LEFT_FRONT,
		CAN_TALON_RIGHT_REAR,
		CAN_TALON_LEFT_REAR,
		CAN_TALON_ELEVATOR,
		CAN_EMPTY_6,
		CAN_EMPTY_7,
		CAN_EMPTY_8,
		CAN_EMPTY_9,
		CAN_EMPTY_10,
	};
	enum RELAY_OUT
	{
		RELAY_EMPTY_1 = 1,
		RELAY_EMPTY_2,
		RELAY_EMPTY_3,
		RELAY_EMPTY_4,
		RELAY_EMPTY_5,
		RELAY_EMPTY_6,
		RELAY_EMPTY_7,
		RELAY_EMPTY_8
	};
	enum DIO_PORTS
	{
		DIO_LIMIT_SWITCH_ELEVATOR_MAX = 1,
		DIO_LIMIT_SWITCH_ELEVATOR_MIN,
		DIO_LIMIT_SWITCH_ARMS_MAX,
		DIO_LIMIT_SWITCH_ARMS_MIN,
		DIO_EMPTY_5,
		DIO_EMPTY_6,
		DIO_EMPTY_7,
		DIO_EMPTY_8,
		DIO_EMPTY_9,
		DIO_EMPTY_10,
		DIO_EMPTY_11,
		DIO_EMPTY_12,
		DIO_EMPTY_13,
		DIO_EMPTY_14
	};
	enum SOLENOID_PORTS
	{
		SOLENOID_EMPTY_1 =1,
		SOLENOID_EMPTY_2,
		SOLENOID_EMPTY_3,
		SOLENOID_EMPTY_4,
		SOLENOID_EMPTY_5,
		SOLENOID_EMPTY_6,
		SOLENOID_EMPTY_7,
		SOLENOID_EMPTY_8
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
		bool Button7Pressed;		//
		bool Button8Pressed;		//
		bool Button9Pressed;		//
		bool Button10Pressed;		//
		bool Button11Pressed;		//
		bool Button12Pressed;		//
		bool YForwardPressed;		//	Elevator Up
		bool YReversePressed;		//	Elevator Down
		bool Axis5Forward;			//	Camera Down?
		bool Axis5Backward;			//	Camera Up?
		bool Axis6Forward;			//	Camera Right?
		bool Axis6Backward;			//	Camera Left?
	};

	RobotDrive driveSystem;

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

	DigitalInput armsMaxLimitSwitch;
	bool armsMaxLimitSwitchIsOpen;

	DigitalInput armsMinLimitSwitch;
	bool armsMinLimitSwitchIsOpen;

	float cameraTiltAngle;
	float cameraTiltMinAngle;
	float cameraTiltMaxAngle;
	Servo cameraTilt;

	float cameraPanAngle;
	float cameraPanMinAngle;
	float cameraPanMaxAngle;
	Servo cameraPan;

	const float SPIN_SPEED = 0.5;
	const float ELEVATOR_SPEED = 0.5;
	const float ARMS_SPEED = 0.5;

public:
	Robot() :
		driveSystem(PWM_TALON_RIGHT_FRONT,PWM_TALON_LEFT_FRONT,PWM_TALON_RIGHT_REAR,PWM_TALON_LEFT_REAR),
		driverStick(JOYSTICK_1),
		driverStick2(JOYSTICK_2),
		controlStick(JOYSTICK_3),
		elevator(PWM_TALON_ELEVATOR),
		arms(PWM_TALON_ARMS),
		elevatorMaxLimitSwitch(DIO_LIMIT_SWITCH_ELEVATOR_MAX),
		elevatorMinLimitSwitch(DIO_LIMIT_SWITCH_ELEVATOR_MIN),
		armsMaxLimitSwitch(DIO_LIMIT_SWITCH_ARMS_MAX),
		armsMinLimitSwitch(DIO_LIMIT_SWITCH_ARMS_MIN),
		cameraTilt(PWM_SERVO_CAMERA_TILT),
		cameraPan(PWM_SERVO_CAMERA_PAN)
	{
		//SmartDashboard::PutString("DB/String 0", "(: Hello World! :)");
		//SmartDashboard::PutString("DB/String 1", "Written by: ");
		//SmartDashboard::PutString("DB/String 2", "FRC Team 2501");

		driveSystem.SetExpiration(0.1);

		//printf("Matt Cassin is Awesome!");
	}
	void initCamera()
	{
		cameraTiltMinAngle = cameraTilt.GetMinAngle();
		cameraTiltMaxAngle = cameraTilt.GetMaxAngle();
		cameraTiltAngle = cameraTiltMinAngle;
		cameraTilt.SetAngle(cameraTiltAngle);

		cameraPanMinAngle = cameraPan.GetMinAngle();
		cameraPanMaxAngle = cameraPan.GetMaxAngle();
		cameraPanAngle = cameraPanMinAngle;
		cameraPan.SetAngle(cameraPanAngle);
	}
	void checkCameraTiltButtons()
	{
		if (controlStick.GetRawButton(9)) // tilt down
		{
			if (!controlState.Button9Pressed)
			{
				controlState.Button9Pressed = true;
				if (cameraTiltAngle != cameraTiltMinAngle)
				{
					cameraTiltAngle -= 10;
					if (cameraTiltAngle < cameraTiltMinAngle) cameraTiltAngle = cameraTiltMinAngle;

					cameraTilt.SetAngle(cameraTiltAngle);
				}
			}
		}
		else
		{
			if (controlState.Button9Pressed)
			{
				controlState.Button9Pressed = false;
			}
		}
		if (controlStick.GetRawButton(10)) // tilt up
		{
			if (!controlState.Button10Pressed)
			{
				controlState.Button10Pressed = true;
				if (cameraTiltAngle != cameraTiltMaxAngle)
				{
					cameraTiltAngle += 10;
					if (cameraTiltAngle > cameraTiltMaxAngle) cameraTiltAngle = cameraTiltMaxAngle;
						cameraTilt.SetAngle(cameraTiltAngle);
				}
			}
		}
		else
		{
			if (controlState.Button10Pressed)
			{
				controlState.Button10Pressed = false;
			}
		}
	}
	void checkCameraPanButtons()
	{
		if (controlStick.GetRawButton(7)) // Pan down
		{
			if (!controlState.Button7Pressed)
			{
				controlState.Button7Pressed = true;
				if (cameraPanAngle != cameraPanMinAngle)
				{
					cameraPanAngle -= 10;
					if (cameraPanAngle < cameraPanMinAngle) cameraPanAngle = cameraPanMinAngle;

					cameraPan.SetAngle(cameraPanAngle);
				}
			}
		}
		else
		{
			if (controlState.Button7Pressed)
			{
				controlState.Button7Pressed = false;
			}
		}
		if (controlStick.GetRawButton(8)) // Pan up
		{
			if (!controlState.Button8Pressed)
			{
				controlState.Button8Pressed = true;
				if (cameraPanAngle != cameraPanMaxAngle)
				{
					cameraPanAngle += 10;
					if (cameraPanAngle > cameraPanMaxAngle) cameraPanAngle = cameraPanMaxAngle;
						cameraPan.SetAngle(cameraPanAngle);
				}
			}
		}
		else
		{
			if (controlState.Button8Pressed)
			{
				controlState.Button8Pressed = false;
			}
		}
	}
	void initElevator()
	{
		elevatorIsRunningUp = 0;
		elevatorIsRunningDown = 0;
		elevator.Set(0.0);
	}
	void checkElevatorButtons()
	{
		if (controlStick.GetRawButton(12))
		{
			if (!controlState.Button12Pressed)
			{
				controlState.Button12Pressed = true;

				if (!elevatorIsRunningUp)
				{
					elevatorIsRunningUp = true;
					elevator.Set(ELEVATOR_SPEED);
				}
			}
		}
		else
		{
			if (controlState.Button12Pressed)
			{
				controlState.Button12Pressed = false;
				if (elevatorIsRunningUp)
				{
					elevatorIsRunningUp = false;
					elevator.Set(0.0);
				}
			}
		}
		if (controlStick.GetRawButton(11))
		{
			if (!controlState.Button11Pressed)
			{
				controlState.Button11Pressed = true;

				if (!elevatorIsRunningDown)
				{
					elevatorIsRunningDown = true;
					elevator.Set(ELEVATOR_SPEED * -1);
				}
			}
		}
		else
		{
			if (controlState.Button11Pressed)
			{
				controlState.Button11Pressed = false;
				if (elevatorIsRunningDown)
				{
					elevatorIsRunningDown = false;
					elevator.Set(0.0);
				}
			}
		}
	}
	void debug()
	{
		if(elevatorIsRunningDown)
		{
			SmartDashboard::PutString("DB/String 0", "DOWN TRUE");
		}
		else
		{
			SmartDashboard::PutString("DB/String 0", "DOWN FALSE");
		}
		if(elevatorIsRunningUp)
		{
			SmartDashboard::PutString("DB/String 1", "UP TRUE");
		}
		else
		{
			SmartDashboard::PutString("DB/String 1", "UP FALSE");
		}
		if(controlStick.GetRawButton(12))
		{
			SmartDashboard::PutString("DB/String 2", "12 ON");
		}
		else
		{
			SmartDashboard::PutString("DB/String 2", "12 OFF");
		}
		if(controlStick.GetRawButton(11))
		{
			SmartDashboard::PutString("DB/String 3", "11 ON");
		}
		else
		{
			SmartDashboard::PutString("DB/String 3", "11 OFF");
		}
	}
	void initArms()
	{
		armsIsRunningUp = 0;
		armsIsRunningDown = 0;
		arms.Set(0.0);
	}
	void checkArmsButtons()
	{
		if (controlStick.GetRawButton(1))
		{
			if (!controlState.TriggerPressed)
			{
				controlState.TriggerPressed = true;

				if (!armsIsRunningUp)
				{
					armsIsRunningUp = true;
					arms.Set(ARMS_SPEED);
				}
			}
		}
		else
		{
			if (controlState.TriggerPressed)
			{
				controlState.TriggerPressed = false;

				if (armsIsRunningUp)
				{
					armsIsRunningUp = false;
					arms.Set(0.0);
				}
			}
		}
		if (controlStick.GetRawButton(2))
		{
			if (!controlState.Button2Pressed)
			{
				controlState.Button2Pressed = true;

				if (!armsIsRunningDown)
				{
					armsIsRunningDown = true;
					arms.Set(ARMS_SPEED * -1);
				}
			}
		}
		else
		{
			if (controlState.Button2Pressed)
			{
				controlState.Button2Pressed = false;

				if (armsIsRunningDown)
				{
					armsIsRunningDown = false;
					arms.Set(0.0);
				}
			}
		}
	}
	float SignSquare(float f)
	{
		if (f < 0)
		{
			return -1.0 * f * f;
		}
		else
		{
			return f * f;
		}
	}
	float Twist(float f)
	{
		if (f < 0)
		{
			return -1.0 * f * f * SPIN_SPEED;
		}
		else
		{
			return f * f * SPIN_SPEED;
		}
	}
	void Autonomous()
	{
		driveSystem.SetSafetyEnabled(false);
		driveSystem.MecanumDrive_Cartesian(0,0,0);
		Wait(2.0);
	}
	void OperatorControl()
	{
		int elevatorMaxOldLimit, elevatorMaxNewLimit, elevatorMinNewLimit, elevatorMinOldLimit, armsMaxOldLimit, armsMaxNewLimit, armsMinOldLimit, armsMinNewLimit;	//UINT32

		driveSystem.SetSafetyEnabled(true);
		Wait(0.005);

		initElevator();
		initArms();
		initCamera();

		elevatorMaxNewLimit = elevatorMaxLimitSwitch.Get();
		elevatorMaxOldLimit = elevatorMaxNewLimit;

		elevatorMinNewLimit = elevatorMinLimitSwitch.Get();
		elevatorMinOldLimit = elevatorMinNewLimit;

		armsMaxNewLimit = armsMaxLimitSwitch.Get();
		armsMaxOldLimit = armsMaxNewLimit;

		armsMinNewLimit = armsMinLimitSwitch.Get();
		armsMinOldLimit = armsMinNewLimit;

		//SmartDashboard::PutString("DB/String 6", "MyRobot -- LEFT Front %d\n", PWM_TALON_LEFT_FRONT);
		//SmartDashboard::PutString("DB/String 7", "MyRobot -- RIGHT Front %d\n", PWM_TALON_RIGHT_FRONT);
		//SmartDashboard::PutString("DB/String 8", "MyRobot -- LEFT Rear %d\n", PWM_TALON_LEFT_REAR);
		//SmartDashboard::PutString("DB/String 9", "MyRobot -- RIGHT Rear %d\n", PWM_TALON_RIGHT_REAR);

		while (IsOperatorControl() && IsEnabled())
		{
			float x = SignSquare(driverStick.GetX());
			float y = SignSquare(driverStick.GetY());
			float rotate = Twist(driverStick2.GetX());
			driveSystem.MecanumDrive_Cartesian(x,y,rotate);

			Wait(0.005);

			checkElevatorButtons();
			checkArmsButtons();
			checkCameraTiltButtons();
			checkCameraPanButtons();

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

			armsMaxNewLimit = armsMaxLimitSwitch.Get();
			if(armsMaxOldLimit == 0 && armsMaxNewLimit == 1)
			{
				armsIsRunningUp = false;
				armsIsRunningDown = false;
				arms.Set(0.0);
			}
			armsMaxOldLimit = armsMaxNewLimit;

			armsMinNewLimit = armsMinLimitSwitch.Get();
			if(armsMinOldLimit == 0 && armsMinNewLimit == 1)
			{
				armsIsRunningUp = false;
				armsIsRunningDown = false;
				arms.Set(0.0);
			}
			armsMinOldLimit = armsMinNewLimit;
		}
	}
};

START_ROBOT_CLASS(Robot);
//THIS IS CODE SMART ONE
