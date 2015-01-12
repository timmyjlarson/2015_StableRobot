#include "WPILib.h"
#include <time.h>

/**
 * 2501's 2015 Robot Code
 * Written by:
 * Tyler Seiford
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
		PWM_EMPTY_6,
		PWM_EMPTY_7,
		PWM_EMPTY_8,
		PWM_EMPTY_9,
		PWM_EMPTY_10,
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
		DIO_EMPTY_1 = 1,
		DIO_EMPTY_2,
		DIO_EMPTY_3,
		DIO_EMPTY_4,
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
		JOYSTICK_1 = 1,
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

	const float SPIN_SPEED = 0.5;
	const float ELEVATOR_SPEED = 0.5;

public:
	Robot() :
		driveSystem(PWM_TALON_RIGHT_FRONT,PWM_TALON_LEFT_FRONT,PWM_TALON_RIGHT_REAR,PWM_TALON_LEFT_REAR),
		driverStick(JOYSTICK_1),
		driverStick2(JOYSTICK_2),
		controlStick(JOYSTICK_3),
		elevator(PWM_TALON_ELEVATOR)
	{
		SmartDashboard::PutString("DB/String 0", "(: Hello World! :)");
		SmartDashboard::PutString("DB/String 1", "Written by: ");
		SmartDashboard::PutString("DB/String 2", "FRC Team 2501");

		driveSystem.SetExpiration(0.1);

		elevatorIsRunningUp = 0;
		elevatorIsRunningDown = 0;
		elevator.Set(0.0);
		//printf("Matt Cassin is Awesome!");
	}

	void checkElevatorButtons()
	{
		if (controlStick.GetY() > 0.5)
		{
			if (!controlState.YForwardPressed)
			{
				controlState.YForwardPressed = true;

				if (!elevatorIsRunningUp)
				{
					elevatorIsRunningUp = true;
					elevator.Set(ELEVATOR_SPEED);
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
					elevator.Set(0.0);
				}
			}
		}
		if (controlStick.GetY() < -0.5)
		{
			if (!controlState.YReversePressed)
			{
				controlState.YReversePressed = true;

				if (!elevatorIsRunningDown)
				{
					elevatorIsRunningDown = true;
					elevator.Set(ELEVATOR_SPEED * -1);
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
					elevator.Set(0.0);
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


	void OperatorControl()
	{
		driveSystem.SetSafetyEnabled(true);
		Wait(0.005);

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
		}
	}
};

START_ROBOT_CLASS(Robot);
//THIS IS CODE SMART ONE
