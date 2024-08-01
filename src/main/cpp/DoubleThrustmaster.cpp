#include "DoubleThrustmaster.h"
#include "math.h"

#define BTN_A 1
#define BTN_B 2
#define BTN_X 3
#define BTN_Y 4
#define BTN_LB 5
#define BTN_RB 6
#define BTN_BACK 7
#define BTN_START 8

/**
 * Constructs an instance of the DoubleThrustmaster4 class.
 *
 * @param port The port on the driver station that the gamepad is plugged into.
 */
DoubleThrustmaster::
DoubleThrustmaster(unsigned int port1, unsigned int port2)
{
	m_deadBand = 0.0f;
	LJoystick = new frc::Joystick(port1);
	RJoystick = new frc::Joystick(port2);
    ClearState();
    leftXOffset = leftYOffset = rightXOffset = rightYOffset = 0.0f;
	printf("LEft Controller name:%s\n", LJoystick->GetName().c_str());
	printf("Right Controller name:%s\n", RJoystick->GetName().c_str());
	
}

DoubleThrustmaster::
~DoubleThrustmaster()
{ }

bool DoubleThrustmaster::
Calibrate(float max_offset)
{
	bool cal_ok = true;

	leftXOffset = LJoystick->GetRawAxis(kLeftXAxis);
	leftYOffset = LJoystick->GetRawAxis(kLeftYAxis);
	rightXOffset = RJoystick->GetRawAxis(kRightXAxis);
	rightYOffset = RJoystick->GetRawAxis(kRightYAxis);

	if(fabs(leftXOffset)>max_offset) {cal_ok = false;  leftXOffset = 0.0; printf("leftXOffset offset error\n");}
	if(fabs(leftYOffset)>max_offset) {cal_ok = false;  leftYOffset = 0.0; printf("leftYOffset offset error\n");}
	if(fabs(rightXOffset)>max_offset) {cal_ok = false;  rightXOffset = 0.0; printf("rightXOffset offset error\n");}
	if(fabs(rightYOffset)>max_offset) {cal_ok = false;  rightYOffset = 0.0; printf("rightYOffset offset error\n");}

	return cal_ok;

}

void DoubleThrustmaster::
ClearState()
{
    size_t i;

    for (i = 0; i < (size_t)STATE_MAX; ++i)
        m_state[i] = false;
}

float DoubleThrustmaster::
GetLeftX()
{
	float LX = LJoystick->GetRawAxis(kLeftXAxis) - leftXOffset;
	if (fabs(LX) < m_deadBand)
	    return 0.0f;

	return LX;
}

float DoubleThrustmaster::
GetLeftY()
{
	float LY = LJoystick->GetRawAxis(kLeftYAxis) - leftYOffset;
    if (fabs(LY) < m_deadBand)
    	return 0.0f;

	return LY;
}

float DoubleThrustmaster::
GetRightX()
{
	float RX = RJoystick->GetRawAxis(kRightXAxis) - rightXOffset;
	if (fabs(RX) < m_deadBand)
        return 0.0f;

	return RX;
}

float DoubleThrustmaster::
GetRightY()
{
	float RY = RJoystick->GetRawAxis(kRightYAxis) - rightYOffset;
	if (fabs(RY) < m_deadBand)
        return 0.0f;

	return RY;
}

/**
 * Gets the state of the button with the given number.
 *
 * @param buttonNumber The number of the button.
 * @return The state of the button.
 */
bool DoubleThrustmaster::
GetNumberedButton(unsigned int buttonNumber)
{
    return LJoystick->GetRawButton(buttonNumber);
}

/**
 * Gets whether or not the left joystick is depressed.
 *
 * @return The state of the left axis joystick button.
 */
bool DoubleThrustmaster::
GetLeftPush()
{
    return LJoystick->GetRawButton(kLeftJoystickButton);
}

/**
 * Gets whether or not the right joystick is depressed.
 *
 * @return The state of the right axis joystick button.
 */
bool DoubleThrustmaster::
GetRightPush()
{
    return LJoystick->GetRawButton(kRightJoystickButton);
}

/**
 * Gets the state of the directional pad.
 * The left joystick cannot be used when the directional pad is active (i.e. the
 * red LED is illuminated). Press the mode button to toggle this behavior.
 *
 * @return The state of the directional pad.
 */
float DoubleThrustmaster::
GetDPad()
{
   return RJoystick->GetPOV(0);

}

bool DoubleThrustmaster::
GetDPadUp()
{
	return (bool)(GetDPad() == 0.0f);

}

bool DoubleThrustmaster::
GetDPadRight()
{
	return (bool)(GetDPad() == 90.0f);
}

bool DoubleThrustmaster::
GetDPadDown()
{
	return (bool)(GetDPad() == 180.0f);
}

bool DoubleThrustmaster::
GetDPadLeft()
{
	return (bool)(GetDPad() == 270.0f);
}

float DoubleThrustmaster::
GetLeftTrigger()
{
	return LJoystick->GetRawAxis(kLeftTrigger);
}

float DoubleThrustmaster::
GetRightTrigger()
{
	return RJoystick->GetRawAxis(kRightTrigger);
}

int DoubleThrustmaster::
GetRawPOV(uint32_t pov)
{
	return RJoystick->GetPOV(pov);
}

bool DoubleThrustmaster::
GetBtnA()
{
	return LJoystick->GetRawButton(BTN_A);
}

bool DoubleThrustmaster::
GetBtnB()
{
	return LJoystick->GetRawButton(BTN_B);
}

bool DoubleThrustmaster::
GetBtnX()
{
	return LJoystick->GetRawButton(BTN_X);
}

bool DoubleThrustmaster::
GetBtnY()
{
	return LJoystick->GetRawButton(BTN_Y);
}

/*
bool DoubleThrustmaster::
GetLeftBumper()
{
	return GetRawButton(BTN_LB);
}

bool DoubleThrustmaster::
GetRightBumper()
{
	return GetRawButton(BTN_RB);
}

bool DoubleThrustmaster::
GetBtnStart()
{
	return GetRawButton(BTN_START);
}

bool DoubleThrustmaster::
GetBtnBack()
{
	return GetRawButton(BTN_BACK);
}
*/
bool DoubleThrustmaster::
on_state(bool raw, enum state_idx idx)
{
    if (raw && !m_state[idx]) {
        m_state[idx] = true;
        return true;
    }

    if (!raw)
    	m_state[idx] = false;

    return false;
}

/* State Checking versions */
bool DoubleThrustmaster::
OnLeftPush() { return on_state(GetLeftPush(), STATE_LEFT_PUSH); }
bool DoubleThrustmaster::
OnRightPush() { return on_state(GetRightPush(), STATE_RIGHT_PUSH); }

/*bool DoubleThrustmaster::
OnLeftBumper() { return on_state(GetLeftBumper(), STATE_LEFT_BUMPER); }
bool DoubleThrustmaster::
OnRightBumper() { return on_state(GetRightBumper(), STATE_RIGHT_BUMPER); }

bool DoubleThrustmaster::
OnStart() { return on_state(GetBtnStart(), STATE_BTN_START); }
bool DoubleThrustmaster::
OnBack() { return on_state(GetBtnBack(), STATE_BTN_BACK); }
*/
bool DoubleThrustmaster::
OnA() { return on_state(GetBtnA(), STATE_BTN_A); }
bool DoubleThrustmaster::
OnB() { return on_state(GetBtnB(), STATE_BTN_B); }
bool DoubleThrustmaster::
OnX() { return on_state(GetBtnX(), STATE_BTN_X); }
bool DoubleThrustmaster::
OnY() { return on_state(GetBtnY(), STATE_BTN_Y); }

bool DoubleThrustmaster::
OnDPadUp() { return on_state(GetDPadUp(), STATE_DPAD_UP); }
bool DoubleThrustmaster::
OnDPadDown() { return on_state(GetDPadDown(), STATE_DPAD_DOWN); }
bool DoubleThrustmaster::
OnDPadLeft() { return on_state(GetDPadLeft(), STATE_DPAD_LEFT); }
bool DoubleThrustmaster::
OnDPadRight() { return on_state(GetDPadRight(), STATE_DPAD_RIGHT); }

/* vim: set ts=4 sw=4 sts=4 expandtab: */
