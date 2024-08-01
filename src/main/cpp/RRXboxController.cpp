#include "RRXboxController.h"
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
 * Constructs an instance of the RRXboxController class.
 *
 * @param port The port on the driver station that the gamepad is plugged into.
 */
RRXboxController::
RRXboxController(unsigned int port)
    : Joystick(port)//, kNumAxes, kNumButtons)
    , m_deadBand(0.0f)
{
    ClearState();
    leftXOffset = leftYOffset = rightXOffset = rightYOffset = 0.0f;
	printf("Controller name:%s\n", GetName().c_str());
}

RRXboxController::
~RRXboxController()
{ }

bool RRXboxController::
Calibrate(float max_offset)
{
	bool cal_ok = true;

	leftXOffset = GetRawAxis(kLeftXAxis);
	leftYOffset = GetRawAxis(kLeftYAxis);
	rightXOffset = GetRawAxis(kRightXAxis);
	rightYOffset = GetRawAxis(kRightYAxis);

	if(fabs(leftXOffset)>max_offset) {cal_ok = false;  leftXOffset = 0.0; printf("leftXOffset offset error\n");}
	if(fabs(leftYOffset)>max_offset) {cal_ok = false;  leftYOffset = 0.0; printf("leftYOffset offset error\n");}
	if(fabs(rightXOffset)>max_offset) {cal_ok = false;  rightXOffset = 0.0; printf("rightXOffset offset error\n");}
	if(fabs(rightYOffset)>max_offset) {cal_ok = false;  rightYOffset = 0.0; printf("rightYOffset offset error\n");}

	return cal_ok;

}

void RRXboxController::
ClearState()
{
    size_t i;

    for (i = 0; i < (size_t)STATE_MAX; ++i)
        m_state[i] = false;
}

float RRXboxController::
GetLeftX()
{
	float LX = GetRawAxis(kLeftXAxis) - leftXOffset;
	if (fabs(LX) < m_deadBand)
	    return 0.0f;

	return LX;
}

float RRXboxController::
GetLeftY()
{
	float LY = GetRawAxis(kLeftYAxis) - leftYOffset;
    if (fabs(LY) < m_deadBand)
    	return 0.0f;

	return LY;
}

float RRXboxController::
GetRightX()
{
	float RX = GetRawAxis(kRightXAxis) - rightXOffset;
	if (fabs(RX) < m_deadBand)
        return 0.0f;

	return RX;
}

float RRXboxController::
GetRightY()
{
	float RY = GetRawAxis(kRightYAxis) - rightYOffset;
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
bool RRXboxController::
GetNumberedButton(unsigned int buttonNumber)
{
    return GetRawButton(buttonNumber);
}

/**
 * Gets whether or not the left joystick is depressed.
 *
 * @return The state of the left axis joystick button.
 */
bool RRXboxController::
GetLeftPush()
{
    return GetRawButton(kLeftJoystickButton);
}

/**
 * Gets whether or not the right joystick is depressed.
 *
 * @return The state of the right axis joystick button.
 */
bool RRXboxController::
GetRightPush()
{
    return GetRawButton(kRightJoystickButton);
}

/**
 * Gets the state of the directional pad.
 * The left joystick cannot be used when the directional pad is active (i.e. the
 * red LED is illuminated). Press the mode button to toggle this behavior.
 *
 * @return The state of the directional pad.
 */
float RRXboxController::
GetDPad()
{
   return GetPOV(0);

}

bool RRXboxController::
GetDPadUp()
{
	return (bool)(GetDPad() == 0.0f);

}

bool RRXboxController::
GetDPadRight()
{
	return (bool)(GetDPad() == 90.0f);
}

bool RRXboxController::
GetDPadDown()
{
	return (bool)(GetDPad() == 180.0f);
}

bool RRXboxController::
GetDPadLeft()
{
	return (bool)(GetDPad() == 270.0f);
}

float RRXboxController::
GetLeftTrigger()
{
	return GetRawAxis(kLeftTrigger);
}

float RRXboxController::
GetRightTrigger()
{
	return GetRawAxis(kRightTrigger);
}

int RRXboxController::
GetRawPOV(uint32_t pov)
{
	return GetPOV(pov);
}

bool RRXboxController::
GetBtnA()
{
	return GetRawButton(BTN_A);
}

bool RRXboxController::
GetBtnB()
{
	return GetRawButton(BTN_B);
}

bool RRXboxController::
GetBtnX()
{
	return GetRawButton(BTN_X);
}

bool RRXboxController::
GetBtnY()
{
	return GetRawButton(BTN_Y);
}

bool RRXboxController::
GetLeftBumper()
{
	return GetRawButton(BTN_LB);
}

bool RRXboxController::
GetRightBumper()
{
	return GetRawButton(BTN_RB);
}

bool RRXboxController::
GetBtnStart()
{
	return GetRawButton(BTN_START);
}

bool RRXboxController::
GetBtnBack()
{
	return GetRawButton(BTN_BACK);
}

bool RRXboxController::
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
bool RRXboxController::
OnLeftPush() { return on_state(GetLeftPush(), STATE_LEFT_PUSH); }
bool RRXboxController::
OnRightPush() { return on_state(GetRightPush(), STATE_RIGHT_PUSH); }

bool RRXboxController::
OnLeftBumper() { return on_state(GetLeftBumper(), STATE_LEFT_BUMPER); }
bool RRXboxController::
OnRightBumper() { return on_state(GetRightBumper(), STATE_RIGHT_BUMPER); }

bool RRXboxController::
OnStart() { return on_state(GetBtnStart(), STATE_BTN_START); }
bool RRXboxController::
OnBack() { return on_state(GetBtnBack(), STATE_BTN_BACK); }

bool RRXboxController::
OnA() { return on_state(GetBtnA(), STATE_BTN_A); }
bool RRXboxController::
OnB() { return on_state(GetBtnB(), STATE_BTN_B); }
bool RRXboxController::
OnX() { return on_state(GetBtnX(), STATE_BTN_X); }
bool RRXboxController::
OnY() { return on_state(GetBtnY(), STATE_BTN_Y); }

bool RRXboxController::
OnDPadUp() { return on_state(GetDPadUp(), STATE_DPAD_UP); }
bool RRXboxController::
OnDPadDown() { return on_state(GetDPadDown(), STATE_DPAD_DOWN); }
bool RRXboxController::
OnDPadLeft() { return on_state(GetDPadLeft(), STATE_DPAD_LEFT); }
bool RRXboxController::
OnDPadRight() { return on_state(GetDPadRight(), STATE_DPAD_RIGHT); }

/* vim: set ts=4 sw=4 sts=4 expandtab: */
