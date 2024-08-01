#include "Thrustmaster1600.h"
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
 * Constructs an instance of the ThrustmasterController class.
 *
 * @param port The port on the driver station that the gamepad is plugged into.
 */
ThrustmasterController::
ThrustmasterController(unsigned int port)  :
   // Joystick(port)//, kNumAxes, kNumButtons)  ,
     m_deadBand(0.0f)
{
	stick = new frc::Joystick(port);

    ClearState();
    leftXOffset = leftYOffset = rightXOffset = rightYOffset = 0.0f;
	
}

ThrustmasterController::
~ThrustmasterController()
{ }

bool ThrustmasterController::
Calibrate(float max_offset)
{
	printf("Calibrate...\n");
/*
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

*/
	printf("Controller name:%s\n", stick->GetName().c_str());

}

void ThrustmasterController::
ClearState()
{
    size_t i;

    for (i = 0; i < (size_t)STATE_MAX; ++i)
        m_state[i] = false;
}

float ThrustmasterController::
GetStickX()
{
	float LX = stick->GetRawAxis(kXAxis);// - leftXOffset;
	if (fabs(LX) < m_deadBand)
	    return 0.0f;

	return LX;
}

float ThrustmasterController::
GetStickY()
{
	float LY = stick->GetRawAxis(kYAxis);// - leftYOffset;
    if (fabs(LY) < m_deadBand)
    	return 0.0f;

	return LY;
}

float ThrustmasterController::
GetStickZ()
{
	float RX = stick->GetRawAxis(kRotateAxis);// - rightXOffset;
	if (fabs(RX) < m_deadBand)
        return 0.0f;

	return RX;
}

float ThrustmasterController::
GetThrottle()
{
	float RY = stick->GetRawAxis(kThrottle);// - rightYOffset;
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
bool ThrustmasterController::
GetNumberedButton(unsigned int buttonNumber)
{
    return stick->GetRawButton(buttonNumber);
}

/**
 * Gets whether or not the left joystick is depressed.
 *
 * @return The state of the left axis joystick button.
 */
/*
bool ThrustmasterController::
GetLeftPush()
{
    return stick->GetRawButton(kLeftJoystickButton);
}

/**
 * Gets whether or not the right joystick is depressed.
 *
 * @return The state of the right axis joystick button.
 */
/*
bool ThrustmasterController::
GetRightPush()
{
    return stick->GetRawButton(kRightJoystickButton);
}

/**
 * Gets the state of the directional pad.
 * The left joystick cannot be used when the directional pad is active (i.e. the
 * red LED is illuminated). Press the mode button to toggle this behavior.
 *
 * @return The state of the directional pad.
 */

float ThrustmasterController::
GetDPad()
{
   return stick->GetPOV(0);
}

bool ThrustmasterController::
GetDPadUp()
{
	return (bool)(GetDPad() == 0.0f);

}

bool ThrustmasterController::
GetDPadRight()
{
	return (bool)(GetDPad() == 90.0f);
}

bool ThrustmasterController::
GetDPadDown()
{
	return (bool)(GetDPad() == 180.0f);
}

bool ThrustmasterController::
GetDPadLeft()
{
	return (bool)(GetDPad() == 270.0f);
}

/*
float ThrustmasterController::
GetLeftTrigger()
{
	return GetRawAxis(kLeftTrigger);
}

float ThrustmasterController::
GetRightTrigger()
{
	return GetRawAxis(kRightTrigger);
}
*/


int ThrustmasterController::
GetRawPOV(uint32_t pov)
{
	return stick->GetPOV(pov);//GetPOV(pov);
}

bool ThrustmasterController::
GetBtn(int btn)
{
	return stick->GetRawButton(btn);
}

/*

bool ThrustmasterController::
GetBtnB()
{
	return stick->GetRawButton(BTN_B);
}

bool ThrustmasterController::
GetBtnX()
{
	return stick->GetRawButton(BTN_X);
}

bool ThrustmasterController::
GetBtnY()
{
	return stick->GetRawButton(BTN_Y);
}

bool ThrustmasterController::
GetLeftBumper()
{
	return stick->GetRawButton(BTN_LB);
}

bool ThrustmasterController::
GetRightBumper()
{
	return stick->GetRawButton(BTN_RB);
}

bool ThrustmasterController::
GetBtnStart()
{
	return stick->GetRawButton(BTN_START);
}

bool ThrustmasterController::
GetBtnBack()
{
	return stick->GetRawButton(BTN_BACK);
}
*/




bool ThrustmasterController::
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

bool ThrustmasterController::
OnBtn(int btn) { return on_state(GetBtn(btn), (enum state_idx )btn); }

/*
bool ThrustmasterController::
OnLeftPush() { return on_state(GetLeftPush(), STATE_LEFT_PUSH); }
bool ThrustmasterController::
OnRightPush() { return on_state(GetRightPush(), STATE_RIGHT_PUSH); }

bool ThrustmasterController::
OnLeftBumper() { return on_state(GetLeftBumper(), STATE_LEFT_BUMPER); }
bool ThrustmasterController::
OnRightBumper() { return on_state(GetRightBumper(), STATE_RIGHT_BUMPER); }

bool ThrustmasterController::
OnStart() { return on_state(GetBtnStart(), STATE_BTN_START); }
bool ThrustmasterController::
OnBack() { return on_state(GetBtnBack(), STATE_BTN_BACK); }

bool ThrustmasterController::
OnA() { return on_state(GetBtnA(), STATE_BTN_A); }
bool ThrustmasterController::
OnB() { return on_state(GetBtnB(), STATE_BTN_B); }
bool ThrustmasterController::
OnX() { return on_state(GetBtnX(), STATE_BTN_X); }
bool ThrustmasterController::
OnY() { return on_state(GetBtnY(), STATE_BTN_Y); }

*/
bool ThrustmasterController::
OnDPadUp() { return on_state(GetDPadUp(), STATE_DPAD_UP); }
bool ThrustmasterController::
OnDPadDown() { return on_state(GetDPadDown(), STATE_DPAD_DOWN); }
bool ThrustmasterController::
OnDPadLeft() { return on_state(GetDPadLeft(), STATE_DPAD_LEFT); }
bool ThrustmasterController::
OnDPadRight() { return on_state(GetDPadRight(), STATE_DPAD_RIGHT); }



/* vim: set ts=4 sw=4 sts=4 expandtab: */
