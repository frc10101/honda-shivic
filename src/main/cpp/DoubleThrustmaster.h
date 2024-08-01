#ifndef H_TM1600_CONTROLLER
#define H_TM1600_CONTROLLER

#include <frc/Joystick.h>

/**
 * Handles input from a Logitech Gamepad connected to the Driver Station.
 * This class is a convenience class for the Logitech Dual Action Gamepad. It
 * wraps the Joystick class to provide methods to access the inputs on the
 * gamepad.
 */
class DoubleThrustmaster /*: public frc::Joystick */ {
    public:

        typedef enum {
            kCenter,
            kUp,
            kUpLeft,
            kLeft,
            kDownLeft,
            kDown,
            kDownRight,
            kRight,
            kUpRight
        } DPadDirection;

        DoubleThrustmaster(unsigned int port1, unsigned int port2);
        virtual ~DoubleThrustmaster();

        bool Calibrate(float max_offset = 0.3);

        int GetRawPOV(uint32_t pov);
        float GetLeftX(void);
        float GetLeftY(void);
        float GetLeftTrigger(void);
        float GetRightTrigger(void);
        float GetRightX(void);
        float GetRightY(void);

        float GetDeadBand() { return m_deadBand; };
        void SetDeadBand(float db) { m_deadBand = db; };

        bool GetNumberedButton(unsigned int buttonNumber);

        bool GetLeftPush();
        bool GetRightPush();
        bool GetLeftBumper();
        bool GetRightBumper();
        bool GetBtnStart();
        bool GetBtnBack();
        bool GetBtnA();
        bool GetBtnB();
        bool GetBtnX();
        bool GetBtnY();

        float GetDPad();

        bool GetDPadUp();
        bool GetDPadRight();
        bool GetDPadDown();
        bool GetDPadLeft();


        /* State-checking variants. */
        bool OnLeftPush();
        bool OnRightPush();
    //    bool OnLeftBumper();
    //    bool OnRightBumper();

    //    bool OnStart();
    //    bool OnBack();
        bool OnA();
        bool OnB();
        bool OnX();
        bool OnY();

        bool OnDPadUp();
        bool OnDPadDown();
        bool OnDPadLeft();
        bool OnDPadRight();

        void ClearState();

    private:
        static const unsigned int kNumButtons = 12;
        static const unsigned int kLeftJoystickButton = 9;
        static const unsigned int kRightJoystickButton = 10;
        static const unsigned int kNumAxes = 6;

        float m_deadBand;

        typedef enum {
            kLeftXAxis  = 0,
            kLeftYAxis  = 1,
			kLeftTrigger = 2,
			kRightTrigger = 3,
            kRightXAxis = 4,
            kRightYAxis = 5
        } GamepadAxisType;


        enum state_idx {
            STATE_LEFT_PUSH,
            STATE_RIGHT_PUSH,
            STATE_LEFT_BUMPER,
            STATE_RIGHT_BUMPER,
            STATE_BTN_START,
            STATE_BTN_BACK,
            STATE_BTN_A,
            STATE_BTN_B,
            STATE_BTN_X,
            STATE_BTN_Y,
            STATE_DPAD_UP,
            STATE_DPAD_DOWN,
            STATE_DPAD_LEFT,
            STATE_DPAD_RIGHT,
            STATE_MAX
        };

        bool m_state[STATE_MAX];
        bool on_state(bool raw, enum state_idx idx);

        float leftXOffset, leftYOffset, rightXOffset, rightYOffset;

        frc::Joystick * LJoystick;
        frc::Joystick * RJoystick;
};

#endif /* H_TM1600_CONTROLLER*/

/* vim: set ts=4 sw=4 sts=4 expandtab: */
