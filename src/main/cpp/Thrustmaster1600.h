#ifndef H_TM1600_CONTROLLER
#define H_TM1600_CONTROLLER

#include <frc/Joystick.h>

/**
 * Handles input from a Logitech Gamepad connected to the Driver Station.
 * This class is a convenience class for the Logitech Dual Action Gamepad. It
 * wraps the Joystick class to provide methods to access the inputs on the
 * gamepad.
 */
class ThrustmasterController //: public frc::Joystick 
{
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

        ThrustmasterController(unsigned port);
        virtual ~ThrustmasterController();

        bool Calibrate(float max_offset = 0.3);

        int GetRawPOV(uint32_t pov);
        float GetStickX(void);
        float GetStickY(void);
        float GetStickZ(void);
        float GetThrottle(void);
     //   float GetRightX(void);
     //   float GetRightY(void);

        float GetDeadBand() { return m_deadBand; };
        void SetDeadBand(float db) { m_deadBand = db; };

        bool GetNumberedButton(unsigned int buttonNumber);
        bool GetBtn(int btn);

 /*       
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
*/
        float GetDPad();

        bool GetDPadUp();
        bool GetDPadRight();
        bool GetDPadDown();
        bool GetDPadLeft();


        /* State-checking variants. */
         /*
        bool OnLeftPush();
        bool OnRightPush();
        bool OnLeftBumper();
        bool OnRightBumper();

        bool OnStart();
        bool OnBack();
        bool OnA();
        bool OnB();
        bool OnX();
        bool OnY();
*/
        bool OnDPadUp();
        bool OnDPadDown();
        bool OnDPadLeft();
        bool OnDPadRight();

        void ClearState();

    private:
        frc::Joystick * stick;


        static const unsigned int kNumButtons = 12;
        static const unsigned int kLeftJoystickButton = 9;
        static const unsigned int kRightJoystickButton = 10;
        static const unsigned int kNumAxes = 6;

        float m_deadBand;

        typedef enum {
            kXAxis  = 0,
            kYAxis  = 1,
			kRotateAxis = 2,
			kThrottle = 3,
      //      kRightXAxis = 4,
       //     kRightYAxis = 5
        } GamepadAxisType;


        enum state_idx {
            STATE_BTN_1,
            STATE_BTN_2,
            STATE_BTN_3,
            STATE_BTN_4,
            STATE_BTN_5,
            STATE_BTN_6,
            STATE_BTN_7,
            STATE_BTN_8,
            STATE_BTN_9,
            STATE_BTN_10,
            STATE_BTN_11,
            STATE_BTN_12,
            STATE_BTN_13,
            STATE_BTN_14,
            STATE_BTN_15,
            STATE_BTN_16,
            STATE_DPAD_UP,
            STATE_DPAD_DOWN,
            STATE_DPAD_LEFT,
            STATE_DPAD_RIGHT,
            STATE_MAX
        };

        bool m_state[STATE_MAX];
        bool on_state(bool raw, enum state_idx idx);



        float leftXOffset, leftYOffset, rightXOffset, rightYOffset;


    public:
        bool OnBtn(int btn);

};

#endif /* H_TM1600_CONTROLLER*/

/* vim: set ts=4 sw=4 sts=4 expandtab: */
