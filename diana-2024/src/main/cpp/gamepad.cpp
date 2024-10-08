#include <frc/smartdashboard/SmartDashboard.h>
#include <stdio.h>
// #include <rev/CANSparkMax.h>
#include <frc/Joystick.h>
#include "gamepad.h"

static frc::Joystick driver_gamepad (0);
static frc::Joystick mate_gamepad (1);


static GamepadButton getGamepadButton(int x, frc::Joystick* joystick)
{
    GamepadButton button;
    button.down = joystick->GetRawButtonPressed(x);
    button.up   = joystick->GetRawButtonReleased(x);
    button.held = joystick->GetRawButton(x);
    return button;
}

static void doGamepadInput(GamepadInput* ctrl, frc::Joystick* joystick)
{
    ctrl->joystick_left.x = joystick->GetRawAxis(0);
    ctrl->joystick_left.y = -joystick->GetRawAxis(1);

    ctrl->trigger_left = (joystick->GetRawAxis(3) + 1.0f) / 2.0f;
    ctrl->trigger_right = (joystick->GetRawAxis(4) + 1.0f) / 2.0f;

    ctrl->joystick_right.x = joystick->GetRawAxis(2);
    ctrl->joystick_right.y = -joystick->GetRawAxis(5);

    ctrl->dpad = joystick->GetPOV(0);

    ctrl->bumper_left  = getGamepadButton(5, joystick);
    ctrl->bumper_right = getGamepadButton(6, joystick);

    ctrl->a = getGamepadButton(1, joystick);
    ctrl->b = getGamepadButton(2, joystick);
    ctrl->x = getGamepadButton(3, joystick);
    ctrl->y = getGamepadButton(4, joystick);

    //printf("LT %f RT %f RightX %f RightY %f LeftX %f LeftY %f\n", ctrl->trigger_left, ctrl->trigger_right, ctrl->joystick_right.x, ctrl->joystick_right.y, ctrl->joystick_left.x, ctrl->joystick_left.y);
}


void updateGamepad(Input* input)
{
    doGamepadInput(&input->driver, &driver_gamepad);
    doGamepadInput(&input->mate, &mate_gamepad);
}