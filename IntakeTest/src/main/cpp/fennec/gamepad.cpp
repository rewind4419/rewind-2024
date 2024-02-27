// #include <frc/smartdashboard/SmartDashboard.h>
// #include <stdio.h>
// #include <rev/CANSparkMax.h>
// #include <frc/Joystick.h>



// #include "intake.h"
// #include "gamepad.h"

// ///#define NEW_SPARK_MAX(can) new rev::CANSparkMax(can, rev::CANSparkMaxLowLevel::MotorType::kBrushless)

// static frc::Joystick driver_gamepad (0);
// static frc::Joystick mate_gamepad (1);
// static Intake intake;

// rev::CANSparkMax intake_motor {
//    40,
//    rev::CANSparkMax::MotorType::kBrushless
//  };

// // void intake(Robot* robot, bool out)
// // {
// //     if (out) {
// //         robot->intake.intake_motor.Set(0.4f);
// //     }
// //     else {
// //         robot->intake.intake_motor.Set(-0.4f);
// //     }
// // }

// //robot->intake.axis_motor.Set(robot->intake.actual_throttle);

// //robot->intake.intake_motor.Set(robot->intake.puller_throttle);

// //  robot->intake.actual_throttle = mix(
//         //robot->intake.actual_throttle,
//         //target_throttle,
//         //0.2f
//     //);

// robot->intake.actual_throttle = CLAMP(robot->intake.actual_throttle, -CFG_INTAKE_THROTTLE, CFG_INTAKE_THROTTLE);

// static GamepadButton getGamepadButton(int x, frc::Joystick* joystick)
// {
//     GamepadButton button;
//     button.down = joystick->GetRawButtonPressed(x);
//     button.up   = joystick->GetRawButtonReleased(x);
//     button.held = joystick->GetRawButton(x);
//     return button;
// }

// static void doGamepadInput(GamepadInput* ctrl, frc::Joystick* joystick)
// {
//     #if 0 // noraml controller
//     ctrl->joystick_left.x = joystick->GetRawAxis(0);
//     ctrl->joystick_left.y = -joystick->GetRawAxis(1);

//     ctrl->trigger_left = joystick->GetRawAxis(2);
//     ctrl->trigger_right = joystick->GetRawAxis(3);

//     ctrl->joystick_right.x = joystick->GetRawAxis(4);
//     ctrl->joystick_right.y = -joystick->GetRawAxis(5);

//     ctrl->dpad = joystick->GetPOV(0);

//     ctrl->bumper_left  = getGamepadButton(5, joystick);
//     ctrl->bumper_right = getGamepadButton(6, joystick);

//     ctrl->a = getGamepadButton(1, joystick);
//     ctrl->b = getGamepadButton(2, joystick);
//     ctrl->x = getGamepadButton(3, joystick);
//     ctrl->y = getGamepadButton(4, joystick);
//     #else // dual shock

//     ctrl->joystick_left.x = joystick->GetRawAxis(0);
//     ctrl->joystick_left.y = -joystick->GetRawAxis(1);

//     ctrl->trigger_left = joystick->GetRawAxis(3);
//     ctrl->trigger_right = joystick->GetRawAxis(4);

//     ctrl->joystick_right.x = joystick->GetRawAxis(2);
//     ctrl->joystick_right.y = -joystick->GetRawAxis(5);

//     ctrl->dpad = joystick->GetPOV(0);

//     ctrl->bumper_left  = getGamepadButton(5, joystick);
//     ctrl->bumper_right = getGamepadButton(6, joystick);

//     ctrl->a = getGamepadButton(2, joystick);
//     ctrl->b = getGamepadButton(3, joystick);
//     ctrl->x = getGamepadButton(1, joystick);
//     ctrl->y = getGamepadButton(4, joystick);

//     ctrl->big_button = getGamepadButton(14, joystick);

//     #endif
// }


// void updateGamepad(Input* input)
// {
//     doGamepadInput(&input->driver, &driver_gamepad);
//     doGamepadInput(&input->mate, &mate_gamepad);
//     //Start motor here
//     //
   
//     if (input->driver.x) {
//          robot->intake.intake_motor.Set(0.4f);
//      }
//      else {
//          robot->intake.intake_motor.Set(-0.4f);
//     }
// }

// //void updateGamepad(Input* input, frc::Intake* intake)
// //{
//   //  doGamepadInput(&input->driver, &driver_gamepad);
//     //doGamepadInput(&input->mate, &mate_gamepad);
//     //Start motor here
//     //intake.
   
//     //if (input->driver.x) {
//       //   robot->intake.intake_motor.Set(0.4f);
//      //}
//      //else {
//        //  robot->intake.intake_motor.Set(-0.4f);
//     //}
// //}

// //Step 1. No new code, just compile the Intake test folder and make sure it compiles fine
// //Step 2. Add the intake.h file at the top, compile and make sure it compiles fine
// //step 3. add line # 14 and line #16 and compile
// //step 4. Now change the updateGamepad and compile
// //step 5. once the step 4 is done, do test it with the hardware
// //Step 6. If the step 4 is not done, then go the 2023 folder in C:\Users\frc4419\rewind-mono\robot2023\atlas-2023\src\main\cpp
// // and check if there are any file that uses intake.h and intake.cpp and see how they are using the intake motor.
// //come back to this file and make the change and start from step 1
