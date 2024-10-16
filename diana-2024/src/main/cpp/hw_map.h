#include "rev/CANSparkMax.h"
#include "frc/motorcontrol/Spark.h"
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include <rev/CANSparkMax.h>

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

#define HW_LEFT_MOTOR_COUNT  2
#define HW_RIGHT_MOTOR_COUNT 2

struct RobotHardware
{
	const ControlMode controlMode = ControlMode::Velocity;
	rev::CANSparkMax* left_motors[HW_LEFT_MOTOR_COUNT];
	rev::CANSparkMax* right_motors[HW_RIGHT_MOTOR_COUNT];
};

inline void hwInit(RobotHardware* hw)
{
	hw->left_motors[0]  = new rev::CANSparkMax(23, rev::CANSparkLowLevel::MotorType::kBrushed);
	hw->left_motors[1]  = new rev::CANSparkMax(24, rev::CANSparkLowLevel::MotorType::kBrushed);

	hw->right_motors[0] = new rev::CANSparkMax(22, rev::CANSparkLowLevel::MotorType::kBrushed);
	hw->right_motors[1] = new rev::CANSparkMax(25, rev::CANSparkLowLevel::MotorType::kBrushed);
}
