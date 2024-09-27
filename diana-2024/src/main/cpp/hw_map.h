#include "rev/CANSparkMax.h"
#include "frc/motorcontrol/Spark.h"
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

#define HW_LEFT_MOTOR_COUNT  2
#define HW_RIGHT_MOTOR_COUNT 2

struct RobotHardware
{
	const ControlMode controlMode = ControlMode::Velocity;
	TalonSRX* left_motors[HW_LEFT_MOTOR_COUNT];
	TalonSRX* right_motors[HW_RIGHT_MOTOR_COUNT];
};

inline void hwInit(RobotHardware* hw)
{
	hw->left_motors[0]  = new TalonSRX(23);
	hw->left_motors[1]  = new TalonSRX(24);

	hw->right_motors[0] = new TalonSRX(22);
	hw->right_motors[1] = new TalonSRX(25);
}
