#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include "frc/motorcontrol/Spark.h"

#define HW_LEFT_MOTOR_COUNT  2
#define HW_RIGHT_MOTOR_COUNT 2

struct RobotHardware
{
	WPI_TalonSRX* left_motors[HW_LEFT_MOTOR_COUNT];
	WPI_TalonSRX* right_motors[HW_RIGHT_MOTOR_COUNT];
};

inline void hwInit(RobotHardware* hw)
{
	hw->left_motors[0]  = new WPI_TalonSRX(23);
	hw->left_motors[1]  = new WPI_TalonSRX(24);

	hw->right_motors[0] = new WPI_TalonSRX(22);
	hw->right_motors[1] = new WPI_TalonSRX(25);
}
