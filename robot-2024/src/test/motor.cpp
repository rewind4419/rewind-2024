r->test_motor = new rev::CANSparkMax(1, rev::CANSparkMaxLowLevel::MotorType::kBrushless);

if (mode == ROBOT_TELEOP)
    {
        r->test_motor->Set(0.1);
    }