namespace Ports {
    //USB
    const int driver = 0;

    //CAN
    const int left_front = 0;
    const int left_back = 1;
    const int right_front = 2;
    const int right_back = 3;
    const int arm = 4;
    const int extension = 5;

    //DIO
    const int arm_limit_low = 0;
    const int arm_limit_high = 1;
    const int extension_limit_back = 2;
    const int extension_limit_front = 3;

    //I2C
    const frc::I2C::Port color_sensor = frc::I2C::Port::kMXP;
}