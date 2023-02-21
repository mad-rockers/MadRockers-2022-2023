namespace Ports {
    //USB
    const int driver = 0;

    //CAN
    const int left_front = 1;
    const int left_back = 2;
    const int right_front = 3;
    const int right_back = 4;
    const int arm = 5;
    const int extension = 6;

    //DIO
    const int arm_limit_low = 0;
    const int arm_limit_high = 1;
    const int extension_limit_back = 2;
    const int extension_limit_front = 3;

    //Pneumatics
    const int box = 0;
    const int pressure_switch = 1;
    const int grabber = 2;
    
    //I2C
    const frc::I2C::Port color_sensor = frc::I2C::Port::kMXP;
}