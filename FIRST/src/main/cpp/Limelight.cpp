#include <string>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include "Robot.h"

/* Reference:

Config Panel:


Camera Stream:


*/

/**
 * Retrieve a value from the limelight table.
 * 
 * @param variable The variable to retrieve.
 * @param default_value An optional parameter for specifying the defualt value.
 * @return The value of the variable.
 */
double Robot::limelight_get(std::string variable, double default_value = 0.0) {
    return nt::NetworkTableInstance::GetDefault().GetTable("limelight-shooter")->GetNumber(variable, default_value);
}

/**
 * Set a value in the limelight table.
 * 
 * @param variable The variable to set.
 * @param value The value to set the variable to.
 */
void Robot::limelight_set(std::string variable, double value) {
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-shooter")->PutNumber(variable, value);
}
