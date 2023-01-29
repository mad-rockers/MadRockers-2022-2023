#include <string>
#include <vector>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "Robot.h"

using namespace std;

/* Reference:

Config Panel:


Camera Stream:


*/

/**
 * Retrieve a variable from the limelight table.
 * 
 * @param variable The variable to retrieve.
 * @return The value of the variable.
 */
double Robot::GetLimelightValue(string variable) {
    return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber(variable, 0);
}

/**
 * Retrieve a variable array from the limelight table.
 * 
 * @param variable The variable to retrieve.
 * @return An vector with the values of the variable.
 */
vector<double> Robot::GetLimelightArray(string variable) {
    return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("<variablename>", vector<double>(6));
}

/**
 * Set a variable in the limelight table.
 * 
 * @param variable The variable to set.
 * @param value The value to set the variable to.
 */
void Robot::SetLimelightValue(string variable, double value) {
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber(variable, value);
}
