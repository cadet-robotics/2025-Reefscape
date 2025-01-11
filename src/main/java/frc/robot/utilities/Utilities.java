package frc.robot.utilities;

import frc.robot.Constants.OperatorConstants;

public class Utilities
{

    /**
     * Ignores values inside of the deadzone
     * 
     * @param value Value to evaluate
     * @return passed in value or zero if inside deadzone
     */
    public static double applyDeadzone(double value)
    {
        if (value > OperatorConstants.DEADZONE || value < -OperatorConstants.DEADZONE)
        {
            return value;
        }
        else
        {
            return 0;
        }
    }
}
