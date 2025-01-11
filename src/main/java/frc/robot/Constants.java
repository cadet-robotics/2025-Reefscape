// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{   
    public static class OperatorConstants
    {
        //Controller ports
        public static final int kDriverControllerPort = 0;
        public static final int kCoDriverControllerPort = 1;


        //Spark Max NUmbers
        public static final int LAUNCH_MOTOR_ID1 = 18;
        public static final int LAUNCH_MOTOR_ID2 = 19;
        public static final int LAUNCH_ELEVATION_MOTOR_ID = 20;
        public static final int INTAKE_MOTOR_ID = 21;
        public static final int CLIMBER_MOTOR_ID = 22;

        //dio PORTS
        //Climber limit switch indexes
        public static final int CLIMB_ASCEND_LS = 0;
        public static final int CLIMB_DESCEND_LS = 1;
        //Launch limit switch indexes 
        public static final int LAUNCH_ELEV_ASCEND_LS = 2;
        public static final int LAUNCH_ELEV_DESCEND_LS = 3;
        public static final int LAUNCH_FIRE_LIGHTS = 7;

        public static final double CLIMB_ASCEND_SPEED = 0.8; // Is Down
        public static final double CLIMB_DESCEND_SPEED = -0.9; // Is Up

        public static final double SLOW_FACTOR = .5;
        public static final double DEADZONE = 0.13;
        public static final double INTAKE_SPEED = .8; // Was at .5
        public static final int LAUNCH_1_SPEED = 5000;// for 60% use 3400, was at 4700 for 80%;
        public static final int LAUNCH_2_SPEED = 5000;// for 75% use 4300;
        public static final double DELAY_FOR_SHOOT_START = 1.5;
        public static final double DELAY_FOR_SHOOTING = 2;
        public static final int BLINKIN_LED_CONTROLLER_PORT = 9;
    }

    public static final double LAUNCHER_P = 0.0001;
    public static final double LAUNCHER_D = 0;
    public static final double LAUNCHER_FF = 0.000175;
    public static final class PWMConstants {
        public static final int LED_PWM = 0;
    }
    
}
