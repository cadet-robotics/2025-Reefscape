// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(29.5);

    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(29.5);

    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(

        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;  // CAN ID
    public static final int kRearLeftDrivingCanId = 13;   // CAN ID
    public static final int kFrontRightDrivingCanId = 15; // CAN ID 
    public static final int kRearRightDrivingCanId = 17;  // CAN ID 
    public static final int kFrontLeftTurningCanId = 10;  // CAN ID 
    public static final int kRearLeftTurningCanId = 12;   // CAN ID 
    public static final int kFrontRightTurningCanId = 14; // CAN ID 
    public static final int kRearRightTurningCanId = 16;  // CAN ID 

    public static final boolean kGyroReversed = true;
    
    // Both of these values are changable and impact the speed of the robot in given circumstances
    public static final double kSlowMultiplier = 1.0;
    public static final double kLimelightSpeedMultiplier = 5.0;

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class HorzontalExtenderSubsystem {
    public static final int kSnowblowerMotor = 5; // CAN ID
    public static final double kExtendSpeed = 0.5; // Speed
    public static final int kFrontLimitSwitch = 6; // DIO Port 
    public static final int kBackLimitSwitch = 5;  // DIO Port
    // Can be Hold or Press
    // Hold : button must be heald for movement
    // Press: will fully extend or retract with one press
    public static final String extederMode = "Press";
  }

  public static final class AlgaeSubsystem {

    public static final int kLeftAlgaeMotor = 3; // CAN ID
    public static final int kRightAlgaeMotor = 4; // CAN ID
    public static final double kSpeed = 0.10; // Speed
    
  }

  public static final class ElevatorSubsystem {

    public static final double kElevatorSlowThreashold = 1000.0;
    public static final int kElevatorMotor = 1; // CAN ID

    public static final int kElevatorBrake = 0; // RLS ID

    public static final int kTopLimitSwitch = 1;    // DIO ID
    public static final int kBottomLimitSwitch = 0; // DIO ID

    public static final int kElevatorEncoderA = 2; // DIO ID
    public static final int kElevatorEncoderB = 3; // DIO ID 

    public static String[] LevelNames = { "AlgaeFloor", "ScoreAlgae", "Trough", "PreClimb", "Intake", "ReefLevel2", "ReefLevel3", "AlgaeTop", "ReefTop" };
    public static double[] LevelHeights = { 0.0, 10000.0, 20000.0, 30000.0, 40000.0, 50000.0, 60000.0, 70000.0, 80000.0, 90.0000 };

    // TODO: The following values need to be tuned on Mikey
    public static final double kServoEnagedPos = 1.0; // Postion
    public static final double kServoDisenagedPos = 1.0; // Position
  
    public static final double kBreakEngageTime = 119.5;
  }

  public static final class BucketSubsystem {

    public static final int kSnowblowerMotor = 2; // CAN ID
    public static final double SnowblowerSpeed = 0.10; // Speed
    public static final double[] bucketPositionArray = {0.0,0.25,0.75}; // Positions

  }
} 