// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS4Controller.Button;
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
    public static final double kMaxSpeedMetersPerSecond = 6;
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
    
    // Both of these values are changable and impact the speed or time robot moves for in given circumstances
    public static final double kSlowMultiplier = 2.0;
    public static final double kSlowerMultiplier = 4.0;
    public static final double kLimelightSpeedMultiplier = 5.0;
    public static final double HoldTime = 0.365;
    // public static final double kSlowMultiplier = 2.0;
    // public static final double kSlowerMultiplier = 5.0;

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
    public static final int kCoDriverControllerPort = 1;
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
    public static final double kExtendSpeed = 0.3; // Speed
    public static final int kFrontLimitSwitch = 6; // DIO Port 
    public static final int kBackLimitSwitch = 5;  // DIO Port
    // Can be Hold or Press
    // Hold : button must be heald for movement
    // Press: will fully extend or retract with one press
    public static final String extederMode = "Hold";
  }

  public static final class AlgaeSubsystem {

    public static final double HoldTime = 1; // The time that the robot should hold down the algae motors for ( Seconds )
    public static final int kLeftAlgaeMotor = 3; // CAN ID
    public static final int kRightAlgaeMotor = 4; // CAN ID
    public static final double kAlgaeIntakeSpeed = 0.6; // Speed
    
  }

  public static final class ElevatorSubsystem {

    public static final double PidMax = 0.2; // The maximum speed pid is allowed to go

    public static final double kElevaotrManualSpeed = 0.4;
    public static final double kElevatorSlowThreashold = 40.0;

    public static final int kElevatorMotor = 1; // CAN ID

    public static final int kElevatorBrake = 0; // RLS ID

    public static final int kTopLimitSwitch = 0;    // DIO ID
    public static final int kBottomLimitSwitch = 1; // DIO ID

    public static final int kElevatorEncoderA = 2; // DIO ID
    public static final int kElevatorEncoderB = 3; // DIO ID 

    public static String[] LevelNames = { "Trough", "ScoreAlgae", "AlgaeMid", "CoralStation", "PreClimb", "ReefLevel3", "AlgaeTop", "ReefTop" };
    public static double[] LevelHeights = {  0.0, 7.0, 17.7, 18.223, 38.374, 44.052, 47.7, 72.646, 90.0 };

    // TODO: The following values need to be tuned on Mikey
    public static final double kServoEnagedPos = 1.0; // Postion
    public static final double kServoDisenagedPos = 0.0; // Position
  
    public static final double kBreakEngageTime = 119.5;

    public static final class CrappyPid {
      // TODO: tune all values


      // The slower speed to be used by the elevator when within the slow distance threshold
      public static double kElevatorSlowSpeed = 0.05;

      // Defualt speed for the elevator
      public static double kElevatorNormSpeed = 0.3;
      
      // The speed the elevator will hover at ( must be enabled in elevatorSubsystem )
      public static double kElevatorHoverSpeed = 0.01;

      // The distance when the elevator should stop, or use the however speed instead of moving slowly
      public static double kElevatorStopThreshold = 0.03;

      // The distance where crappy pid should use a slow speed instead of the normal one
      public static double kElevatorSlowDistanceThreashold = 2.0;

    }
  }

  public static final class BucketSubsystem {

    public static final int kCurrentLimit = 10;
    // The minimum value where the bucket no longer blocks movement of the encoder
    // TODO: Tune
    public static final double kBlockingExenderPosition = 0.2;
    
    public static final double PidMax = 1; // The maximum speed pid is allowed to go

    public static final int kSnowblowerMotor = 2; // CAN ID
    public static final double SnowblowerForwardSpeed = 0.6; // Speed
    public static final double SnowblowerBackwardSpeed = 0.4; // Speed
    public static final double[] bucketPositionArray = { 0.99, 0.715, 0.884, 0.64 }; // Positions
// Top dump 64
  }

  public static final class DriverControls
  {
    //CURRENTLY UNUSED BUTTONS ON DRIVER CONTROLLER: Triangle, Cross, L3, R3, All Dpad Buttons, Touchpad, Playstation

    //Press L1 on DRIVER CONTROLLER to go slower than slowdown
    public static final int slowerButton = Button.kL1.value;

    //Press R1 on DRIVER CONTROLLER to slow down
    public static final int slowButton = Button.kR1.value;


    //Press Share on DRIVER CONTROLLER to adjust your position when tracking apriltag with limelight
    //temporarily using Share button
    public static final int useLimelightRight = Button.kCircle.value;

    public static final int useLimelightLeft = Button.kSquare.value;

    //Press Options on DRIVER CONTROLLER to reset the gyro
    public static final int resetGyroButton = Button.kOptions.value;

    //Press Square on DRIVEr CONTROLLER to manually move the bucket backward
    public static final int bucketManualBackwardButton = Button.kL2.value;

    //Press Triangle on DRIVER CONTROLLER to manually move the bucket forward
    public static final int bucketManualForwardButton = Button.kR2.value;

    //Press L1 on CODRIVER CONTROLLER to put the bucket into the load position
    public static final int bucketLoadPositionButton = Button.kL1.value;

    //Press R1 on CODRIVER CONTROLLER to put the bucket into the dump position
    public static final int bucketDumpPositionButton = Button.kR1.value;

    public static final int enableBreak = Button.kShare.value;

    //Press Share on CODRIVER CONTROLLER to put the bucket into the start position
    public static final int bucketStartPositionButton = Button.kTriangle.value;
    
    public static final int moveToTopReef = Button.kCross.value;
  }
  public static final class CoDriverControls {
   
    public static final int disbaleBreak = Button.kShare.value;

    public static final int horizontalRetractButton = Button.kSquare.value;

    //Press Circle on CODRIVER CONTROLLER to extend the "inny outty"/Horizontal Extender
    public static final int horizontalExtendButton = Button.kCross.value;

    //Press L2 on DRIVER CONTROLLER to make the intake spin outwards
    public static final int intakeOutButton = Button.kTriangle.value;

    //Press R2 on DRIVER CONTROLLER to make the intake spin inwards
    public static final int intakeInButton = Button.kCircle.value;
  
    public static final int elevaotrDownManual = Button.kL2.value;
    public static final int elevatorUpManual = Button.kR2.value;

    }
  } 
