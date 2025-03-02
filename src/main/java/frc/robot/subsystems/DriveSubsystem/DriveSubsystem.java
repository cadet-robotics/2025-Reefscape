// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveSubsystem;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.RobotConfig;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;

import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.lib.Limelight.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

public class DriveSubsystem extends CSubsystem {

  // Create MAXSwerveModules
  private final MAXSwerveModule frontLeftMaxSwerveModule = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule frontRightMaxSwerveModule = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule rearLeftMaxSwerveModule = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule rearRightMaxSwerveModule = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // A local copy of the m_driverController from `RobotContainer.java`
  public static PS4Controller driverPS4Controller;

  // The gyro sensor
  private final AHRS gyroAHRS = new AHRS(NavXComType.kMXP_SPI);

  // The multiplier used for slow mode 
  private static double slowMultiplier = 1.0;

  // The functions for if the drive subsystem should be in slow mode
  private static BooleanSupplier elevatorSlowCheck = ()->{return true;}; // Setting the default values in case it never gets inintalized
  private static BooleanSupplier extenderSlowCheck = ()->{return true;};

  // Odometry class for tracking robot pose
  SwerveDriveOdometry swerveDriveOdemtry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(gyroAHRS.getAngle()),
      new SwerveModulePosition[] {
          frontLeftMaxSwerveModule.getPosition(),
          frontRightMaxSwerveModule.getPosition(),
          rearLeftMaxSwerveModule.getPosition(),
          rearRightMaxSwerveModule.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Setting up PathPlanner

    try {
      RobotConfig.fromGUISettings();
    } catch (IOException e) {
      e.printStackTrace();
    } catch (ParseException e) {
      e.printStackTrace();
    }

    // Usage reporting for MAXSwerve template
    gyroAHRS.reset();
    gyroAHRS.setAngleAdjustment(180.0);
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  public void buttonBindings(PS4Controller ps4DriverController, PS4Controller ps4CodriverController) {
    driverPS4Controller = ps4DriverController;

    // Reset gyro
    new JoystickButton(driverPS4Controller, Button.kOptions.value)
        .whileTrue(
            GyroReset());

    // Slow Down Button
    // new JoystickButton(driverPS4Controller, Button.kR2.value)
    //     .whileTrue(
    //         slowDown());
  }

  /**
   * Sets the functions that the drive subsystem will use to check if slow mode should be enabled
   */
  public static void setSlowFunctions( BooleanSupplier elevatorFunction, BooleanSupplier extenderFunction) {
    elevatorSlowCheck = elevatorFunction;
    extenderSlowCheck = extenderFunction;
  }
  
  @Override
  public void periodic() {
    
    if ( driverPS4Controller.getL1ButtonPressed() || extenderSlowCheck.getAsBoolean() || elevatorSlowCheck.getAsBoolean()) {

      slowMultiplier = DriveConstants.kSlowerMultiplier;

    } else if ( driverPS4Controller.getR1ButtonPressed() ) {

      slowMultiplier = DriveConstants.kSlowMultiplier;
      SmartDashboard.putBoolean("slowmode", true);

    } 
    else { 

      slowMultiplier = 1.0;
      SmartDashboard.putBoolean( "slowmode", false);

    }
    // Update the odometry in the periodic block
    SmartDashboard.putNumber("Gyro", gyroAHRS.getAngle());
    boolean useLimeLight = false;

    swerveDriveOdemtry.update(
        Rotation2d.fromDegrees(gyroAHRS.getAngle()),
        new SwerveModulePosition[] {
            frontLeftMaxSwerveModule.getPosition(),
            frontRightMaxSwerveModule.getPosition(),
            rearLeftMaxSwerveModule.getPosition(),
            rearRightMaxSwerveModule.getPosition()
        });

    SmartDashboard.putBoolean("CirclePressed", driverPS4Controller.getCircleButtonPressed());

    //get input from driver PS4 Controller
    double xSpeed = MathUtil.applyDeadband(driverPS4Controller.getLeftY(), OIConstants.kDriveDeadband);
    double ySpeed = MathUtil.applyDeadband(driverPS4Controller.getLeftX(), OIConstants.kDriveDeadband);
    double rotationalSpeed = -MathUtil.applyDeadband(driverPS4Controller.getRightX(), OIConstants.kDriveDeadband);
    boolean fieldRelative = true;

    // Switches to non field-relative driving if the driver presses the Circle
    // button,
    // and switches to using the limelight
    if (LimelightHelpers.getTV("") && driverPS4Controller.getCircleButton()) {
      useLimeLight = true;
      final var rot_limelight = limelight_aim_proportional();
      rotationalSpeed = rot_limelight;

      final var forward_limelight = limelight_range_proportional();
      xSpeed = forward_limelight/10.0;

      // while using Limelight, turn off field-relative driving.
      fieldRelative = false;
    }

    this.drive(xSpeed, ySpeed, rotationalSpeed, fieldRelative, useLimeLight );
  }

  /**
   * Method to drive the robot using joystick info or values passed from the limelight.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotation      Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param limelightdrive Wither the drive function is using the limelight
   */
  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative, boolean limeLightDrive) {

    // The speed multiplier is used to slow down the robot when it is controlled by the limelight
    double speedMultiplier;
    if ( limeLightDrive ) {
      speedMultiplier = DriveConstants.kLimelightSpeedMultiplier;
    } else {
      speedMultiplier = 1.0;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond/speedMultiplier/slowMultiplier;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond/speedMultiplier/slowMultiplier;
    double rotationDelivered = rotation * DriveConstants.kMaxAngularSpeed/speedMultiplier/slowMultiplier;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered, Rotation2d.fromDegrees(-1.0 * gyroAHRS.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    frontLeftMaxSwerveModule.setDesiredState(swerveModuleStates[0]);
    frontRightMaxSwerveModule.setDesiredState(swerveModuleStates[1]);
    rearLeftMaxSwerveModule.setDesiredState(swerveModuleStates[2]);
    rearRightMaxSwerveModule.setDesiredState(swerveModuleStates[3]);

  }

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is
  // proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional
  // to the
  // "tx" value from the Limelight.
  double limelight_aim_proportional() {
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our
    // proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
    // rightmost edge of
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond;

    // invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are
  // different.
  // if your limelight and target are mounted at the same or similar heights,use
  // "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional() {
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTA("limelight") * kP;
    targetingForwardSpeed *= Constants.AutoConstants.kMaxSpeedMetersPerSecond;
    // targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeftMaxSwerveModule.setDesiredState(swerveModuleStates[0]);
    frontRightMaxSwerveModule.setDesiredState(swerveModuleStates[1]);
    rearLeftMaxSwerveModule.setDesiredState(swerveModuleStates[2]);
    rearRightMaxSwerveModule.setDesiredState(swerveModuleStates[3]);
  }
public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
        frontLeftMaxSwerveModule.getState(),
        frontRightMaxSwerveModule.getState(),
        rearLeftMaxSwerveModule.getState(),
        rearRightMaxSwerveModule.getState()
    };
    return DriveConstants.kDriveKinematics.toChassisSpeeds(swerveModuleStates);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return swerveDriveOdemtry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose2d The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose2d) {
    swerveDriveOdemtry.resetPosition(
        Rotation2d.fromDegrees(gyroAHRS.getAngle()),
        new SwerveModulePosition[] {
            frontLeftMaxSwerveModule.getPosition(),
            frontRightMaxSwerveModule.getPosition(),
            rearLeftMaxSwerveModule.getPosition(),
            rearRightMaxSwerveModule.getPosition()
        },
        pose2d);
  }


  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds( desiredStates, DriveConstants.kMaxSpeedMetersPerSecond );
    frontLeftMaxSwerveModule.setDesiredState(desiredStates[0]);
    frontRightMaxSwerveModule.setDesiredState(desiredStates[1]);
    rearLeftMaxSwerveModule.setDesiredState(desiredStates[2]);
    rearRightMaxSwerveModule.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeftMaxSwerveModule.resetEncoders();
    rearLeftMaxSwerveModule.resetEncoders();
    frontRightMaxSwerveModule.resetEncoders();
    rearRightMaxSwerveModule.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyroAHRS.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(gyroAHRS.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyroAHRS.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  // /**
  //  * slowDown 
  //  * Sets the slowMultiplier to the value configured in constants
  //  * DriveSubsystem
  //  */
  // public CCommand slowDown() {
  //   return cCommand_("DriveSubsystem.SlowDown")
  //       .onInitialize(() -> {
  //         slowMultiplier = DriveConstants.kSlowMultiplier;
  //       })
  //       .onEnd(() -> {
  //         slowMultiplier = 1.0;
  //       });
  // }

  /**
   * WheelX 
   * Sets the wheels into a X shape
   * DriveSubsystem
   */
  public CCommand WheelX() {
    return cCommand_("DriveSubsystem.WheelX")
        .onExecute(() -> {
          frontLeftMaxSwerveModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          frontRightMaxSwerveModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          rearRightMaxSwerveModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          rearLeftMaxSwerveModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        });
  }
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public CCommand GyroReset() {
    return cCommand_("DriveSubsystem.GyroReset")
        .onExecute(() -> {
          resetOdometry(getPose());
          zeroHeading();
          resetEncoders();
        });
  }
}
