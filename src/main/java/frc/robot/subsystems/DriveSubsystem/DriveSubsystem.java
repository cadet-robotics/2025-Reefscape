// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveSubsystem;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.RobotConfig;
// Gyro
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
// Swerve Drive
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// Button Matpping
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
// 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.lib.Limelight.LimelightHelpers;
import frc.robot.lib.Limelight.LimelightHelpers.LimelightResults;
import frc.robot.lib.Limelight.LimelightHelpers.LimelightTarget_Barcode;
import frc.robot.lib.Limelight.LimelightHelpers.LimelightTarget_Classifier;
import frc.robot.lib.Limelight.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.lib.Limelight.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.lib.Limelight.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.lib.Limelight.LimelightHelpers.RawFiducial;
// 
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

  public static PS4Controller driverPS4Controller;
  // The gyro sensor
  private final AHRS gyroAHRS = new AHRS(NavXComType.kMXP_SPI);

  public double maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;

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
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= Constants.AutoConstants.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    SmartDashboard.putNumber("Gyro", gyroAHRS.getAngle());

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
    double rotationalSpeed = MathUtil.applyDeadband(driverPS4Controller.getRightX(), OIConstants.kDriveDeadband);
    boolean fieldRelative = true;

    // Switches to non field-relative driving if the driver presses the Circle
    // button,
    // and switches to using the limelight
    if (LimelightHelpers.getTV("") && driverPS4Controller.getCircleButton()) {
      final var rot_limelight = limelight_aim_proportional();
      rotationalSpeed = rot_limelight;

      final var forward_limelight = limelight_range_proportional();
      xSpeed = forward_limelight;

      // while using Limelight, turn off field-relative driving.
      fieldRelative = false;
    }

    this.drive(xSpeed, ySpeed, rotationalSpeed, fieldRelative);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxSpeed);
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

  public void buttonBindings(PS4Controller ps4DriverController, PS4Controller ps4CodriverController) {
    driverPS4Controller = ps4DriverController;
    // this.setDefaultCommand(
    // // The left stick controls translation of the robot.
    // // Turning is controlled by the X axis of the right stick.
    // new RunCommand(
    // () -> {

    // double tx = MathUtil.applyDeadband(m_driverController.getLeftY(),
    // OIConstants.kDriveDeadband);
    // double ty = MathUtil.applyDeadband(m_driverController.getLeftX(),
    // OIConstants.kDriveDeadband);
    // double rot = MathUtil.applyDeadband(m_driverController.getRightX(),
    // OIConstants.kDriveDeadband);
    // boolean fieldRelative = true;

    // // Switches to non field-relative driving if the driver presses the L1
    // button,
    // // and switches to using the limelight
    // if (LimelightHelpers.getTV("") == true &&
    // m_driverController.getL1ButtonPressed()) {

    // RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
    // for (RawFiducial fiducial : fiducials) {
    // int id = fiducial.id; // Tag ID
    // double txnc = fiducial.txnc; // X offset (no crosshair)
    // double tync = fiducial.tync; // Y offset (no crosshair)
    // double ta = fiducial.ta; // Target area
    // double distToCamera = fiducial.distToCamera; // Distance to camera
    // double distToRobot = fiducial.distToRobot; // Distance to robot
    // double ambiguity = fiducial.ambiguity; // Tag pose ambiguity
    // tx = distToCamera;
    // ty = ta;
    // }
    // }

    // this.drive(
    // tx,
    // ty,
    // rot,
    // fieldRelative);
    // },
    // this));

    // Reset gyro
    new JoystickButton(driverPS4Controller, Button.kOptions.value)
        .whileTrue(
            GyroReset());

    // Slow Down Button
    new JoystickButton(driverPS4Controller, Button.kR2.value)
        .whileTrue(
            slowDown());
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
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotation           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * maxSpeed;
    double ySpeedDelivered = ySpeed * maxSpeed;
    double rotationDelivered = rotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered, Rotation2d.fromDegrees(-1.0 * gyroAHRS.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

    frontLeftMaxSwerveModule.setDesiredState(swerveModuleStates[0]);
    frontRightMaxSwerveModule.setDesiredState(swerveModuleStates[1]);
    rearLeftMaxSwerveModule.setDesiredState(swerveModuleStates[2]);
    rearRightMaxSwerveModule.setDesiredState(swerveModuleStates[3]);

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
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, maxSpeed);
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

  // Set speed to 1/4 when command is active
  public CCommand slowDown() {
    return cCommand_("DriveSubsystem.SlowDown")
        .onInitialize(() -> {
          maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond / 4;
        })
        .onEnd(() -> {
          maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;
        });
  }
}
