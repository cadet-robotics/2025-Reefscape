// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveSubsystem;

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
// 
import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

public class DriveSubsystem extends CSubsystem {

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  public double maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Setting up PathPlanner
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Usage reporting for MAXSwerve template
    m_gyro.reset();
    m_gyro.setAngleAdjustment(180.0);
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
    return DriveConstants.kDriveKinematics.toChassisSpeeds(states);
  }

  // // simple proportional turning control with Limelight.
  // // "proportional control" is a control algorithm in which the output is
  // proportional to the error.
  // // in this case, we are going to return an angular velocity that is
  // proportional to the
  // // "tx" value from the Limelight.
  // double limelight_aim_proportional() {
  // // kP (constant of proportionality)
  // // this is a hand-tuned number that determines the aggressiveness of our
  // proportional control loop
  // // if it is too high, the robot will oscillate.
  // // if it is too low, the robot will never reach its target
  // // if the robot never turns in the correct direction, kP should be inverted.
  // double kP = .035;

  // // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
  // rightmost edge of
  // // your limelight 3 feed, tx should return roughly 31 degrees.
  // double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

  // // convert to radians per second for our drive method
  // targetingAngularVelocity *=
  // Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond;

  // //invert since tx is positive when the target is to the right of the
  // crosshair
  // targetingAngularVelocity *= -1.0;

  // return targetingAngularVelocity;
  // }

  // // simple proportional ranging control with Limelight's "ty" value
  // // this works best if your Limelight's mount height and target mount height
  // are different.
  // // if your limelight and target are mounted at the same or similar heights,
  // use "ta" (area) for target ranging rather than "ty"
  // double limelight_range_proportional()
  // {
  // double kP = .1;
  // double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
  // targetingForwardSpeed *= Constants.AutoConstants.kMaxSpeedMetersPerSecond;
  // targetingForwardSpeed *= -1.0;
  // return targetingForwardSpeed;
  // }

  public void buttonBindings(PS4Controller m_driverController, PS4Controller m_codriverController) {
    this.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {

              double tx = MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
              double ty = MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);
              double rot = MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband);
              boolean fieldRelative = true;

              // Switches to non field-relative driving if the driver presses the L1 button,
              // and switches to using the limelight
              if (LimelightHelpers.getTV("") == true && m_driverController.getL1ButtonPressed()) {

                LimelightResults results = LimelightHelpers.getLatestResults("");
                if (results.valid) {
                  // AprilTags/Fiducials
                  if (results.targets_Fiducials.length > 0) {
                    LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
                    double id = tag.fiducialID; // Tag ID
                    String family = tag.fiducialFamily; // Tag family (e.g., "16h5")

                    // 3D Pose Data
                    Pose3d robotPoseField = tag.getRobotPose_FieldSpace(); // Robot's pose in field space
                    Pose3d cameraPoseTag = tag.getCameraPose_TargetSpace(); // Camera's pose relative to tag
                    Pose3d robotPoseTag = tag.getRobotPose_TargetSpace(); // Robot's pose relative to tag
                    Pose3d tagPoseCamera = tag.getTargetPose_CameraSpace(); // Tag's pose relative to camera
                    Pose3d tagPoseRobot = tag.getTargetPose_RobotSpace(); // Tag's pose relative to robot

                    // 2D targeting data
                    tx = tag.tx; // Horizontal offset from crosshair
                    ty = tag.ty; // Vertical offset from crosshair
                    double ta = tag.ta; // Target area (0-100% of image)
                    // x = tx;
                    // y = ty;
                  }
                }
              }

              this.drive(
                  tx,
                  ty,
                  rot,
                  fieldRelative);
            },
            this));

    // Reset gyro
    new JoystickButton(m_driverController, Button.kOptions.value)
        .whileTrue(
            GyroReset());

    // Slow Down Button
    new JoystickButton(m_driverController, Button.kR2.value)
        .whileTrue(
            SlowDown());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * maxSpeed;
    double ySpeedDelivered = ySpeed * maxSpeed;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-1.0 * m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
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
          m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
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
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  // Set speed to 1/4 when command is active
  public CCommand SlowDown() {
    return cCommand_("DriveSubsystem.SlowDown")
        .onInitialize(() -> {
          maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond / 4;
        })
        .onEnd(() -> {
          maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;
        });
  }
}
