// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.PS4Controller.Button;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

// import java.io.BufferedWriter;
import java.util.List;

// TODO: commented out and should be gradually uncommneted to test more features on mikey
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HorizontalExtenderSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  // TODO: commented out and should be gradually uncommneted to test more features on mikey
  private final AlgaeSubsystem m_intake = new AlgaeSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final HorizontalExtenderSubsystem m_horizontalExtender = new HorizontalExtenderSubsystem();
  public final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final BucketSubsystem m_bucket = new BucketSubsystem();

  // The drivers' controllers
  private final PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);
  private final PS4Controller m_coDriverController = new PS4Controller(OIConstants.kCoDriverControllerPort);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // TODO: commented out and should be gradually uncommneted to test more features on mikey
    // passSlowModeBooleanSuppliers();

    // Configure default commands
  }

  // TODO: commented out and should be gradually uncommneted to test more features on mikey
  // private void passSlowModeBooleanSuppliers() {
  //   DriveSubsystem.setSlowFunctions( 
  //     m_elevatorSubsystem.elevatorSlowCheck, 
  //     m_horizontalExtender.extenderSlowCheck 
  //   );
  // }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // TODO: commented out and should be gradually uncommneted to test more features on mikey
    // Intake buttons
    m_intake.buttonBindings(m_driverController, m_coDriverController);

    // Bucket System
    m_bucket.buttonBindings(m_driverController, m_coDriverController );

    // Swerve Drive buttons
    m_robotDrive.buttonBindings(m_driverController, m_coDriverController);

    // Horizontal Extender buttons
    m_horizontalExtender.buttonBindings(m_driverController, m_coDriverController);

    // Swerve Drive buttons
    m_robotDrive.buttonBindings(m_driverController, m_coDriverController);

    // Elevator Buttons
    m_elevatorSubsystem.buttonBindings(m_driverController, m_coDriverController);

}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
