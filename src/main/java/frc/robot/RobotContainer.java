// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.PS4Controller.Button;

import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HorizontalExtenderSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final AlgaeSubsystem m_intake = new AlgaeSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final HorizontalExtenderSubsystem m_horizontalExtender = new HorizontalExtenderSubsystem();
  public final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  public final BucketSubsystem m_bucket = new BucketSubsystem();

  // The drivers' controllers
  private final PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);
  private final PS4Controller m_coDriverController = new PS4Controller(OIConstants.kCoDriverControllerPort);
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    passSlowModeBooleanSuppliers();
    extenderBucketBlocking();

    // Configure the button bindings
    String [] autos = new String[] {"test 1", "test 2"};
  
    autoChooser = AutoBuilder.buildAutoChooser("test 1");

    SmartDashboard.putStringArray("Auto List", autos );
    SmartDashboard.putData("Auto Selector", autoChooser);


    // Configure default commands
  }

  /**
   * A simple method to pass the isBucketBlocking between the bucket and extender subsystems
   */
  private void extenderBucketBlocking() { 
    //m_horizontalExtender.setIsBucketBlocking( m_bucket.isBucketBlocking );
  }
  private void passSlowModeBooleanSuppliers() {

    //DriveSubsystem.setSlowFunctions( 
      //m_elevatorSubsystem.elevatorSlowCheck, 
      //m_horizontalExtender.extenderSlowCheck 
    //);
  }

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
    

  
    new JoystickButton( m_coDriverController, Constants.CoDriverControls.horizontalExtendButton )
      .whileTrue( new SequentialCommandGroup( m_horizontalExtender.Extend(), m_intake.IntakeOut() ));
    
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
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
