// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveZero extends Command {
  /** Creates a new SwerveDriveZero. */
  DriveSubsystem m_drive;
  public SwerveDriveZero(DriveSubsystem drive) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
        m_drive = drive;
        addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetOdometry(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
  
  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
  
}
