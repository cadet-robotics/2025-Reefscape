// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.utilities.Utilities;

public class SwerveDriveCommand extends Command
{
    DriveSubsystem m_drive;
    PIDController pidController = new PIDController(.011,0,0.0001);
    /** Creates a new DriveWithGampad. */
    public SwerveDriveCommand(DriveSubsystem drive)
    {
        // Use addRequirements() here to declare subsystem dependencies.
        m_drive = drive;
        addRequirements(m_drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
    }

    // Called every time the scheduler runs while the command is scheduled.
    // Sets up drive motors to ingore deadzone inputs.
    // Allows drive to run.
    @Override
    public void execute()
    {
        double driveScaler =  RobotContainer.m_driverController.L1().getAsBoolean()?OperatorConstants.SLOW_FACTOR:1;
        
        m_drive.drive(driveScaler*Utilities.applyDeadzone(RobotContainer.m_driverController.getLeftY()), // x speed
                driveScaler*Utilities.applyDeadzone(RobotContainer.m_driverController.getLeftX()), // y speed
                RobotContainer.m_driverController.R1().getAsBoolean()?
                    pidController.calculate(m_drive.getHeading(), 0.0):
                    -driveScaler*Utilities.applyDeadzone(RobotContainer.m_driverController.getRightX()), // rot speed
                true, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        m_drive.drive(0, 0, 0, true, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
