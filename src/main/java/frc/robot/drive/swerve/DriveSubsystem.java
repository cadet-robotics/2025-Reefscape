// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase
{
    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft =
            new MAXSwerveModule(SwerveConstants.DriveConstants.kFrontLeftDrivingCanId,
                    SwerveConstants.DriveConstants.kFrontLeftTurningCanId,
                    SwerveConstants.DriveConstants.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule m_frontRight =
            new MAXSwerveModule(SwerveConstants.DriveConstants.kFrontRightDrivingCanId,
                    SwerveConstants.DriveConstants.kFrontRightTurningCanId,
                    SwerveConstants.DriveConstants.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule m_rearLeft =
            new MAXSwerveModule(SwerveConstants.DriveConstants.kRearLeftDrivingCanId,
                    SwerveConstants.DriveConstants.kRearLeftTurningCanId,
                    SwerveConstants.DriveConstants.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule m_rearRight =
            new MAXSwerveModule(SwerveConstants.DriveConstants.kRearRightDrivingCanId,
                    SwerveConstants.DriveConstants.kRearRightTurningCanId,
                    SwerveConstants.DriveConstants.kBackRightChassisAngularOffset);

    // The gyro sensor
    AHRS ahrs = new AHRS(SPI.Port.kMXP);

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter =
            new SlewRateLimiter(SwerveConstants.DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter =
            new SlewRateLimiter(SwerveConstants.DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry =
            new SwerveDriveOdometry(SwerveConstants.DriveConstants.kDriveKinematics,
                    Rotation2d.fromDegrees(-ahrs.getAngle()).plus(new Rotation2d(Math.PI)), new SwerveModulePosition[]
                    {m_frontLeft.getPosition(), m_frontRight.getPosition(),
                            m_rearLeft.getPosition(), m_rearRight.getPosition()
                    });

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem()
    {
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getSpeeds, 
            this::driveSpeeds, 
            SwerveConstants.AutoConstants.pathFollowerConfig,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
        setDefaultCommand(new SwerveDriveCommand(this));
    }

    @Override
    public void periodic()
    {
        // Update the odometry in the periodic block
        m_odometry.update(Rotation2d.fromDegrees(-ahrs.getAngle()), new SwerveModulePosition[]
        {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        });
        SmartDashboard.putNumber("PoseX", m_odometry.getPoseMeters().getTranslation().getX());
        SmartDashboard.putNumber("PoseY", m_odometry.getPoseMeters().getTranslation().getY());
        SmartDashboard.putNumber("gyro", getHeading());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose()
    {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose)
    {
        m_odometry.resetPosition(Rotation2d.fromDegrees(-ahrs.getAngle()), new SwerveModulePosition[]
        {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        }, pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     * @param rateLimit Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
            boolean rateLimit)
    {

        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit)
        {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            double directionSlewRate;
            if (m_currentTranslationMag != 0.0)
            {
                directionSlewRate = Math.abs(SwerveConstants.DriveConstants.kDirectionSlewRate
                        / m_currentTranslationMag);
            }
            else
            {
                directionSlewRate = 500.0; // some high number that means the slew rate is
                                           // effectively instantaneous
            }


            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif =
                    SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
            if (angleDif < 0.45 * Math.PI)
            {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir,
                        inputTranslationDir, directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            }
            else if (angleDif > 0.85 * Math.PI)
            {
                if (m_currentTranslationMag > 1e-4)
                { // some small number to avoid floating-point errors with equality checking
                  // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                }
                else
                {
                    m_currentTranslationDir =
                            SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            }
            else
            {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir,
                        inputTranslationDir, directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }
            m_prevTime = currentTime;

            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
            m_currentRotation = m_rotLimiter.calculate(rot);


        }
        else
        {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            m_currentRotation = rot;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered =
                xSpeedCommanded * SwerveConstants.DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered =
                ySpeedCommanded * SwerveConstants.DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = m_currentRotation * SwerveConstants.DriveConstants.kMaxAngularSpeed;

        
        driveSpeeds(fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered,
                                rotDelivered,m_odometry.getPoseMeters().getRotation())
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    }

    public void driveSpeeds(ChassisSpeeds speeds){
        var swerveModuleStates =
                SwerveConstants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                SwerveConstants.DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX()
    {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                SwerveConstants.DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = m_frontLeft.getState();
        states[1] = m_frontRight.getState();
        states[2] = m_rearLeft.getState();
        states[3] = m_rearRight.getState();
        return states;
    }

    public ChassisSpeeds getSpeeds() {
        return SwerveConstants.DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }
    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders()
    {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    // public void zeroHeading()
    // {
    //     ahrs.reset();
    // }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading()
    {
        return m_odometry.getPoseMeters().getRotation().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate()
    {
        return ahrs.getRate() * (SwerveConstants.DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
}
