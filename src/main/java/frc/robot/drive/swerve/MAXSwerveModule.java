// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class MAXSwerveModule
{
    private final SparkMax m_drivingSparkMax;
    private final SparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkClosedLoopController m_drivingPIDController;
    private final SparkClosedLoopController m_turningPIDController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
     * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs,
     * SPARKS MAX, and a Through Bore Encoder.
     */
    public MAXSwerveModule(int drivingId, int turningId, double chassisAngularOffset)
    {
        m_drivingSparkMax = new SparkMax(drivingId, MotorType.kBrushless);
        m_turningSparkMax = new SparkMax(turningId, MotorType.kBrushless);

        // DrivingSparkMaxConfig
        SparkBaseConfig driving_config = new SparkMaxConfig();
        driving_config
            .smartCurrentLimit(SwerveConstants.ModuleConstants.kDrivingMotorCurrentLimit)
            .idleMode( SwerveConstants.ModuleConstants.kDrivingMotorIdleMode )
        driving_config.encoder 
            .positionConversionFactor( SwerveConstants.ModuleConstants.kDrivingEncoderPositionFactor) 
            .velocityConversionFactor( SwerveConstants.ModuleConstants.kDrivingEncoderVelocityFactor); 
        driving_config.closedLoop
            .outputRange(
               SwerveConstants.ModuleConstants.kDrivingMinOutput,
                SwerveConstants.ModuleConstants.kDrivingMaxOutput
             )
            .feedbackSensor( m_drivingEncoder )
            .pid( 
                  SwerveConstants.ModuleConstants.kDrivingP,
                  SwerveConstants.ModuleConstants.kDrivingI,
                  SwerveConstants.ModuleConstants.kDrivingD
            );
        //  TurningSparkMax Config
        SparkBaseConfig turning_config = new SparkMaxConfig();
        turning_config
            .smartCurrentLimit(SwerveConstants.ModuleConstants.kTurningMotorCurrentLimit)
           .idleMode( SwerveConstants.ModuleConstants.kTurningMotorIdleMode )
           .inverted( SwerveConstants.ModuleConstants.kTurningEncoderInverted );
        turning_config.encoder
            .positionConversionFactor( SwerveConstants.ModuleConstants.kTurningEncoderPositionFactor)
            .velocityConversionFactor( SwerveConstants.ModuleConstants.kTurningEncoderVelocityFactor);
        turning_config.closedLoop
            .outputRange(SwerveConstants.ModuleConstants.kTurningMinOutput, SwerveConstants.ModuleConstants.kTurningMaxOutput)
            .feedbackSensor( m_turningEncoder )
            .positionWrappingEnabled( true ) 
            .positionPIDWrappingMaxInput( 
                SwerveConstants.ModuleConstants.kTurningEncoderPositionPIDMaxInput
             )
            .pid( 
               SwerveConstants.ModuleConstants.kTurningP,
               SwerveConstants.ModuleConstants.kTurningI,
               SwerveConstants.ModuleConstants.kTurningD
            );
        m_drivingPIDController.setFF(SwerveConstants.ModuleConstants.kDrivingFF);
        m_turningPIDController.setFF(SwerveConstants.ModuleConstants.kTurningFF);

        // Configure the motors
        m_drivingSparkMax.configure( 
           driving_config, 
           ResetMode.kResetSafeParameters, 
           PersistMode.kPersistParameters
        );
        m_turningSparkMax.configure( 
           turning_config, 
           ResetMode.kResetSafeParameters,
           PersistMode.kPersistParameters
        );

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState()
    {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(m_drivingEncoder.getVelocity(),
                new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition()
    {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(m_drivingEncoder.getPosition(),
                new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState)
    {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle =
                desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
                correctedDesiredState,
                new Rotation2d(m_turningEncoder.getPosition())
            );

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond,
                SparkMax.ControlType.kVelocity);
        m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(),
                SparkMax.ControlType.kPosition);

        m_desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders()
    {
        m_drivingEncoder.setPosition(0);
    }
}
