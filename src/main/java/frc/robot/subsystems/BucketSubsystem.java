package frc.robot.subsystems;

import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

import frc.robot.Configs;
import frc.robot.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class BucketSubsystem extends CSubsystem {

    // Creating a sparkmax to control the motor
    // Snowblower motors must be set as "kBrushed"
    private final SparkMax m_snowblowerMotor = new SparkMax( Constants.BucketSubsystem.kSnowblowerMotor, MotorType.kBrushed );
    // Creating the Encoder
    private DutyCycleEncoder m_snowblowerEncoder = new DutyCycleEncoder(Constants.BucketSubsystem.kSnowblowerMotor);

    // Create an instace of the BucketSubsystem
    public BucketSubsystem() {
        // Changes the brake type on the motor
        m_snowblowerMotor.configure( 
            Configs.BucketSubsystem.kSnowblowerConfig,
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters 
        );
    }

    public SparkMax getSnowblower() {
        return m_snowblowerMotor;
    }
    
    public DutyCycleEncoder getSnowblowerEncoder() {
        return m_snowblowerEncoder;
    }
    
    // Create all the bindings for this Subsystem
    public void buttonBindings( PS4Controller m_driverController, PS4Controller m_coDriverController ) {

        new JoystickButton(m_driverController, Button.kR1.value )
            .whileTrue( BucketDump() );

        new JoystickButton(m_driverController, Button.kL1.value )
            .whileTrue( BucketLoad() );

        new JoystickButton(m_driverController, Button.kShare.value )
            .whileTrue( BucketStart() );
    }

    // Bucket to start position
    public CCommand BucketStart() {
        return cCommand_( "BucketSubsystem.BucketStart" )
            .onExecute( () -> {
                // Checks whether the motor has to spin forward, backward, or not at all to get to the start position
                if ( m_snowblowerEncoder.get() < Constants.BucketSubsystem.kStartPosition ) {
                    m_snowblowerMotor.set( Constants.BucketSubsystem.SnowblowerSpeed );
                } else if ( m_snowblowerEncoder.get() > Constants.BucketSubsystem.kStartPosition ) {
                    m_snowblowerMotor.set( -Constants.BucketSubsystem.SnowblowerSpeed );
                } else {
                    m_snowblowerMotor.stopMotor();
                    SmartDashboard.putNumber("bucketPosition", 0);
                }
            })
            .onEnd( () -> {
                m_snowblowerMotor.stopMotor();
            });
    }

    // Bucket to dump position
    public CCommand BucketDump() {
        return cCommand_( "BucketSubsystem.BucketStart" )
            .onExecute( () -> {
                if ( m_snowblowerEncoder.get() < Constants.BucketSubsystem.kDumpPosition ) {
                    m_snowblowerMotor.set( Constants.BucketSubsystem.SnowblowerSpeed );
                } else if ( m_snowblowerEncoder.get() > Constants.BucketSubsystem.kDumpPosition ) {
                    m_snowblowerMotor.set( -Constants.BucketSubsystem.SnowblowerSpeed );
                } else {
                    m_snowblowerMotor.stopMotor();
                    SmartDashboard.putNumber("bucketPosition", 1);
                }
            })
            .onEnd( () -> {
                m_snowblowerMotor.stopMotor();
            });
    }

    // Bucket to load position
    public CCommand BucketLoad () {
        return cCommand_( "BucketSubsystem.BucketLoad" )
            .onExecute( () -> {
                if ( m_snowblowerEncoder.get() < Constants.BucketSubsystem.kLoadPosition ) {
                    m_snowblowerMotor.set( Constants.BucketSubsystem.SnowblowerSpeed );
                } else if ( m_snowblowerEncoder.get() > Constants.BucketSubsystem.kLoadPosition ) {
                    m_snowblowerMotor.set( -Constants.BucketSubsystem.SnowblowerSpeed );
                } else {
                    m_snowblowerMotor.stopMotor();
                    SmartDashboard.putNumber("bucketPosition", 2);
                }
            })
            .onEnd( () -> {
                m_snowblowerMotor.stopMotor();
            });
    }
}