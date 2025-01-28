// TODO: Setup encoders for the snowblower motor

package frc.robot.subsystems;

import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

import frc.robot.Configs;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class BucketSubsytem extends CSubsystem {

    // Creating a sparkmax to control the motor
    // Snowblower motors must be set as "kBrushed"
    private final SparkMax m_snowblower = new SparkMax( Constants.BucketSubsytem.kSnowblowerMotor, MotorType.kBrushed );
    // Creating the Encoder
    private final Encoder m_snowblowerEncoder = new Encoder( Constants.BucketSubsytem.kSnowblowerEncoderA, Constants.BucketSubsytem.kSnowblowerEncoderB );

    // Create an instace of the BucketSubsystme
    public BucketSubsytem() {
        // Changes the brake type on the motor
        m_snowblower.configure( 
            Configs.BucketSubsystem.kSnowblowerConfig,
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters 
        );
        // Encoder Setup
        resetEncoder();
        // Value Still to be found
        m_snowblowerEncoder.setDistancePerPulse( Constants.BucketSubsytem.kSnowblowerEncoderDistancePerPulse );
    }

    public SparkMax getSnowblower() {
        return m_snowblower;
    }
    
    public Encoder getSnowblowerEncoder() {
        return m_snowblowerEncoder;
    }
    
    public void resetEncoder() {
        m_snowblowerEncoder.reset();
    }

    // Create all the bindings for this Subsystem
    public void buttonBindings( PS4Controller m_driverController ) {

        new JoystickButton(m_driverController, Button.kR1.value )
            .whileTrue( BucketDump() );

        new JoystickButton(m_driverController, Button.kL1.value )
            .whileTrue( BucketLoad() );

        new JoystickButton(m_driverController, Button.kShare.value )
            .whileTrue( BucketLoad() );
    }

    // Bucket to start position
    public CCommand BucketStart() {
        return cCommand_( "BucketSubsystem.BucketStart" )
            .onExecute( () -> {
                // Checks wither the motor has to spin forward, backword, or not at all to get to the start position
                if ( m_snowblowerEncoder.getDistance() < Constants.BucketSubsytem.kStartPosition ) {
                    m_snowblower.set( Constants.BucketSubsytem.SnowblowerSpeed );
                } else if ( m_snowblowerEncoder.getDistance() > Constants.BucketSubsytem.kStartPosition ) {
                    m_snowblower.set( -Constants.BucketSubsytem.SnowblowerSpeed );
                } else {
                    m_snowblower.stopMotor();
                }
            })
            .onEnd( () -> {
                m_snowblower.stopMotor();
            });
    }

    // Bucket to dump position
    public CCommand BucketDump() {
        return cCommand_( "BucketSubsystem.BucketStart" )
            .onExecute( () -> {
                if ( m_snowblowerEncoder.getDistance() < Constants.BucketSubsytem.kDumpPosition ) {
                    m_snowblower.set( Constants.BucketSubsytem.SnowblowerSpeed );
                } else if ( m_snowblowerEncoder.getDistance() > Constants.BucketSubsytem.kDumpPosition ) {
                    m_snowblower.set( -Constants.BucketSubsytem.SnowblowerSpeed );
                } else {
                    m_snowblower.stopMotor();
                }
            })
            .onEnd( () -> {
                m_snowblower.stopMotor();
            });
    }

    // Bucket to load position
    public CCommand BucketLoad () {
        return cCommand_( "BucketSubsystem.BucketLoad" )
            .onExecute( () -> {
                if ( m_snowblowerEncoder.getDistance() < Constants.BucketSubsytem.kLoadPosition ) {
                    m_snowblower.set( Constants.BucketSubsytem.SnowblowerSpeed );
                } else if ( m_snowblowerEncoder.getDistance() > Constants.BucketSubsytem.kLoadPosition ) {
                    m_snowblower.set( -Constants.BucketSubsytem.SnowblowerSpeed );
                } else {
                    m_snowblower.stopMotor();
                }
            })
            .onEnd( () -> {
                m_snowblower.stopMotor();
            });
    }
}
