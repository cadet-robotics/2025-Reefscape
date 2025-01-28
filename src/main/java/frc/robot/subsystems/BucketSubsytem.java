// TODO: Setup encoders for the snowblower motor

package frc.robot.subsystems;

import frc.robot.lib.custom.CSubsystem;
import frc.robot.Constants;
import frc.robot.lib.custom.CCommand;

import frc.robot.Constants;
import frc.robot.Configs;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class BucketSubsytem extends CSubsystem {

    private final SparkMax m_snowblower = new SparkMax( Constants.BucketSubsytem.kSnowblowerMotor, MotorType.kBrushed );
    private final Encoder m_snowblowerEncoder = new Encoder( Constants.BucketSubsytem.kSnowblowerEncoderA, Constants.BucketSubsytem.kSnowblowerEncoderB );

    public BucketSubsytem() {
        m_snowblower.configure( 
            Configs.BucketSubsystem.kSnowblowerConfig,
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters 
        );
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

    public void buttonBindings( PS4Controller m_driverController ) {
        new JoystickButton(m_driverController, Button.kR1.value )
            .whileTrue( BucketDump() );

        new JoystickButton(m_driverController, Button.kL1.value )
            .whileTrue( BucketLoad() );

        new JoystickButton(m_driverController, Button.kShare.value )
            .whileTrue( BucketLoad() );
    }

    public CCommand BucketStart() {
        return cCommand_( "BucketSubsystem.BucketStart" )
            .onExecute( () -> {
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