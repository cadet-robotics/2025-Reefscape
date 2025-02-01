package frc.robot.subsystems;

import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

import frc.robot.Configs;
import frc.robot.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class BucketSubsytem extends CSubsystem {

    // Creating a sparkmax to control the motor
    // Snowblower motors must be set as "kBrushed"
    private final SparkFlex m_snowblowerMotor = new SparkMax( Constants.BucketSubsytem.kSnowblowerMotor, MotorType.kBrushed );
    // Creating the Encoder
    private RelativeEncoder m_snowblowerEncoder;

    // Create an instace of the BucketSubsystme
    public int BucketSubsytem() {
        // Changes the brake type on the motor
        m_snowblowerMotor.configure( 
            Configs.BucketSubsystem.kSnowblowerConfig,
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters 
        );
        m_snowblowerEncoder = m_snowblowerMotor.getAbsoluteEncoder();
    }

    public SparkMax getSnowblower() {
        return m_snowblowerMotor;
    }
    
    public AbsoluteEncoder getSnowblowerEncoder() {
        return m_snowblowerEncoder;
    }
    
    // Create all the bindings for this Subsystem
    public void buttonBindings( PS4Controller m_driverController ) {

        new JoystickButton(m_driverController, Button.kR1 )
            whileTrue( BucketDump() )

        new JoystickButton(m_driverController, Button.kL1 )
            whileTrue( BucketLoad() )

        new JoystickButton(m_driverController, Button.kShare )
            whileTrue( BucketLoad() )
    }

    // Bucket to start position
    public CCommand BucketStart() {
        return cCommand_( "BucketSubsystem.BucketStart" )
            .onExecute( () -> {
                // Checks wither the motor has to spin forward, backword, or not at all to get to the start position
                if ( m_snowblowerEncoder.getPosition() < Constants.BucketSubsytem.kStartPosition ) {
                    m_snowblowerMotor.set( Constants.BucketSubsytem.SnowblowerSpeed );
                } else if ( m_snowblowerEncoder.getPosition() > Constants.BucketSubsytem.kStartPosition ) {
                    m_snowblowerMotor.set( -Constants.BucketSubsytem.SnowblowerSpeed );
                } else {
                    m_snowblowerMotor.stopMotor();
                }
            })
            .onEnd( () -> {
                m_snowblowerMotor.Motor();
            });
    }

    // Bucket to dump position
    public CCommand BucketDump() {
        return cCommand_( "BucketSubsystem.BucketStart" )
            .onExecute( () -> {
                if ( m_snowblowerEncoder.getPosition() < Constants.BucketSubsytem.kDumpPosition ) {
                    m_snowblowerMotor.set( Constants.BucketSubsytem.SnowblowerSpeed );
                } else if ( m_snowblowerEncoder.getPosition() > Constants.BucketSubsytem.kDumpPosition ) {
                    m_snowblowerMotor.set( -Constants.BucketSubsytem );
                } else {
                    m_snowblowerMotor.stopMotor();
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
                if ( m_snowblowerEncoder.getPosition() < Constants.BucketSubsytem.kLoadPosition ) {
                    m_snowblowerMotor.speed( Constants.BucketSubsytem.SnowblowerSpeed );
                } else if ( m_snowblowerEncoder.getPosition() > Constants.BucketSubsytem.kLoadPosition ) {
                    m_snowblowerMotor.set( -Constants.BucketSubsytem.SnowblowerSpeed );
                } else {
                    m_snowblowerMotor.stopMotor();
                }
            })
            .onEnd( () -> {
                m_snowblowerMotor.stopMotor();
            });
    }
}
