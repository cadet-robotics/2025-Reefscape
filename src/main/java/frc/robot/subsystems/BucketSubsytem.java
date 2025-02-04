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

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class BucketSubsytem extends CSubsystem {

    // Creating a sparkmax to control the motor
    // Snowblower motors must be set as "kBrushed"
    private final SparkMax m_snowblowerMotor = new SparkMax( Constants.BucketSubsytem.kSnowblowerMotor, MotorType.kBrushed );
    // Creating the Encoder
    private AbsoluteEncoder m_snowblowerEncoder;

    // Create an instace of the BucketSubsystem
    public BucketSubsytem() {
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
    
    //TODO: Fix Bucket Subsystem: "ERROR  4  [CAN SPARK] IDs: 21, WPILib or External HAL Error: CAN: Message not found Periodic Status 5"

    // Create all the bindings for this Subsystem
    public void buttonBindings( PS4Controller m_driverController ) {

        new JoystickButton(m_driverController, Button.kR1.value )
            .whileTrue( BucketDump() );

        new JoystickButton(m_driverController, Button.kL1.value )
            .whileTrue( BucketLoad() );

        new JoystickButton(m_driverController, Button.kShare.value )
            .whileTrue( BucketStart() );
    }

    // Bucket to start position
    public CCommand BucketStart() {
        return cCommand_( "BucketSubsytem.BucketStart" )
            .onExecute( () -> {
                // Checks whether the motor has to spin forward, backward, or not at all to get to the start position
                if ( m_snowblowerEncoder.getPosition() < Constants.BucketSubsytem.kStartPosition ) {
                    m_snowblowerMotor.set( Constants.BucketSubsytem.SnowblowerSpeed );
                } else if ( m_snowblowerEncoder.getPosition() > Constants.BucketSubsytem.kStartPosition ) {
                    m_snowblowerMotor.set( -Constants.BucketSubsytem.SnowblowerSpeed );
                } else {
                    m_snowblowerMotor.stopMotor();
                }
            })
            .onEnd( () -> {
                m_snowblowerMotor.stopMotor();
            });
    }

    // Bucket to dump position
    public CCommand BucketDump() {
        return cCommand_( "BucketSubsytem.BucketStart" )
            .onExecute( () -> {
                if ( m_snowblowerEncoder.getPosition() < Constants.BucketSubsytem.kDumpPosition ) {
                    m_snowblowerMotor.set( Constants.BucketSubsytem.SnowblowerSpeed );
                } else if ( m_snowblowerEncoder.getPosition() > Constants.BucketSubsytem.kDumpPosition ) {
                    m_snowblowerMotor.set( -Constants.BucketSubsytem.SnowblowerSpeed );
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
        return cCommand_( "BucketSubsytem.BucketLoad" )
            .onExecute( () -> {
                if ( m_snowblowerEncoder.getPosition() < Constants.BucketSubsytem.kLoadPosition ) {
                    m_snowblowerMotor.set( Constants.BucketSubsytem.SnowblowerSpeed );
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
