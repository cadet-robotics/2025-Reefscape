package frc.robot.subsystems;

import frc.robot.lib.custom.*;
import frc.robot.Constants;
import frc.robot.Configs;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ElevatorSubsystem extends CSubsystem {

    private static final SparkMax m_elevatorMotor = new SparkMax( 
        Constants.ElevatorSubsystem.kElevatorMotor, 
        MotorType.kBrushless 
    );
    private static final Encoder m_elevatorEncoder = new Encoder( 
        Constants.ElevatorSubsystem.kElevatorEncoderA, 
        Constants.ElevatorSubsystem.kElevatorEncoderB 
    );
    private static final Servo m_elevatorBrake = new Servo( 
        Constants.ElevatorSubsystem.kElevatorBrake 
    );

    // The number corresponding to the level of the elevator
    private static int level = 0;

    public ElevatorSubsystem() {
        m_elevatorMotor.configure( 
            Configs.ElevatorSubsystem.kElevatorMotorConfig,
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters 
        );
        m_elevatorEncoder.setDistancePerPulse( 
            Constants.ElevatorSubsystem.kElevatorEncoderDistancePerPulse 
        );
    }

    public SparkMax getElevatorMotor() {
        return m_elevatorMotor;
    }
    public Encoder getElevatorEncoder() {
        return m_elevatorEncoder;
    }
    public Servo getElevatorBrake() {
        return m_elevatorBrake;
    }
    public void resetEncoder() {
        m_elevatorEncoder.reset();
    }
    // TODO: Find how to do bindings via the PS4Controller up and down dpad buttons
    public void buttonBindings( PS4Controller m_driverController ) {}

    public CCommand ToLevel() {
        return cCommand_( "ElevatorSubsystem.ToLevel" )
            .onExecute( () -> {
                if ( m_elevatorEncoder.getDistance() > Constants.ElevatorSubsystem.LevelHeights[level] ) {
                    m_elevatorMotor.set( -Constants.ElevatorSubsystem.kElevatorSpeed );
                } else if ( m_elevatorEncoder.getDistance() < Constants.ElevatorSubsystem.LevelHeights[level] ) {
                    m_elevatorMotor.set( Constants.ElevatorSubsystem.kElevatorSpeed );
                } else {
                    m_elevatorMotor.stopMotor();
                    return;
                }
            });
    }
    public CCommand ElevatorLevelUp() {
        return cCommand_( "ElevatorSubsystem.ElevatorLevelUp" )
            .onExecute( () -> {
                if ( level < 8 ) {
                    level = level + 1;
                }
            })
            .onEnd( () -> {
                m_elevatorMotor.stopMotor();
            });
    }
    public CCommand ElevatorLevelDown() {
        return cCommand_( "ElevatorSubsystem.ElevatorLevelDown" )
            .onExecute( () -> {
                if ( level > 0 ) {
                    level = level - 1;
                }
            })
            .onEnd( () -> {
                ToLevel();
            });
    }

}