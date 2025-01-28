package frc.robot.subsystems;

import frc.robot.lib.custom.*;
import frc.robot.Constants;
import frc.robot.Configs;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ElevatorSubsystem extends CSubsystem {

   // Motor Setup
    private static final SparkMax m_elevatorMotor = new SparkMax( 
        Constants.ElevatorSubsystem.kElevatorMotor, 
        MotorType.kBrushless 
    );

    // Encoder Setup
    private static final Encoder m_elevatorEncoder = new Encoder( 
        Constants.ElevatorSubsystem.kElevatorEncoderA, 
        Constants.ElevatorSubsystem.kElevatorEncoderB 
    );

    // Servo Setup
    // This servo is the brake for the elevator
    private static final Servo m_elevatorBrake = new Servo( 
        Constants.ElevatorSubsystem.kElevatorBrake 
    );

    // Top Limit Switch Setup
    private static final DigitalInput m_topLimitSwitch = new DigitalInput( 
          Constants.ElevatorSubsystem.kTopLimitSwitchA, 
          Constants.ElevatorSubsystem.kTopLimitSwitchB 
    );

    // Bottom Limit Switch Setup
    private static final DigitalInput m_bottomLimitSwitch = new DigitalInput( 
          Constants.ElevatorSubsystem.kBottomLimitSwitchA, 
          Constants.ElevatorSubsystem.kBottomLimitSwitchB 
    );

    // The number corresponding to the level of the elevator
    // This should be a value between 0 and 8 ( There are 9 levels but a 0 based system will be used )
    private static int level = 0;

    // Creates the Elevator Subsystem
    public ElevatorSubsystem() {
        // Configuring the brake mode on the elevator motor
        m_elevatorMotor.configure( 
            Configs.ElevatorSubsystem.kElevatorMotorConfig,
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters 
        );
        // Ecoder condig
        resetEncoder();
        // This value still needs to be found
        m_elevatorEncoder.setDistancePerPulse( 
            Constants.ElevatorSubsystem.kElevatorEncoderDistancePerPulse 
        );
    }

    // The following 5 functions are just in case the RobotContainer needs to access any of these; most likely for testing.
    public SparkMax getElevatorMotor() {
        return m_elevatorMotor;
    }
    public Encoder getElevatorEncoder() {
        return m_elevatorEncoder;
    }
    public Servo getElevatorBrake() {
        return m_elevatorBrake;
    }
    public DigtalInput getTopLimitSwitch() {
       return m_topLimitSwitch;
    }
    public DigtalInput getBottomLimitSwitch() {
       return m_bottomLimitSwitch;
    }
    
    // Basic global reset button for the encoder. This should only be used in testing and at the time of startup
    public void resetEncoder() {
        m_elevatorEncoder.reset();
    }

    // TODO: Find how to do bindings via the PS4Controller up and down dpad buttons
    public void buttonBindings( PS4Controller m_driverController ) {}

    // This function should bring the elevator to the currently seleted level and is called after a level change
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

    // Increase the elevator level by one if it's not already at the max level ( 8 )
    public CCommand ElevatorLevelUp() {
        return cCommand_( "ElevatorSubsystem.ElevatorLevelUp" )
            .onExecute( () -> {
                if ( level < 8 ) {
                    level = level + 1;
                }
            })
            .onEnd( () -> {
               ToLevel();
            });
    }
    // Decrease the elevator level by one if it's not already at the bottom level
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
