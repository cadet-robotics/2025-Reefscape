package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

public class ElevatorSubsystem extends CSubsystem {

   // Motor Setup
    private static final SparkFlex m_elevatorMotor = new SparkFlex( 
        Constants.ElevatorSubsystem.kElevatorMotor, 
        MotorType.kBrushless 
    );

    // TODO: Pid tuning should be conducted on Mikey
    private static PIDController m_elevatorController = new PIDController(1.0, 0, 0);

    // Encoder Setup
    private static Encoder s_elevatorEncoder = new Encoder( 2, 3 );
        
    // Servo Setup
    // This servo is the brake for the elevator
    private static final Servo m_elevatorBrake = new Servo( 
        Constants.ElevatorSubsystem.kElevatorBrake 
    );

    // Top Limit Switch Setup
    private static final DigitalInput m_topLimitSwitch = new DigitalInput( 
          Constants.ElevatorSubsystem.kTopLimitSwitch
    );

    // Bottom Limit Switch Setup
    private static final DigitalInput m_bottomLimitSwitch = new DigitalInput( 
          Constants.ElevatorSubsystem.kBottomLimitSwitch
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
    }

    public void buttonBindings( PS4Controller m_driverController, PS4Controller m_coDriverController ) {

        // // Should be dpad up with a 10 degree margin for error on either side
        // new JoystickButton( m_driverController, m_driverController.getPOV() )
        //     .and( () -> Math.abs( m_driverController.getPOV() - 0 ) < 10)
        //     .whileTrue( 
        //         ElevatorLevelUp()
        //     );
        // // Should be dpad down with a 10 degree margin for error on either side
        //  new JoystickButton( m_driverController, m_driverController.getPOV() )
        //      .and( () -> Math.abs( m_driverController.getPOV() - 180 ) < 10)
        //     .whileTrue( 
        //         ElevatorLevelDown()
        //     );

        // TODO: @zach the POV buttons stopped working
        new JoystickButton( m_driverController, Button.kCross.value )
            .whileTrue( 
                ElevatorLevelUp()
            );
         new JoystickButton( m_driverController, Button.kTriangle.value )
            .whileTrue( 
                ElevatorLevelDown()
            );

        new JoystickButton(m_driverController, Button.kCircle.value )
           .whileTrue( EngageBrake() );
        new JoystickButton(m_driverController, Button.kSquare.value )
            .whileTrue( DisengageBrake() );
    }

    // Basic global reset button for the encoder. This should only be used in testing and at the time of startup
    public void resetEncoder() {
        s_elevatorEncoder.reset();
    }

    public double getState() {
        return s_elevatorEncoder.getDistance();
    }

    // This function should bring the elevator to the currently seleted level and is called after a level change
    // Should also stop the motor if it tries to go past the limit switch:w
    public void setDesiredState( double desiredState ) {
        double move = m_elevatorController.calculate( s_elevatorEncoder.getDistance(), desiredState );
        if ( move > 0 && !m_topLimitSwitch.get() ) {
            m_elevatorMotor.set( move/100.0 );
        } else {
            if ( move < 0 && !m_bottomLimitSwitch.get() ) {
                m_elevatorMotor.set( move/100.0 );
            } else {
                m_elevatorMotor.set( 0);
            }
        }
    }

    @Override
    public void periodic() {

        SmartDashboard.putString( "ElevatorLevel", Constants.ElevatorSubsystem.LevelNames[level] );
        SmartDashboard.putNumber( "Encoder", s_elevatorEncoder.getDistance() );

        setDesiredState( Constants.ElevatorSubsystem.LevelHeights[level] );

    }
     
    // Increase the elevator level by one if it's not already at the max level ( 8 )
    public CCommand ElevatorLevelUp() {
        return cCommand_( "ElevatorSubsystem.ElevatorLevelUp" )
            .onInitialize( () -> {
                if ( level < 8 ) {
                    level = level + 1;
                }
            });
    }
    // Decrease the elevator level by one if it's not already at the bottom level
    public CCommand ElevatorLevelDown() {
        return cCommand_( "ElevatorSubsystem.ElevatorLevelDown" )
            .onInitialize( () -> {
                if ( level > 0 ) {
                    level = level - 1;
                }
            });
    }

    // Moves the servo to a posisiton that will stop elevator motion
    public CCommand EngageBrake() {
        return cCommand_( "ElevatorSubsystem.EngageBrake")
            // Filler code TODO: must be changed when migrating to mikey
            .onInitialize( () -> {
                m_elevatorBrake.set( 0 );
            });
    }

    // Moves the servo to a posisiton that will allow elevator motion
    public CCommand DisengageBrake() {
        return cCommand_( "ElevatorSubsystem.DisengageBrake")
            // Filler code TODO: must be changed when migrating to mikey
            .onInitialize( () -> {
                m_elevatorBrake.set( 1 );
            });
    }
}
