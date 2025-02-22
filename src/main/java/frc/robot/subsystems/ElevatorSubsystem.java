package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
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

   // Elevator motor Setup
    private static final SparkFlex m_elevatorMotor = new SparkFlex( 
        Constants.ElevatorSubsystem.kElevatorMotor, 
        MotorType.kBrushless 
    );

    // TODO: Pid tuning should be conducted on Mikey
    private static PIDController m_elevatorController = new PIDController(1.0, 0, 0);

    // Encoder Setup
    private static Encoder s_elevatorEncoder = new Encoder( Constants.ElevatorSubsystem.kElevatorEncoderA, Constants.ElevatorSubsystem.kElevatorEncoderB );
        
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

    private static Timer m_breakTimer = new Timer();

    // The number corresponding to the level of the elevator
    // This should be a value between 0 and 8 ( There are 9 levels but a 0 based system will be used )
    private static int level = 0;

    /**
     * Checks if the robot should be in slow mode based on the position of the elevator
     */
    public final BooleanSupplier elevatorSlowCheck = ()->{
        if ( s_elevatorEncoder.getDistance() > Constants.ElevatorSubsystem.kElevatorSlowThreashold ) {
            return true;
        }
        return false;
    };

    /**
     * Creates the elevator subsystem
     */
    public ElevatorSubsystem() {
        // Configuring the brake mode on the elevator motor
        m_elevatorMotor.configure( 
            Configs.ElevatorSubsystem.kElevatorMotorConfig,
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters 
        );
    }

    public void startTimer() {
        m_breakTimer.start();
    }

    /**
     * Does all of the button bindings for the subsystem
     *
     * @param m_driverController The main driver controller
     * @param m_coDriverController The co-driver controller
     */
    public void buttonBindings( PS4Controller m_driverController, PS4Controller m_coDriverController ) {

        // Should be dpad up with a 10 degree margin for error on either side
        // ElevatorLevelUp ( Dpad Up ) ( Co Driver )
        new JoystickButton( m_driverController, m_driverController.getPOV() )
            .and( () -> Math.abs( m_coDriverController.getPOV() - 0 ) < 10)
                .whileTrue( ElevatorLevelUp() );
        // ElevatorLevelDown ( Dpad Down ) ( Co Driver )
        // Should be dpad down with a 10 degree margin for error on either side
         new JoystickButton( m_coDriverController, m_driverController.getPOV() )
            .and( () -> Math.abs( m_driverController.getPOV() - 180 ) < 10)
                .whileTrue( ElevatorLevelDown() );

        // EngageBrake ( Right Bumper )
        new JoystickButton(m_driverController, Button.kSquare.value )
           .whileTrue( EngageBrake() );

        // DisengageBrake ( Left Bumper )
        new JoystickButton(m_driverController, Button.kTriangle.value )
            .whileTrue( DisengageBrake() );
    }

    /** 
     * Sets the desired state for the elevator motor
     * 
     * @param desiredState The desired state for the motor
     */
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

    /**
     * Periodic
     * Frequently checks the elevator level and encoder position, both get sent to the dashboard.
     * Sets the elevator motor's desired state to the position that coresponds to the selecected level
     */
    @Override
    public void periodic() {

        SmartDashboard.putString( "ElevatorLevel", Constants.ElevatorSubsystem.LevelNames[level] );
        SmartDashboard.putNumber( "Encoder", s_elevatorEncoder.getDistance() );

        setDesiredState( Constants.ElevatorSubsystem.LevelHeights[level] );

        if ( m_breakTimer.get() <= Constants.ElevatorSubsystem.kBreakEngageTime ) {
            m_elevatorBrake.set( Constants.ElevatorSubsystem.kServoEnagedPos );
        }
    }
     
    /**
     * ElevatorLevelUp
     * Increases the selected elevator level by one
     * Elevator Subsystem
     */
    public CCommand ElevatorLevelUp() {
        return cCommand_( "ElevatorSubsystem.ElevatorLevelUp" )
            .onInitialize( () -> {
                if ( level < 8 ) {
                    level = level + 1;
                }
            });
    }
    /**
     * ElevatorLevelDown
     * Reduces the selected elevator level by one
     * Elevator Subsystem
     */
    public CCommand ElevatorLevelDown() {
        return cCommand_( "ElevatorSubsystem.ElevatorLevelDown" )
            .onInitialize( () -> {
                if ( level > 0 ) {
                    level = level - 1;
                }
            });
    }

    /**
     * EngageBreak
     * Moves the brake to the brake position
     * Elevator Subsystem
     */
    public CCommand EngageBrake() {
        return cCommand_( "ElevatorSubsystem.EngageBrake")
            // Filler code TODO: must be changed when migrating to mikey
            .onInitialize( () -> {
                m_elevatorBrake.set( Constants.ElevatorSubsystem.kServoEnagedPos );
            });
    }

    /**
     * DisengageBrake
     * Moves the brake to the starting position
     * Elevator Subsystem
     */
    public CCommand DisengageBrake() {
        return cCommand_( "ElevatorSubsystem.DisengageBrake")
            // Filler code TODO: must be changed when migrating to mikey
            .onInitialize( () -> {
                m_elevatorBrake.set( Constants.ElevatorSubsystem.kServoDisenagedPos );
            });
    }
}
