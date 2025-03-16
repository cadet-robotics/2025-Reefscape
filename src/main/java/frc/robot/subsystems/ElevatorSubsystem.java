package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

//Press R2 on CODRIVER CONTROLLER to raise the elevator
//Press L2 on CODRIVER CONTROLLER to lower the elevator

public class ElevatorSubsystem extends CSubsystem {

   // Elevator motor Setup
    private static final SparkFlex m_elevatorMotor = new SparkFlex( 
        Constants.ElevatorSubsystem.kElevatorMotor, 
        MotorType.kBrushless 
    );

    private static final SparkClosedLoopController pidController = m_elevatorMotor.getClosedLoopController();

    private static final TrapezoidProfile elevatorProfile = new TrapezoidProfile( new TrapezoidProfile.Constraints(90,200));
    private static TrapezoidProfile.State TrapezoidProfileState = new TrapezoidProfile.State();

    // Encoder Setup
    public static RelativeEncoder s_elevatorEncoder; // = new Encoder( Constants.ElevatorSubsystem.kElevatorEncoderA, Constants.ElevatorSubsystem.kElevatorEncoderB );
    private static boolean isManual = true;
        
    // Servo Setup
    // This servo is the brake for the elevator
    private static final Servo m_elevatorBrake = new Servo( 
        Constants.ElevatorSubsystem.kElevatorBrake 
    );

    // Top Limit Switch Setup
    // Limit SwitchES ARE REVERSED
    private static final DigitalInput m_topLimitSwitch = new DigitalInput( 
          Constants.ElevatorSubsystem.kTopLimitSwitch
    );

    // Bottom Limit Switch Setup
    //limit switch values are reversed
    private static final DigitalInput m_bottomLimitSwitch = new DigitalInput( 
          Constants.ElevatorSubsystem.kBottomLimitSwitch
    );

    // Tracks if the elevator has been zeroed yet
    private static boolean hasBeenZeroed = false;

    private static Timer m_breakTimer = new Timer();

    // The number corresponding to the level of the elevator
    // This should be a value between 0 and 8 ( There are 9 levels but a 0 based system will be used )
    private static int level = 0;

    private static boolean wasManual = true;

    /**
     * Checks if the robot should be in slow mode based on the position of the elevator
     */
    public final BooleanSupplier elevatorSlowCheck = ()->{
        if ( s_elevatorEncoder.getPosition() > Constants.ElevatorSubsystem.kElevatorSlowThreashold ) {
            return true;
        }
        return false;
    };

    /**
     * Function called when the robot gets disabled
     * This is used to make re-enabling the robot safe regardless of prior state
     */
    public void OnDisable() {
        level = 0;
        isManual = true;
        wasManual = true;
        m_elevatorMotor.stopMotor();
    }

    public void startTimer() {
        m_breakTimer.start();
    }

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
        s_elevatorEncoder = m_elevatorMotor.getEncoder();
    }

    private boolean topLimitPressed() {
        return !m_topLimitSwitch.get();
    }

    private boolean bottomLimitPressed() {
        return !m_bottomLimitSwitch.get();
    }

    /**
     * Does all of the button bindings for the subsystem
     *
     * @param m_driverController The main driver controller
     * @param m_coDriverController The co-driver controller
     */
    public void buttonBindings( PS4Controller m_driverController, PS4Controller m_coDriverController ) {

        // EngageBrake ( Right Bumper )
        // new JoystickButton(m_driverController, Constants.DriverControls.enableBreak )
        //    .whileTrue( EngageBrake() );

        // DisengageBrake ( Left Bumper )
        // new JoystickButton(m_driverController, Constatns.CoDriverControls.disableBreak )
        //     .whileTrue( DisengageBrake() );

        new JoystickButton(m_coDriverController, Button.kR1.value )
            .whileTrue( ElevatorLevelUp() );

        new JoystickButton(m_coDriverController, Button.kL1.value )
            .whileTrue( ElevatorLevelDown() );

        new JoystickButton(m_coDriverController, Constants.CoDriverControls.elevatorUpManual )
            .whileTrue( ElevatorDoUp() );

        new JoystickButton(m_coDriverController, Constants.CoDriverControls.elevaotrDownManual )
            .whileTrue( ElevatorDoDown() );

        new JoystickButton(m_driverController, Constants.DriverControls.bucketLoadPositionButton )
            .whileTrue( new RunCommand( () -> {
                level = 3;
                isManual = false;
            }, 
            this 
        ));

    }

    /** 
     * Sets the desired state for the elevator motor
     * 
     * @param desiredState The desired state for the motor
     */
    public void setDesiredState( double desiredState ) {

        TrapezoidProfileState = elevatorProfile.calculate( TimedRobot.kDefaultPeriod, TrapezoidProfileState, new TrapezoidProfile.State( desiredState, 0 ));
        double desiredPosition = TrapezoidProfileState.position;
    
        SmartDashboard.putNumber( "State" ,  desiredPosition );
        SmartDashboard.putNumber( "DesiredState" , desiredState );

        pidController.setReference( desiredPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0 );

    }

    /**
     * Periodic
     * Frequently checks the elevator level and encoder position, both get sent to the dashboard.
     * Sets the elevator motor's desired state to the position that coresponds to the selecected level
     */
    @Override
    public void periodic() {

        SmartDashboard.putBoolean( "ElevatorBottom", bottomLimitPressed() );
        SmartDashboard.putBoolean( "ElevatorTop", topLimitPressed() );
        SmartDashboard.putNumber( "ElevatorLevel", level );
        SmartDashboard.putNumber( "Encoder", s_elevatorEncoder.getPosition() );
        SmartDashboard.putBoolean( "ElevatorSlow", elevatorSlowCheck.getAsBoolean() );

        if ( !hasBeenZeroed ) {
            if ( bottomLimitPressed() )
            {
                s_elevatorEncoder.setPosition(0);
            }
            hasBeenZeroed = true;
        }

        if ( !isManual ) {
            if ( wasManual ) {
                TrapezoidProfileState = new TrapezoidProfile.State( s_elevatorEncoder.getPosition(), s_elevatorEncoder.getVelocity()/60);
            }
            setDesiredState( Constants.ElevatorSubsystem.LevelHeights[level] );
        }

        wasManual = isManual;

        // if ( m_breakTimer.get() <= Constants.ElevatorSubsystem.kBreakEngageTime ) {
        //     m_elevatorBrake.set( Constants.ElevatorSubsystem.kServoEnagedPos );
        // }
    }
     
    /**
     * ElevatorLevelUp
     * Increases the selected elevator level by one
     * Elevator Subsystem
     */
    public CCommand ElevatorLevelUp() {
        return cCommand_( "ElevatorSubsystem.ElevatorLevelUp" )
            .onInitialize( () -> {
                isManual = false;
                if ( level < 8 ) {
                    level = level + 1;
                    // TrapezoidProfileState = new TrapezoidProfile.State( s_elevatorEncoder.getPosition(), s_elevatorEncoder.getVelocity()/60);
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
                isManual = false;
                if ( level > 0 ) {
                    level = level - 1;
                    // TrapezoidProfileState = new TrapezoidProfile.State( s_elevatorEncoder.getPosition(), s_elevatorEncoder.getVelocity()/60);
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
            .onInitialize( () -> {
                m_elevatorBrake.set( Constants.ElevatorSubsystem.kServoDisenagedPos );
            });
    }

    public CCommand ElevatorDoUp() {
        return cCommand_( "ElevatorSubsystem.ElevatorDoUp")
            .onExecute( () -> {
                isManual = true;
                if ( !topLimitPressed() ) {
                    m_elevatorMotor.set( Constants.ElevatorSubsystem.kElevaotrManualSpeed);
                } else {
                    m_elevatorMotor.stopMotor();
                }
            })
            .onEnd( ()->{
                m_elevatorMotor.stopMotor();
            });
    }
    
    public CCommand ElevatorDoDown() {
        return cCommand_( "ElevatorSubsystem.ElevaotrDoDown")
            .onInitialize( () -> {
                isManual = true;
                if ( !bottomLimitPressed() ) {
                    m_elevatorMotor.set( -Constants.ElevatorSubsystem.kElevaotrManualSpeed);
                } else { 
                    m_elevatorMotor.stopMotor();
                }
            })
            .onEnd( ()->{
            });
    }
}