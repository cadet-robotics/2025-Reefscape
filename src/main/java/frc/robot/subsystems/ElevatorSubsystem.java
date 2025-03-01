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

//Press R2 on CODRIVER CONTROLLER to raise the elevator
//Press L2 on CODRIVER CONTROLLER to lower the elevator

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
    // Limit SwitchES ARE REVERSED
    private static final DigitalInput m_topLimitSwitch = new DigitalInput( 
          Constants.ElevatorSubsystem.kTopLimitSwitch
    );

    // Bottom Limit Switch Setup
    //limit switch values are reversed
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
        // new JoystickButton( m_coDriverController, m_driverController.getPOV() )
        //     .and( () -> Math.abs( m_coDriverController.getPOV() - 0 ) < 10)
        //         .whileTrue( ElevatorLevelUp() );
        // // ElevatorLevelDown ( Dpad Down ) ( Co Driver )
        // // Should be dpad down with a 10 degree margin for error on either side
        //  new JoystickButton( m_coDriverController, m_driverController.getPOV() )
        //     .and( () -> Math.abs( m_coDriverController.getPOV() - 180 ) < 10)
        //         .whileTrue( ElevatorLevelDown() );

        // EngageBrake ( Right Bumper )
        // new JoystickButton(m_driverController, Button.kTriangle.value )
        //    .whileTrue( EngageBrake() );

        // DisengageBrake ( Left Bumper )
        // new JoystickButton(m_driverController, Button.kCross.value )
        //     .whileTrue( DisengageBrake() );

        new JoystickButton(m_coDriverController, Constants.CoDriverControls.upDPadElevatorUp )
            .whileTrue( ElevatorLevelUp() );

        new JoystickButton(m_coDriverController, Constants.CoDriverControls.downDPadElevatorDown )
            .whileTrue( ElevatorLevelDown() );
    }

    /** 
     * Sets the desired state for the elevator motor
     * 
     * @param desiredState The desired state for the motor
     */
    public void setDesiredState( double desiredState ) {
        double move = m_elevatorController.calculate( s_elevatorEncoder.getDistance(), desiredState );
        //limit switch values are reversed
        if ( move > 0 && !m_topLimitSwitch.get() && move < Constants.ElevatorSubsystem.PidMax ) {
            m_elevatorMotor.set( move );
        } else {
            //limit switch values are reversed
            if ( move < 0 && !m_bottomLimitSwitch.get() && move > -Constants.ElevatorSubsystem.PidMax ) {
                m_elevatorMotor.set( move );
            } else {
                m_elevatorMotor.stopMotor();
            }
        }
    }

    /** Simple function to make it simple to grab the disstance to the target */
    private static double distanceTo( double target ) {
        return Math.abs( target * target - s_elevatorEncoder.get() * s_elevatorEncoder.get() );
    }
    
    /** A worse but easier to tune version of control for the elevator if we can't get pid to work */
    private static void crappyPID( double target ) {
        // Use the set slow speed to get to the target
        if ( distanceTo( target ) <= Constants.ElevatorSubsystem.CrappyPid.kElevatorStopThreshold ) {
            m_elevatorMotor.stopMotor();
            // Uncomment if we need a set a speed to fight the gravity when trying to hover and comment the prevois line
            // m_elevatorMotor.set( Constants.ElevatorSubsystem.CrappyPid.kElevatorHoverSpeed );
        } else if ( distanceTo( target ) > Constants.ElevatorSubsystem.CrappyPid.kElevatorSlowDistanceThreashold ) {
            if ( s_elevatorEncoder.get() > target && m_bottomLimitSwitch.get() ) { 
                // Go down
                m_elevatorMotor.set( -Constants.ElevatorSubsystem.CrappyPid.kElevatorSlowSpeed );
            } else if ( s_elevatorEncoder.get() < target && m_topLimitSwitch.get() ) { //  
                // Go up
                m_elevatorMotor.set( Constants.ElevatorSubsystem.CrappyPid.kElevatorSlowSpeed );
            }
        } else {

            // Full speed if distance is not within the threshold
            if ( s_elevatorEncoder.get() > target && m_bottomLimitSwitch.get() ) { 
                // Go down
                m_elevatorMotor.set( -Constants.ElevatorSubsystem.CrappyPid.kElevatorNormSpeed );
            } else if ( s_elevatorEncoder.get() < target && m_topLimitSwitch.get()  ) { // m_topLimitSwitch.get() 
                // Go up
                m_elevatorMotor.set( Constants.ElevatorSubsystem.CrappyPid.kElevatorNormSpeed );
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

        //limit switch values are reversed 
        SmartDashboard.putBoolean( "ElevatorBottom", m_bottomLimitSwitch.get() );
        SmartDashboard.putBoolean( "ElevatorTop", m_topLimitSwitch.get() );
        SmartDashboard.putString( "ElevatorLevel", Constants.ElevatorSubsystem.LevelNames[level] );
        SmartDashboard.putNumber( "Encoder", s_elevatorEncoder.getDistance() );

        // setDesiredState( Constants.ElevatorSubsystem.LevelHeights[level] );

        // if ( m_breakTimer.get() <= Constants.ElevatorSubsystem.kBreakEngageTime ) {
        //     m_elevatorBrake.set( Constants.ElevatorSubsystem.kServoEnagedPos );
        // }

        // Uncomment the bollow line to test when and only when ready
        // crappyPID(Constants.ElevatorSubsystem.LevelHeights[level]);
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

    public CCommand ElevatorDoUp() {
        return cCommand_( "ElevatorSubsystem.ElevatorDoUp")
            // Filler code TODO: must be changed when migrating to mikey
            .onExecute( () -> {
                // Limit Switches are reversed
                if ( m_topLimitSwitch.get() ) {
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
            // Filler code TODO: must be changed when migrating to mikey
            .onInitialize( () -> {
                // LIMIT SWITCHES ARE REVERSED
                if ( m_bottomLimitSwitch.get() ) {
                    m_elevatorMotor.set( -Constants.ElevatorSubsystem.kElevaotrManualSpeed);
                } else { 
                    m_elevatorMotor.stopMotor();
                }
            })
            .onEnd( ()->{
                m_elevatorMotor.stopMotor();
            });
    }

    // TODO: at some point we can try to have this run on startup
    /** A command that can be used to zero the elevator encoder by moving it down to the bottom limit switch */
    public CCommand ZeroElevator() {
        return cCommand_( "ElevatorSubsystem.ZeroElevator")
            .onExecute( () -> {
                // LIMIT SWITCHES ARE REVERSED
                if ( m_bottomLimitSwitch.get() ) {
                    m_elevatorMotor.set( -Constants.ElevatorSubsystem.kElevaotrManualSpeed );
                } else {
                    m_elevatorMotor.stopMotor();
                    s_elevatorEncoder.reset();
                }
            })
            // Stop the motor in the off chance that execution of onExecute stopped before it stopped the motor
            .onEnd( () -> { m_elevatorMotor.stopMotor();});
    }
}