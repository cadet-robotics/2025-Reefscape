package frc.robot.subsystems;

import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

import frc.robot.Configs;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//Press Square on CODRIVER CONTROLLER to manually move the bucket backward
//Press Triangle on CODRIVER CONTROLLER to manually move the bucket forward
//Press L1 on CODRIVER CONTROLLER to put the bucket into the load position
//Press R1 on CODRIVER CONTROLLER to put the bucket into the dump position
//Press Share on CODRIVER CONTROLLER to put the bucket into the start position

public class BucketSubsystem extends CSubsystem {

    // Creating a sparkmax to control the motor
    // Snowblower motors must be set as "kBrushed"
    private final SparkMax m_snowblowerMotor = new SparkMax( Constants.BucketSubsystem.kSnowblowerMotor, MotorType.kBrushed );

    // Creating the Encoder
    private DutyCycleEncoder s_snowblowerEncoder = new DutyCycleEncoder( 4 );

    // TODO: Values need to be tuned on Mikey
    private PIDController m_PidController = new PIDController(1, 0, 0);

    private static int positionIndex = 0;

    // Wiether the bucket is being moved manually
    private static boolean moving = false;

    public final BooleanSupplier isBucketBlocking = () -> {
        return ( s_snowblowerEncoder.get() < Constants.BucketSubsystem.kBlockingExenderPosition );
    };

    /**
    * Create an instace of the BucketSubsystem
    */
    public BucketSubsystem() {
        
        SmartDashboard.putBoolean( "Moving forward", false);
        SmartDashboard.putBoolean( "Moving backward", false );
        // Changes the brake type on the motor
        m_snowblowerMotor.configure( 
            Configs.BucketSubsystem.kSnowblowerConfig,
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters 
        );

    }

    /**
     * Does all of the button bindings for the subsystem
     *
     * @param m_driverController The main driver controller
     * @param m_coDriverController The co-driver controller
     */
    public void buttonBindings( PS4Controller m_driverController, PS4Controller m_coDriverController ) {

        // BucketDump ( Right Bumper ) ( Co Driver )
        new JoystickButton(m_coDriverController, Constants.CoDriverControls.bucketDumpPositionButton )
            .whileTrue( BucketDump() );

        // Bucket Load ( Left Bumper ) ( Co Driver )
        new JoystickButton(m_coDriverController, Constants.CoDriverControls.bucketLoadPositionButton )
            .whileTrue( BucketLoad() );

        // Bucket Start ( Share ) ( Co Driver )
        new JoystickButton(m_coDriverController, Constants.CoDriverControls.bucketStartPositionButton )
            .whileTrue( BucketStart() );

        new JoystickButton(m_coDriverController, Constants.CoDriverControls.bucketManualForwardButton )
            .whileTrue( BucketForward() );
        
        new JoystickButton(m_coDriverController, Constants.CoDriverControls.bucketManualBackwardButton )
            .whileTrue( BucketBackward() );
    }

    /**
     * Sets the desired state for the motor to the selected position based on the positionIndex
     */
    public void goToDesiredState() {
       double attempt = m_PidController.calculate( s_snowblowerEncoder.get(), Constants.BucketSubsystem.bucketPositionArray[positionIndex]);
        // Simple limit for PID control
        if ( attempt < Constants.BucketSubsystem.PidMax && attempt > -Constants.BucketSubsystem.PidMax ) {
            m_snowblowerMotor.set( attempt );
        } else if ( attempt < Constants.BucketSubsystem.PidMax ) { 
            m_snowblowerMotor.set( Constants.BucketSubsystem.PidMax );
        } else if ( attempt > -Constants.BucketSubsystem.PidMax ) { 
            m_snowblowerMotor.set( -Constants.BucketSubsystem.PidMax );
        } else { 
            m_snowblowerMotor.stopMotor();
        }
    }
    
    /**
     * peridically calls `goToDesiredState`
     */
    public void periodic ()
    {
        if ( !moving ) {
            // goToDesiredState();
        }

        SmartDashboard.putString( "Bucket Encoder", String.valueOf(s_snowblowerEncoder.get()));
    }

    /**
     * Sets the position index to the bucket to the start position
     * BucketSubsystem
     */
    public CCommand BucketStart() {
        return cCommand_( "BucketSubsystem.BucketStart" )
            .onInitialize( () -> {
                positionIndex = 0;
            })
            .onEnd( () -> {
                m_snowblowerMotor.stopMotor();
            });
    }

    /** 
     * Sets the position index to the bucket to the dump position
     * BucketSubsystem
    */
    public CCommand BucketDump() {
        return cCommand_( "BucketSubsystem.BucketDump" )
            .onInitialize( () -> {
                positionIndex = 1;
            })
            .onEnd( () -> {
                m_snowblowerMotor.stopMotor();
            });
    }

    /**
     * Sets the position index to the load position
     * BucketSubsystem
     */
    public CCommand BucketLoad () {
        return cCommand_( "BucketSubsystem.BucketLoad" )
            .onInitialize( () -> {
                positionIndex = 2;
            })
            .onEnd( () -> {
                m_snowblowerMotor.stopMotor();
            });
    }

    public CCommand BucketForward() {
        return cCommand_( "BucketSubsystem.BucketForward")
            .onInitialize( () -> {
                moving = true;
                SmartDashboard.putBoolean( "Moving forward", true );
            })
            .onExecute( () -> {
                m_snowblowerMotor.set( Constants.BucketSubsystem.SnowblowerSpeed );
            })
            .onEnd( () -> {
                moving = false;
                SmartDashboard.putBoolean( "Moving forward", true );
                m_snowblowerMotor.stopMotor();
            });
    }

    public CCommand BucketBackward() {
        return cCommand_( "BucketSubsystem.BucketBackward")
            .onInitialize( () -> {
                SmartDashboard.putBoolean( "Moving backward", true );
                moving = true;
            })
            .onExecute( () -> {
                m_snowblowerMotor.set( -Constants.BucketSubsystem.SnowblowerSpeed );
            })
            .onEnd( () -> {
                SmartDashboard.putBoolean( "Moving backward", false);
                moving = false;
                m_snowblowerMotor.stopMotor();
            });
    }
}
