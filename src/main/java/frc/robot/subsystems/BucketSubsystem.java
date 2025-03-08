package frc.robot.subsystems;

import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

import frc.robot.Configs;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS4Controller;
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
    // private DutyCycleEncoder s_snowblowerEncoder = new DutyCycleEncoder( 4 );
    private SparkAbsoluteEncoder s_snowblowerEncoder;

    // TODO: Values need to be tuned on Mikey
    private PIDController m_PidController = new PIDController(1, 0, 0);

    private static int positionIndex = 0;

    // Wiether the bucket is being moved manually
    private static boolean isManual = false;

    public  BooleanSupplier isBucketBlocking = () -> {
        return ( s_snowblowerEncoder.getPosition() < Constants.BucketSubsystem.kBlockingExenderPosition );
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
        s_snowblowerEncoder = m_snowblowerMotor.getAbsoluteEncoder();
        // s_snowblowerEncoder.setAssumedFrequency(975.6);
        // s_snowblowerEncoder.setDutyCycleRange(1, 1024);
        // s_snowblowerEncoder.
    }

    public void OnDisable() {
        isManual = true;
    }

    /**
     * Does all of the button bindings for the subsystem
     *
     * @param m_driverController The main driver controller
     * @param m_coDriverController The co-driver controller
     */
    public void buttonBindings( PS4Controller m_driverController, PS4Controller m_coDriverController ) {

        // BucketDump ( Right Bumper ) ( Co Driver )
        new JoystickButton( m_driverController, Constants.DriverControls.bucketDumpPositionButton )
            .whileTrue( BucketDump() );

        // Bucket Load ( Left Bumper ) ( Co Driver )
        new JoystickButton(m_driverController, Constants.DriverControls.bucketLoadPositionButton )
            .whileTrue( BucketLoad() );

        // Bucket Start ( Share ) ( Co Driver )
        new JoystickButton(m_driverController, Constants.DriverControls.bucketStartPositionButton )
            .whileTrue( BucketStart() );

        new JoystickButton(m_driverController, Constants.DriverControls.bucketManualForwardButton )
            .whileTrue( BucketForward() );
        
        new JoystickButton(m_driverController, Constants.DriverControls.bucketManualBackwardButton )
            .whileTrue( BucketBackward() );

        new JoystickButton(m_driverController, Constants.DriverControls.moveToTopReef )
            .whileTrue( BucketTopDump() );
    }

    /**
     * Sets the desired state for the motor to the selected position based on the positionIndex
     */
    public void goToDesiredState() {
        double attempt = m_PidController.calculate( Math.abs( 1 - s_snowblowerEncoder.getPosition() ) , 1 - Constants.BucketSubsystem.bucketPositionArray[positionIndex]);
        SmartDashboard.putNumber( "MoveTargetState", attempt );
        // Simple limit for PID control
        // if ( attempt < Constants.BucketSubsystem.PidMax && attempt > -Constants.BucketSubsystem.PidMax ) {
            m_snowblowerMotor.set( attempt * 3.0 );
        // } else if ( attempt < Constants.BucketSubsystem.PidMax ) { 
        //     m_snowblowerMotor.set( Constants.BucketSubsystem.PidMax );
        // } else if ( attempt > -Constants.BucketSubsystem.PidMax ) { 
        //     m_snowblowerMotor.set( -Constants.BucketSubsystem.PidMax );
        // } else { 
        //     m_snowblowerMotor.stopMotor();
        // }
    }
    
    /**
     * peridically calls `goToDesiredState`
     */
    public void periodic ()
    {
        if ( !isManual ) {
            goToDesiredState();
        }

        SmartDashboard.putNumber( "Bucket Encoder", s_snowblowerEncoder.getPosition());
    }

    /**
     * Sets the position index to the bucket to the start position
     * BucketSubsystem
     */
    public CCommand BucketStart() {
        return cCommand_( "BucketSubsystem.BucketStart" )
            .onInitialize( () -> {
                isManual = false;
                positionIndex = 0;
            });
    }

    /** 
     * Sets the position index to the bucket to the dump position
     * BucketSubsystem
    */
    public CCommand BucketDump() {
        return cCommand_( "BucketSubsystem.BucketDump" )
            .onInitialize( () -> {
                isManual = false;
                positionIndex = 1;
            })
            .onEnd( () -> {
                positionIndex = 2;
            });
    }

    public CCommand BucketTopDump() {
        return cCommand_( "BucketSubsystem.BucketTopDump" )
            .onInitialize( () -> {
                isManual = false;
                positionIndex = 3;
            })
            .onEnd( () -> {
                positionIndex = 2;
            });
    }

    /**
     * Sets the position index to the load position
     * BucketSubsystem
     */
    public CCommand BucketLoad () {
        return cCommand_( "BucketSubsystem.BucketLoad" )
            .onInitialize( () -> {
                isManual = false;
                positionIndex = 2;
            });
    }

    public CCommand BucketForward() {
        return cCommand_( "BucketSubsystem.BucketForward")
            .onInitialize( () -> {
                isManual = true;
                SmartDashboard.putBoolean( "Moving forward", true );
            })
            .onExecute( () -> {
                m_snowblowerMotor.set( Constants.BucketSubsystem.SnowblowerForwardSpeed );
            })
            .onEnd( () -> {
                SmartDashboard.putBoolean( "Moving forward", true );
                m_snowblowerMotor.stopMotor();
            });
    }

    public CCommand BucketBackward() {
        return cCommand_( "BucketSubsystem.BucketBackward")
            .onInitialize( () -> {
                SmartDashboard.putBoolean( "Moving backward", true );
                isManual = true;
            })
            .onExecute( () -> {
                m_snowblowerMotor.set( -Constants.BucketSubsystem.SnowblowerBackwardSpeed );
            })
            .onEnd( () -> {
                SmartDashboard.putBoolean( "Moving backward", false);
                m_snowblowerMotor.stopMotor();
            });
    }
}
