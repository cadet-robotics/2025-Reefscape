package frc.robot.subsystems;

import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

import frc.robot.Configs;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class BucketSubsystem extends CSubsystem {

    // Creating a sparkmax to control the motor
    // Snowblower motors must be set as "kBrushed"
    private final SparkMax m_snowblowerMotor = new SparkMax( Constants.BucketSubsystem.kSnowblowerMotor, MotorType.kBrushed );

    // Creating the Encoder
    private DutyCycleEncoder s_snowblowerEncoder = new DutyCycleEncoder( 4 );

    // TODO: Values need to be tuned on Mikey
    private PIDController m_PidController = new PIDController(1, 0, 0);

    private static int positionIndex = 0;

    /**
    * Create an instace of the BucketSubsystem
    */
    public BucketSubsystem() {
        
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

        new JoystickButton(m_driverController, Button.kR1.value )
            .whileTrue( BucketDump() );

        new JoystickButton(m_driverController, Button.kL1.value )
            .whileTrue( BucketLoad() );

        new JoystickButton(m_driverController, Button.kShare.value )
            .whileTrue( BucketStart() );
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
        return cCommand_( "BucketSubsystem.BucketStart" )
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

    /**
     * Sets the desired state for the motor to the selected position based on the positionIndex
     */
    public void goToDesiredState() {
        m_snowblowerMotor.set( m_PidController.calculate( s_snowblowerEncoder.get(), Constants.BucketSubsystem.bucketPositionArray[positionIndex]));
    }
    
    /**
     * peridically calls `goToDesiredState`
     */
    public void periodic ()
    {
        goToDesiredState();
    }
}