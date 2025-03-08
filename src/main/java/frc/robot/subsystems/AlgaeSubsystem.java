package frc.robot.subsystems;

import frc.robot.Configs;
import frc.robot.Constants;

import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//Press L2 on DRIVER CONTROLLER to make the intake spin outwards button
//Press R2 on DRIVER CONTROLLER to make the intake spin inwards


public class AlgaeSubsystem extends CSubsystem {
    
    // Creation of the two sparkmaxes that control the neo 550 motors
    private final SparkMax m_leftAlgaeMotor  = new SparkMax( Constants.AlgaeSubsystem.kLeftAlgaeMotor,  MotorType.kBrushless );
    private final SparkMax m_rightAlgaeMotor = new SparkMax( Constants.AlgaeSubsystem.kRightAlgaeMotor, MotorType.kBrushless );

    /**
     * Creates a new AlgaeSubsystem
     */
    public AlgaeSubsystem() {

       // Configuring the brake type on the left motor
        m_leftAlgaeMotor.configure( 
            Configs.AlgaeSubsystemConfig.kLeftMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters 
        );

        // Configuring the brake type of and inverting the right motor
        m_rightAlgaeMotor.configure( 
            Configs.AlgaeSubsystemConfig.kRightMotorConfig,
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
        // Intake ( Right Trigger )
        new JoystickButton(m_coDriverController, Constants.CoDriverControls.intakeInButton )
            .whileTrue( IntakeIn() );

        // Outtake ( Left Trigger )
        new JoystickButton(m_coDriverController, Constants.CoDriverControls.intakeOutButton )
            .whileTrue( IntakeOut() );
    }

    /**
     *  IntakeIn
     *  Spins the intake motors inwards to gather an algae
     *  AlgaeSubsystem
     */
    public ParallelRaceGroup IntakeIn() {
        return cCommand_("AccessoryMotorSubsystem.IntakeIn")
            .onExecute( () -> {
                m_leftAlgaeMotor.set( -Constants.AlgaeSubsystem.kAlgaeIntakeSpeed );
                m_rightAlgaeMotor.set( -Constants.AlgaeSubsystem.kAlgaeIntakeSpeed );
            })
            .onEnd( () -> {
                m_leftAlgaeMotor.stopMotor();
                m_rightAlgaeMotor.stopMotor();
            })
            .withTimeout(Constants.AlgaeSubsystem.HoldTime);
    }

    /*
     *  IntakeOut
     *  Spins the intake motors outwards to eject an algae
     *  AlgaeSubsystem
     */
    public CCommand IntakeOut() {
        return cCommand_("AccessoryMotorSubsystem.IntakeOut")
            .onExecute( () -> {
                m_leftAlgaeMotor.set( Constants.AlgaeSubsystem.kAlgaeIntakeSpeed );
                m_rightAlgaeMotor.set( Constants.AlgaeSubsystem.kAlgaeIntakeSpeed );
            })
            .onEnd( () -> {
                m_leftAlgaeMotor.stopMotor();
                m_rightAlgaeMotor.stopMotor();
            });
    }

    /**
     *  StopIntake
     *  Stops the intake motors manualy, serving a backup for the auto stop that normally follows the in and out commands
     *  AlgaeSubsystem
     */
    public CCommand StopIntake() {
        return cCommand_("AccessoryMotorSubsystem.StopIntake")
            .onExecute( ()->{
                m_leftAlgaeMotor.stopMotor();
                m_rightAlgaeMotor.stopMotor();
            });
    }
}
