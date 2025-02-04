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
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class AlgaeSubsystem extends CSubsystem {
    
    // Make a new motor with a specified port and type
    private final SparkMax m_leftAlgaeMotor = new SparkMax( Constants.AlgaeSubsystem.kLeftAlgaeMotor, MotorType.kBrushless );
    private final SparkMax m_rightAlgaeMotor = new SparkMax( Constants.AlgaeSubsystem.kRightAlgaeMotor, MotorType.kBrushless );

    // Make a new instance of the AccessoryMotorSubsystem and configure the motor
    public AlgaeSubsystem() {

       // Changing the brake type on the left motor
        m_leftAlgaeMotor.configure( 
            Configs.AccessoryMotorSubsystem.kLeftMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters 
        );
        // Changing the brake type and inverted on the right motor
        m_rightAlgaeMotor.configure( 
            Configs.AccessoryMotorSubsystem.kRightMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters  
        );
    }

    // Allows access of the motor objects
    public SparkMax Left() { return m_leftAlgaeMotor; }
    public SparkMax Right() { return m_rightAlgaeMotor; }

    public void buttonBindings( PS4Controller m_driverController ) {
        // Intake
        new JoystickButton(m_driverController, Button.kR2.value )
            .whileTrue( IntakeIn() );

        // Outtake
        new JoystickButton(m_driverController, Button.kL2.value )
            .whileTrue( IntakeOut() );
    }

    // Start the motor
    public CCommand IntakeIn() {
        return cCommand_("AccessoryMotorSubsystem.IntakeIn")
            .onExecute( ()->{
                m_leftAlgaeMotor.set( -Constants.AlgaeSubsystem.speed );
                m_rightAlgaeMotor.set( -Constants.AlgaeSubsystem.speed );
            })
            .onEnd( () -> {
                m_leftAlgaeMotor.stopMotor();
                m_rightAlgaeMotor.stopMotor();
            });
    }
    // Spins the motors outwards until the command stops running
    public CCommand IntakeOut() {
        return cCommand_("AccessoryMotorSubsystem.IntakeOut")
            .onExecute( ()->{
                m_leftAlgaeMotor.set( Constants.AlgaeSubsystem.speed );
                m_rightAlgaeMotor.set( Constants.AlgaeSubsystem.speed );
            })
            .onEnd( () -> {
                m_leftAlgaeMotor.stopMotor();
                m_rightAlgaeMotor.stopMotor();
            });
    }

    // Spins the motors inwards until the command stops running
    public CCommand StopIntake() {
        return cCommand_("AccessoryMotorSubsystem.StopIntake")
            .onExecute( ()->{
                m_leftAlgaeMotor.stopMotor();
                m_rightAlgaeMotor.stopMotor();
            });
    }
}
