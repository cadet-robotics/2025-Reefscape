package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.lib.custom.CSubsystem;
import frc.robot.lib.custom.CCommand;

public class IntakeSubsystem extends CSubsystem {
    
    // Make a new motor with a specified port and type
    private final SparkMax m_l_motor = new SparkMax( Constants.IntakeSubsystem.kLeftAccessoryMotorPort, MotorType.kBrushless );
    private final SparkMax m_r_motor = new SparkMax( Constants.IntakeSubsystem.kRightAccessoryMotorPort, MotorType.kBrushless );

    // Make a new instance of the AccessoryMotorSubsystem and configure the motor
    public IntakeSubsystem() {
        m_l_motor.configure( 
            Configs.AccessoryMotorSubsystem.kLeftMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters 
        );
        m_r_motor.configure( 
            Configs.AccessoryMotorSubsystem.kRightMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters  
        );
    }

    // Return the motor
    public SparkMax Left() { return m_l_motor; }
    public SparkMax Right() { return m_r_motor; }

    public void buttonBindings( PS4Controller m_driverController ) {
        // Intake
        new JoystickButton(m_driverController, Button.kR1.value )
            .whileTrue( IntakeIn() );

        // Outtake
        new JoystickButton(m_driverController, Button.kL1.value )
            .whileTrue( IntakeOut() );
    }
    // Start the motor
    public CCommand IntakeIn() {
        return cCommand_("AccessoryMotorSubsystem.IntakeIn")
            .onExecute( ()->{
                m_l_motor.set( -Constants.IntakeSubsystem.speed );
                m_r_motor.set( -Constants.IntakeSubsystem.speed );
            }
        )
            .onEnd( () -> {
                m_l_motor.stopMotor();
                m_r_motor.stopMotor();
            }
        );
    }
    public CCommand IntakeOut() {
        return cCommand_("AccessoryMotorSubsystem.IntakeOut")
            .onExecute( ()->{
                m_l_motor.set(  Constants.IntakeSubsystem.speed );
                m_r_motor.set(  Constants.IntakeSubsystem.speed );
            }
        )
            .onEnd( () -> {
                m_l_motor.stopMotor();
                m_r_motor.stopMotor();
            }
        );
    }

    public CCommand StopIntake() {
        return cCommand_("AccessoryMotorSubsystem.StopIntake")
            .onExecute( ()->{
                m_l_motor.stopMotor();
                m_r_motor.stopMotor();
            }
        );
    }

}
