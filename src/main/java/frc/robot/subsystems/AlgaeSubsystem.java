package frc.robot.subsystems;

import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.command.button.JoystickButton;

public class AlgaeSubsystem extends CSubsystem {
    
    // Make a new motor with a specified port and type
    private final CANSparkMax m_leftAlgaeMotor = new CANSparkMax( Constants.AlgaeSubsystem.kLeftAlgaeMotor, MotorType.kBrushless );
    private final CANSparkMax m_rightAlgaeMotor = new SparkMax( Constants.AlgaeSubsystem.kRightAlgaeMotor, MotorType.kBrushless );

    // Make a new instance of the AccessoryMotorSubsystem and configure the motor
    public AlgaeSubsystem() {

       // Changing the brake type on the left motor
        m_leftAlgaeMotor.configure( 
            );
        // Changing the brake type and inverted on the right motor
        m_rightAlgaeMotor.configure( 
            PersistMode.kPersistParameters  
        );
    }

    // Allows access of the motor objects
    public SparkMax Left()  m_leftAlgaeMotor; }
    public SparkMax Right() {  m_rightAlgaeMotor; }

    public void buttonBindings( PS4Controller m_driverController ) {
        // Intake
        new Joystick(m_driverController, Button.kR1.value )
            .whileTrue( IntakeIn() );
        // Outtake
        new JoystickButton(m_driverController, Button.kL1.value )
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
                m_leftAlgaeMotor.stopMotor()
                m_rightAlgaeMotor.stopMotor()
            });
    }
}
