package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.lib.custom.CSubsystem;

public class AccessoryMotorSubsystem extends CSubsystem {
    
    // Make a new motor with a specified port and type
    private final SparkMax motor = new SparkMax( Constants.AccessorySubsystem.AccessoryMotorPort, MotorType.kBrushless );

    // Make a new instance of the AccessoryMotorSubsystem and configure the motor
    private AccessoryMotorSubsystem() {
        motor.configure( 
            Configs.AccessoryMotorSubsystem.motor_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters  
        );
    }

    // Return the motor
    public SparkMax AccessoryMotor() { return motor; }

    // Start the motor
    public Command StartMotor( double speed ) {
        return cCommand_("AccessoryMotorSubsystem.StartMotor")
            .onExecute( ()->{
                motor.set( speed );
            }
        );
    }
    public Command StopMotor() {
        return cCommand_("AccessoryMotorSubsystem.StopMotor")
            .onExecute( ()->{
                motor.stopMotor();
            }
        );
    }

}
