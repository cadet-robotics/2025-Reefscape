package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.lib.custom.CSubsystem;
import com.revrobotics.spark.config.SparkBaseConfig;
public class AccessoryMotorSubsystem extends CSubsystem {
    
    private final SparkMax motor = new SparkMax( DriveConstants.AccessoryMotor1, MotorType.kBrushless );

    private AccessoryMotorSubsystem() {}

    public SparkMax AccessoryMotor() {
        return motor;
    }

    public Command StartMotor() {
        return cCommand("AccessoryMotorSubsystem")
            AccessoryMotor()
        });
    }

}
