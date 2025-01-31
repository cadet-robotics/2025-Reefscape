package frc.robot.subsystems;
import javax.xml.stream.events.StartDocument;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.lib.custom.CSubsystem;

public class IntakeSubsystem extends CSubsystem {
    public static void main(String[] args) {
        if(PS4Controller.getSquareButtonPressed)
        {
            StartMotor(1);
                    }
                    else if(PS4Controller.getSquareButtonReleased)
                    {
                        StopMotor();
                                            }
                                        }
                                        
                                    
                                        // Make a new motor with a specified port and type
                                        private final SparkMax motor = new SparkMax( Constants.AccessorySubsystem.AccessoryMotorPort, MotorType.kBrushless );
                                    
                                        // Make a new instance of the AccessoryMotorSubsystem and configure the motor
                                        private IntakeSubsystem() {
                                            motor.configure( 
                                                Configs.AccessoryMotorSubsystem.motor_config,
                                                ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters
                                            );
                                        }
                                    
                                        // Return the motor
                                        public SparkMax AccessoryMotor() { return motor; }
                                    
                                        // Start the motor
                                        public static Command StartMotor( double speed ) {
                                return cCommand_("AccessoryMotorSubsystem.StartMotor")
                                    .onExecute( ()->{
                                        motor.set( speed );
                                    }
                                );
                            }
                            public static Command StopMotor() {
        return cCommand_("AccessoryMotorSubsystem.StopMotor")
            .onExecute( ()->{
                motor.stopMotor();
            }
        );
    }

}
