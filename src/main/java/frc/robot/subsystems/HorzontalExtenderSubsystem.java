package frc.robot.subsystems;

import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

import frc.robot.Constants;
import frc.robot.Configs;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class HorzontalExtenderSubsystem extends CSubsystem {

    private final SparkMax m_horizontalExtenderMotor = new SparkMax( Constants.HorzontalExtenderSubsystem.kSnowblowerMotor, MotorType.kBrushed );    
    private final DigitalInput m_frontLimitSwitch = new DigitalInput( Constants.HorzontalExtenderSubsystem.kFrontLimitSwitch );
    private final DigitalInput m_backLimitSwitch = new DigitalInput( Constants.HorzontalExtenderSubsystem.kBackLimitSwitch );

    public HorzontalExtenderSubsystem() {
        m_horizontalExtenderMotor.configure( 
            Configs.HorizontalExtenderSubsystem.kSnowblowerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    // Creates the button bindings for the subsystem
    public void buttonBindings( PS4Controller m_driverController) {

        // Extend ( Circle ) 
        new JoystickButton(m_driverController, Button.kCircle.value )
            .and( () -> !frontLimitSwitchPressing()  )
            .whileTrue(
                Extend()
            );

        // Retract ( Cross )
        new JoystickButton(m_driverController, Button.kCross.value )
            .and( () -> !backLimitSwitchPressing() )
            .whileTrue(
                Retract()
            );
    }

    // Access the motor
    public SparkMax getHorizontalExtenderMotor() {
        return m_horizontalExtenderMotor;
    }

    // Funcions to access the state of the limit state
    public Boolean frontLimitSwitchPressing() {
        return m_frontLimitSwitch.get();
    }

    public Boolean backLimitSwitchPressing() {
        return m_backLimitSwitch.get();
    }

    public CCommand Extend() {
        return cCommand_( "HorzontalExtenderSubsystem.Extend" )
            .onExecute( () -> {
               if ( frontLimitSwitchPressing() ) {
                  m_horizontalExtenderMotor.stopMotor();
                  return;
               }
                m_horizontalExtenderMotor.set( Constants.HorzontalExtenderSubsystem.kExtendSpeed );
            })
            .onEnd( () -> {
                m_horizontalExtenderMotor.stopMotor();
            });
    }

    public CCommand Retract() {
        return cCommand_( "HorzontalExtenderSubsystem.Retract" )
            .onExecute( () -> {
               if ( backLimitSwitchPressing() ) {
                  m_horizontalExtenderMotor.stopMotor();
                  return;
               }
                m_horizontalExtenderMotor.set( -Constants.HorzontalExtenderSubsystem.kExtendSpeed );
            })
            .onEnd( () -> {
                m_horizontalExtenderMotor.stopMotor();
            });
    }
}
