package frc.robot.subsystems;

import frc.robot.lib.custom.CSubsystem;
import frc.robot.lib.custom.CCommand;

import frc.robot.Constants;
import frc.robot.Configs;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
            Configs.HorzontalExtenderSubsystem.kSnowblowerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void createButtons( PS4Controller m_driverController) {

        // Extend
        new JoystickButton(m_driverController, Button.kCircle.value )
            .and( () -> !frontLimitSwitchPressing()  )
            .whileTrue(
                Extend()
            );

        // Retract
        new JoystickButton(m_driverController, Button.kCross.value )
            .and( () -> !backLimitSwitchPressing() )
            .whileTrue(
                Retract()
            );
    }

    public SparkMax getHorizontalExtenderMotor() {
        return m_horizontalExtenderMotor;
    }

    public Boolean frontLimitSwitchPressing() {
        return m_frontLimitSwitch.get();
    }

    public Boolean backLimitSwitchPressing() {
        return m_backLimitSwitch.get();
    }

    public CCommand Extend() {
        return cCommand_( "HorzontalExtenderSubsystem.Extend" )
            .onExecute( () -> {
                m_horizontalExtenderMotor.set( Constants.HorzontalExtenderSubsystem.kExtendSpeed );
                if ( frontLimitSwitchPressing() ) {
                    m_horizontalExtenderMotor.stopMotor();
                }
            })
            .onEnd( () -> {
                m_horizontalExtenderMotor.stopMotor();
            });
    }

    public CCommand Retract() {
        return cCommand_( "HorzontalExtenderSubsystem.Retract" )
            .onExecute( () -> {
                m_horizontalExtenderMotor.set( -Constants.HorzontalExtenderSubsystem.kExtendSpeed );
                if ( backLimitSwitchPressing() ) {
                    m_horizontalExtenderMotor.stopMotor();
                }
            })
            .onEnd( () -> {
                m_horizontalExtenderMotor.stopMotor();
            });
    }
}
