/*
 * Tests to run on Mikey
 * - Stops when correspondign limit switch is pressed and ignores other
 */
package frc.robot.subsystems;

import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

import frc.robot.Configs;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class HorizontalExtenderSubsystem extends CSubsystem {

    // The spark max for the snowblower motor
    private final SparkMax m_horizontalExtenderMotor = new SparkMax( Constants.HorzontalExtenderSubsystem.kSnowblowerMotor, MotorType.kBrushed );    
    private final DigitalInput s_frontLimitSwitch = new DigitalInput( Constants.HorzontalExtenderSubsystem.kFrontLimitSwitch );
    private final DigitalInput s_backLimitSwitch = new DigitalInput( Constants.HorzontalExtenderSubsystem.kBackLimitSwitch );

    /**
     * HorizontalSubsytemSetup
     */
    public HorizontalExtenderSubsystem() {
        // Configures the snowblower motor
        m_horizontalExtenderMotor.configure( 
            Configs.HorizontalExtenderSubsystem.kSnowblowerConfig,
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

    /**
     * Gets the state of the front limit switch
     *
     * @return boolean limit switch state
     */
    public Boolean frontLimitSwitchPressing() {
        return s_frontLimitSwitch.get();
    }

    /**
     * Gets the state of the back limit switch
     *
     * @return boolean limit switch state
     */
    public Boolean backLimitSwitchPressing() {
        return s_backLimitSwitch.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("allIn", frontLimitSwitchPressing());
        SmartDashboard.putBoolean("allOut", backLimitSwitchPressing());
    }

    /**
     * Horizontal Extender Subsystem
     * Extends the extender until limit switch hit
     * Snowblower motor
     */
    public CCommand Extend() {
        return cCommand_( "HorzontalExtenderSubsystem.Extend" )
            // Code for press extension
            .onInitialize( () -> {
                SmartDashboard.putBoolean( "moveOut", true );
                if ( Constants.HorzontalExtenderSubsystem.extederMode.equals( "Press" )) {
                    while( !( frontLimitSwitchPressing())) {
                        m_horizontalExtenderMotor.set( Constants.HorzontalExtenderSubsystem.kExtendSpeed);
                    }
                }
            })  
            // Code for press extension
            .onExecute( () -> {
                if ( Constants.HorzontalExtenderSubsystem.extederMode.equals( "Hold" )) {
                    if ( frontLimitSwitchPressing() ) {
                        m_horizontalExtenderMotor.stopMotor();
                        return;
                    }
                    m_horizontalExtenderMotor.set( Constants.HorzontalExtenderSubsystem.kExtendSpeed );
                }
            })
            // Regardless of which mode we use, stopping the motor is required
            .onEnd( () -> {
                SmartDashboard.putBoolean( "moveOut", false );
                m_horizontalExtenderMotor.stopMotor();
            });
    }


    /**
     * Horizontal Extender Subsystem
     * Retracts the extender until limit switch hit
     * Snowblower motor
     */
    public CCommand Retract() {
        return cCommand_( "HorzontalExtenderSubsystem.Retract" )
            // Code for press retraction
            .onInitialize( () -> {
                SmartDashboard.putBoolean( "moveIn", true );
                if ( Constants.HorzontalExtenderSubsystem.extederMode.equals( "Press" )) {
                    while( !( backLimitSwitchPressing())) {
                        m_horizontalExtenderMotor.set( -Constants.HorzontalExtenderSubsystem.kExtendSpeed);
                    }
                }
            })  
            // Code for press retraction
            .onExecute( () -> {
                if ( Constants.HorzontalExtenderSubsystem.extederMode.equals( "Hold" )) {
                    if ( backLimitSwitchPressing() ) {
                        m_horizontalExtenderMotor.stopMotor();
                        return;
                    }
                    m_horizontalExtenderMotor.set( -Constants.HorzontalExtenderSubsystem.kExtendSpeed );
                }
            })
            // Regardless of which mode we use, stopping the motor is required
            .onEnd( () -> {
                SmartDashboard.putBoolean( "moveIn", false );
                m_horizontalExtenderMotor.stopMotor();
            });
    }
}
