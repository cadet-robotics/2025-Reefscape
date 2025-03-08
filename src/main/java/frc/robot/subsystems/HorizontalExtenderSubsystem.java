/*
 * Tests to run on Mikey
 * - Stops when correspondign limit switch is pressed and ignores other
 */
package frc.robot.subsystems;

import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

import frc.robot.Configs;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//Press Cross on CODRIVER CONTROLLER to retract the "inny outty"/Horizontal Extender
//Press Circle on CODRIVER CONTROLLER to extend the "inny outty"/Horizontal Extender//Press R1 on CODRIVER CONTROLLER to put the bucket into the dump position

public class HorizontalExtenderSubsystem extends CSubsystem {

    // The spark max for the snowblower motor
    private final SparkMax m_horizontalExtenderMotor = new SparkMax( Constants.HorzontalExtenderSubsystem.kSnowblowerMotor, MotorType.kBrushed );
    //limit switch values are reversed    
    private final DigitalInput s_frontLimitSwitch = new DigitalInput( Constants.HorzontalExtenderSubsystem.kFrontLimitSwitch );
    private final DigitalInput s_backLimitSwitch = new DigitalInput( Constants.HorzontalExtenderSubsystem.kBackLimitSwitch );

    /**
     * Checks if the robot should be in slow mode based on the horizontal extender
     */
    public final BooleanSupplier extenderSlowCheck = () -> {
        //limit switch values are reversed
        return s_frontLimitSwitch.get(); // Will only enter slow mode if the front limit switch is pressed
        // return !s_backLimitSwitch.get(); // Will only enter slow mode if the back limit switch is not pressed
    };

    public static BooleanSupplier isBucketBlocking = () -> { return true; };

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


        // Retract ( Cross )
        new JoystickButton(m_coDriverController, Constants.CoDriverControls.horizontalRetractButton )
            //limit switch values are reversed
            .and( () -> backLimitSwitchPressing() )
                .whileTrue( Retract() );
    }

    /**
     * Gets the state of the front limit switch
     *
     * @return boolean limit switch state
     */
    public Boolean frontLimitSwitchPressing() {
        //limit switch values are reversed
        return s_frontLimitSwitch.get();
    }

    /**
     * Gets the state of the back limit switch
     *
     * @return boolean limit switch state
     */
    public Boolean backLimitSwitchPressing() {
        //limit switch values are reversed
        return s_backLimitSwitch.get();
    }

    public void setIsBucketBlocking(BooleanSupplier isBucketBlocking) {
        HorizontalExtenderSubsystem.isBucketBlocking = isBucketBlocking;
    }

    @Override
    public void periodic() {
        //limit switch values are reversed
        SmartDashboard.putBoolean("allIn", backLimitSwitchPressing());
        SmartDashboard.putBoolean("allOut", frontLimitSwitchPressing());
        SmartDashboard.putBoolean("ExtenderSlow", extenderSlowCheck.getAsBoolean());
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
                    //limit switch values are reversed
                    while( ( frontLimitSwitchPressing())) {
                        m_horizontalExtenderMotor.set( Constants.HorzontalExtenderSubsystem.kExtendSpeed);
                    }
                }
            })  
            // Code for press extension
            .onExecute( () -> {
                if ( Constants.HorzontalExtenderSubsystem.extederMode.equals( "Hold" )) {
                    //limit switch values are reversed
                    if ( !frontLimitSwitchPressing() ) {
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
            })
            .isFinished( () -> { return !frontLimitSwitchPressing(); });
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
                    //limit switch values are reversed
                    while( ( backLimitSwitchPressing())) {
                        m_horizontalExtenderMotor.set( -Constants.HorzontalExtenderSubsystem.kExtendSpeed);
                    }
                }
            })  
            // Code for press retraction
            .onExecute( () -> {
                if ( Constants.HorzontalExtenderSubsystem.extederMode.equals( "Hold" )) {
                    //limit switch values are reversed
                    if ( !backLimitSwitchPressing() ) {
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
