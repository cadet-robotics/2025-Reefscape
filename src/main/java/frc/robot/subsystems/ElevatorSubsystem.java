package frc.robot.subsystems;

import javax.naming.ldap.ControlFactory;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.lib.custom.CCommand;
import frc.robot.lib.custom.CSubsystem;

public class ElevatorSubsystem extends CSubsystem {

   // Motor Setup
    private static final SparkFlex m_elevatorMotor = new SparkFlex( 
        Constants.ElevatorSubsystem.kElevatorMotor, 
        MotorType.kBrushless 
    );

    private static SparkClosedLoopController m_elevatorController;

    // Encoder Setup
    private static Encoder s_elevatorEncoder = new Encoder( 2, 3 );
        
    // Servo Setup
    // This servo is the brake for the elevator
    private static final Servo m_elevatorBrake = new Servo( 
        Constants.ElevatorSubsystem.kElevatorBrake 
    );

    // Top Limit Switch Setup
    private static final DigitalInput m_topLimitSwitch = new DigitalInput( 
          Constants.ElevatorSubsystem.kTopLimitSwitch
    );

    // Bottom Limit Switch Setup
    private static final DigitalInput m_bottomLimitSwitch = new DigitalInput( 
          Constants.ElevatorSubsystem.kBottomLimitSwitch
    );

    // The number corresponding to the level of the elevator
    // This should be a value between 0 and 8 ( There are 9 levels but a 0 based system will be used )
    private static int level = 0;

    // Creates the Elevator Subsystem
    public ElevatorSubsystem() {
        // Configuring the brake mode on the elevator motor
        m_elevatorMotor.configure( 
            Configs.ElevatorSubsystem.kElevatorMotorConfig,
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters 
        );
        m_elevatorController = m_elevatorMotor.getClosedLoopController();
    }

    public void buttonBindings( PS4Controller m_driverController, PS4Controller m_coDriverController ) {

        // Should be dpad up with a 10 degree margin for error on either side
        new JoystickButton( m_driverController, m_driverController.getPOV() )
        .and( () -> Math.abs( m_driverController.getPOV() - 0 ) < 10)
        .whileTrue( 
           ElevatorLevelUp()
        );
        // Should be dpad down with a 10 degree margin for error on either side
         new JoystickButton( m_driverController, m_driverController.getPOV() )
             .and( () -> Math.abs( m_driverController.getPOV() - 180 ) < 10)
            .whileTrue( 
                ElevatorLevelDown()
            );
    }

    // The following 5 functions are just in case the RobotContainer needs to access any of these; most likely for testing.
    public SparkFlex getElevatorMotor() {
        return m_elevatorMotor;
    }
    public Encoder getElevatorEncoder() {
        return s_elevatorEncoder;
    }
    public Servo getElevatorBrake() {
        return m_elevatorBrake;
    }
    public DigitalInput getTopLimitSwitch() {
       return m_topLimitSwitch;
    }
    public DigitalInput getBottomLimitSwitch() {
       return m_bottomLimitSwitch;
    }
    
    // // Basic global reset button for the encoder. This should only be used in testing and at the time of startup
    // public void resetEncoder() {
    //     m_elevatorEncoder.;
    // }

    // This function should bring the elevator to the currently seleted level and is called after a level change
    public void setDesiredState( double desiredState ) {
        double position = s_elevatorEncoder.getDistance();
        m_elevatorController.setReference(
            desiredState,
            ControlType.kPosition
        );
    }

    @Override
    public void periodic() {

        SmartDashboard.putString( "ElevatorLevel", Constants.ElevatorSubsystem.LevelNames[level] );
        SmartDashboard.putNumber( "Encoder", s_elevatorEncoder.getDistance() * 360.0 );

    }
     
    // Increase the elevator level by one if it's not already at the max level ( 8 )
    public CCommand ElevatorLevelUp() {
        return cCommand_( "ElevatorSubsystem.ElevatorLevelUp" )
            .onInitialize( () -> {
                if ( level < 8 ) {
                    level++;
                }
            });
    }
    // Decrease the elevator level by one if it's not already at the bottom level
    public CCommand ElevatorLevelDown() {
        return cCommand_( "ElevatorSubsystem.ElevatorLevelDown" )
            .onInitialize( () -> {
                if ( level > 0 ) {
                    level--;
                }
            });
    }

}
