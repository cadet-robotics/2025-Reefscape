package frc.robot.subsystems;

import frc.robot.lib.custom.CSubsystem;
import frc.robot.Constants;
import frc.robot.lib.custom.CCommand;

import frc.robot.Constants;
import frc.robot.Configs;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class BucketSubsytem extends CSubsystem {

    private final SparkMax m_snowblower = new SparkMax( Constants.BucketSubsytem.kSnowblower, MotorType.kBrushed );

    public BucketSubsytem() {
        m_snowblower.configure( 
            Configs.BucketSubsystem.kSnowblowerConfig,
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters 
        );
    }

    public CCommand Testing1 () {
        return cCommand_( "BucketSubsystem.Testing1" )
            .onExecute( () -> {
                    m_snowblower.set( Constants.BucketSubsytem.SnowblowerSpeed );
                }
            )
            .onEnd( () -> {
                    m_snowblower.stopMotor();
                }
            );
    }

    public CCommand Testing2 () {
        return cCommand_( "BucketSubsystem.Testing2" )
            .onExecute( () -> {
                    m_snowblower.set( Constants.BucketSubsytem.SnowblowerSpeed );
                }
            )
            .onEnd( () -> {
                    m_snowblower.stopMotor();
                }
            );
    }
}
