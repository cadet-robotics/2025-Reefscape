// https://github.com/Greater-Rochester-Robotics/GRRBase/blob/main/src/main/java/org/team340/lib/util/command/GRRSubsystem.java
package frc.robot.lib.custom;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CSubsystem implements Subsystem {
    
    // Register the subsystem on creation
    public CSubsystem() {
        register();
    }

    // Creating a new command
    public CCommand cCommand() {
        return new CCommand( this );
    }

    // Creating a new command with a name
    public static CCommand cCommand_( String name ) {
        return new CCommand( name, this );
    }
}