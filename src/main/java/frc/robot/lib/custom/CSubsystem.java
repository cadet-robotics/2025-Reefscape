// The following code is from the following link but may be modified in some places.
// https://github.com/Greater-Rochester-Robotics/GRRBase/blob/main/src/main/java/org/team340/lib/util/command/GRRSubsystem.java

package frc.robot.lib.custom;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CSubsystem implements Subsystem {
    
    /** Register the subsystem on creation */
    public CSubsystem() {
        register();
    }

    /**
     * Function that is called when the robot is disabled. 
     * This should be used to ensure that enabling the robot does not make it unsafe.
     */
    public void onDisableInit() {}

    /** Creates a nameless command */
    public CCommand cCommand() {
        return new CCommand( this );
    }

    /** Creating a new command with a name */
    public CCommand cCommand_( String name ) {
        return new CCommand( name, this );
    }
}