// https://github.com/Greater-Rochester-Robotics/GRRBase/blob/main/src/main/java/org/team340/lib/util/command/CommandBuilder.java
package frc.robot.lib.custom;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ConcurrentModificationException;
import java.util.function.BooleanSupplier;

/**
 * A command builder. Very similar to {@link FunctionalCommand}.
 * Stylistic alternative to using decorators.
 */
public class CCommand extends Command {

    private Runnable onInitialize = () -> {};
    private Runnable onExecute = () -> {};
    private BooleanConsumer onEnd = interrupted -> {};
    private BooleanSupplier isFinished = () -> false;

    /**
     * Create the command builder.
     * @param requirements The subsystems required by the command.
     * @param name The command's name.
     */
    public CCommand(String name, Subsystem... requirements) {
        this(requirements);
        setName(name);
    }

    /**
     * Create the command builder.
     * @param requirements The subsystems required by the command.
     */
    public CCommand(Subsystem... requirements) {
        addRequirements(requirements);
    }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    public CCommand onInitialize(Runnable onInitialize) {
        if (this.isScheduled()) throw new ConcurrentModificationException(
            "Cannot change methods of a command while it is scheduled"
        );
        this.onInitialize = onInitialize;
        return this;
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    public CCommand onExecute(Runnable onExecute) {
        if (this.isScheduled()) throw new ConcurrentModificationException(
            "Cannot change methods of a command while it is scheduled"
        );
        this.onExecute = onExecute;
        return this;
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally,
     * or when it interrupted/canceled.
     */
    public CCommand onEnd(Runnable onEnd) {
        return onEnd(interrupted -> onEnd.run());
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally,
     * or when it interrupted/canceled. Supplied boolean is if the command was interrupted.
     */
    public CCommand onEnd(BooleanConsumer onEnd) {
        if (this.isScheduled()) throw new ConcurrentModificationException(
            "Cannot change methods of a command while it is scheduled"
        );
        this.onEnd = onEnd;
        return this;
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it. By default, this returns {@code false}. If {@code true}, the command
     * is effectively an {@link InstantCommand}. If {@code false}, the command will run continuously until
     * it is canceled.
     */
    public CCommand isFinished(boolean isFinished) {
        return isFinished(() -> isFinished);
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it. By default, this returns {@code false}.
     */
    public CCommand isFinished(BooleanSupplier isFinished) {
        if (this.isScheduled()) throw new ConcurrentModificationException(
            "Cannot change methods of a command while it is scheduled"
        );
        this.isFinished = isFinished;
        return this;
    }

    @Override
    public void initialize() {
        onInitialize.run();
    }

    @Override
    public void execute() {
        onExecute.run();
    }

    @Override
    public void end(boolean interrupted) {
        onEnd.accept(interrupted);
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }
}
