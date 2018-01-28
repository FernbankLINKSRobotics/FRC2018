package org.usfirst.frc.team4468.robot.commands.Manipulators;

import org.usfirst.frc.team4468.robot.Constants;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeClamp extends Command {
    private Intake in = Robot.intake;
    
    private Value clamp;
    
    public IntakeClamp(Value v) {
        requires(in);
        clamp = v;
    }
    
    /* Called repeatedly when this Command is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
        in.clamp(clamp);
    }

    /* Make this return true when this Command no longer needs to run execute()
     * @return the command stops when true
     */
    protected boolean isFinished() {
        return clamp == in.getState();
    }
}
