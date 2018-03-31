package org.usfirst.frc.team4468.robot.Commands.Manipulators;

import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Subsystems.RotatingLift;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class BreakClamp extends Command {
    private RotatingLift rl = Robot.rotatingLift;
    
    private Value clamp;
    
    public BreakClamp(Value v) {
        requires(rl);
        clamp = v;
    }
    
    /* Called repeatedly when this Command is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
        rl.clamp(clamp);
    }

    /* Make this return true when this Command no longer needs to run execute()
     * @return the command stops when true
     */
    protected boolean isFinished() {
        return false;//clamp == rl.getState();
    }
    
    protected void end() {
        rl.clamp(Value.kForward);
    }
    
    protected void interupted() {
        rl.clamp(Value.kForward);
    }
}
