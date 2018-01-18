package org.usfirst.frc.team4468.robot.commands.Manipulators;

import org.usfirst.frc.team4468.robot.Constants;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Util.PID;
import org.usfirst.frc.team4468.robot.subsystems.RotatingLift;

import edu.wpi.first.wpilibj.command.Command;

/**
 * This uses the Pot attached to the rotating arm to move
 * to a specific angle.
 */
public class RotateAngle extends Command {
    
    private double theta;
    
    private RotatingLift rl = Robot.rotatingLift;
    private PID pid;
    
    public RotateAngle(double t) {
        requires(rl);
        theta = t;
        
        pid = new PID(Constants.lifterP, Constants.lifterI, Constants.lifterD);
        pid.setInputRange(0, 180);
        pid.setOutputRange(-1, 1);
        pid.setPoint(theta);
    }
    
    /* Called repeatedly when this Command is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
        rl.rotate(pid.calculate(rl.getAngle()));
    }

    /* Make this return true when this Command no longer needs to run execute()
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#isFinished()
     * @return the command stops when true
     */
    protected boolean isFinished() {
        return pid.onTarget(rl.getAngle());
    }

    /* Called once after isFinished returns true
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#end()
     */
    protected void end() {
        rl.stop();
    }

    /* Called when another command which requires one or more of the same
     * subsystems is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#interrupted()
     */
    protected void interrupted() {
        rl.stop();
    }
}
