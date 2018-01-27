package org.usfirst.frc.team4468.robot.commands.Manipulators;

import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.subsystems.RotatingLift;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */


public class Rotate extends Command {

	private double speed = 0;
	private RotatingLift rl = Robot.rotatingLift;
	
    public Rotate(double s) {
        requires(Robot.rotatingLift);
        speed = s;
    }
    
    /* Called repeatedly when this Command is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
        rl.rotate(speed);
    }

    /* Make this return true when this Command no longer needs to run execute()
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#isFinished()
     * @return the command stops when true
     */
    protected boolean isFinished() {
        return !Robot.oi.left.getRawButton(2) || !Robot.oi.left.getRawButton(2);
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


