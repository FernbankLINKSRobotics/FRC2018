package org.usfirst.frc.team4468.robot.commands;

import org.usfirst.frc.team4468.robot.Constants;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Util.PID;
import org.usfirst.frc.team4468.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LeftDistance extends Command {

    private double distance;
    
    private Drivetrain dt = Robot.drive;
    private PID pid;
        
    public LeftDistance(double d) {
        requires(dt);
        distance = d;
            
        pid = new PID(Constants.leftP, Constants.leftI, Constants.leftD);
        pid.setOutputRange(-1, 1);
        pid.setPoint(distance);
    }
        
    /* Called repeatedly when this Command is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
        dt.drive(pid.calculate(dt.getLeftDistance()), 0);
    }

    /* Make this return true when this Command no longer needs to run execute()
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#isFinished()
     * @return the command stops when true
     */
    protected boolean isFinished() {
        return pid.onTarget(dt.getLeftDistance());
    }

    /* Called once after isFinished returns true
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#end()
     */
    protected void end() {
        dt.stop();
    }

    /* Called when another command which requires one or more of the same
     * subsystems is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#interrupted()
     */
    protected void interrupted() {
        dt.stop();
    }
}
