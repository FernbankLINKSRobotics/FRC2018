package org.usfirst.frc.team4468.robot.Commands.Drive;

import org.usfirst.frc.team4468.robot.Constants;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Util.PID;
import org.usfirst.frc.team4468.robot.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RightDistance extends Command {
    private double distance;
    
    private Drivetrain dt = Robot.drive;
    private PID pid;
        
    public RightDistance(double d) {
    	System.out.println("PID Right Started");
        requires(dt);
        distance = d;
            
        pid = new PID(Constants.rightP, Constants.rightI, Constants.rightD, 1.0);
        pid.setOutputRange(-1.0, 1.0);
        pid.setPoint(distance);
    }
        
    /* Called repeatedly when this Command is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
    		System.out.println("Inside Right execute");
        System.out.println("Right Output: " + pid.calculate(dt.getLeftDistance()));
        System.out.println("Right Error: " + pid.getError());
        System.out.println("Right SetPoint: " + pid.getSetpoint());
        dt.tank(0, pid.calculate(dt.getRightDistance()));
    }

    /* Make this return true when this Command no longer needs to run execute()
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#isFinished()
     * @return the command stops when true
     */
    protected boolean isFinished() {
        return pid.onTarget();
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
