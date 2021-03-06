package org.usfirst.frc.team4468.robot.Commands.Drive;

import org.usfirst.frc.team4468.robot.Constants;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Util.PID;
import org.usfirst.frc.team4468.robot.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LeftDistance extends Command {

    private double distance;
    
    private Drivetrain dt = Robot.drive;
    private PID pid;
        
    public LeftDistance(double d) {
    	
    		System.out.println("PID");
    	
        requires(dt);
        distance = d;
        pid = new PID(Constants.leftP, Constants.leftI, Constants.lifterD, 1.0);
        pid.setOutputRange(-1.0, 1.0);
        pid.setPoint(distance); 		
    }
    
 
        
    /* Called repeatedly when this Command is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
    	/*
    		System.out.println("Inside Left execute");
    		System.out.println("Left Output: " + pid.calculate(dt.getLeftDistance()));
    		System.out.println("Left Error: " + pid.getError());
    		System.out.println("Left SetPoint: " + pid.getSetpoint());
    		*/
    	dt.tank(pid.calculate(dt.getLeftDistance()), 0.0);
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
    		System.out.println("Interrupted");
        dt.stop();
    }
}
