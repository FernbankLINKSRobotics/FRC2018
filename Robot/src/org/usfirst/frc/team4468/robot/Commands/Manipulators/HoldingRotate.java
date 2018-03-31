package org.usfirst.frc.team4468.robot.Commands.Manipulators;

import org.usfirst.frc.team4468.robot.Constants;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Util.PID;
import org.usfirst.frc.team4468.robot.Subsystems.RotatingLift;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

/**
 * This uses the Pot attached to the rotating arm to move
 * to a specific angle.
 */
public class HoldingRotate extends Command {

    private RotatingLift rl = Robot.rotatingLift;
    private PID pid;
    
    
    public HoldingRotate(double angle) {
    		double theta = angle;
    		System.out.println("PID RotateAngle");

        pid = new PID(Constants.lifterP, Constants.lifterI, Constants.lifterD);
        pid.reset();
        
        //pid.setInputRange(-180.0, 180.0);
        pid.setOutputRange(-1.0, 1.0);
        pid.setPerTolerance(.5);
        
        //System.out.println(pid.getSetpoint());
        pid.setPoint(theta);
        //System.out.println("setpoint one: " + pid.getSetpoint());
        
    }
    
    /* Called repeatedly when this Command is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
    		System.out.println("Executed");
    		rl.rotate(pid.calculate(rl.getAngle()));
    		//System.out.println("Calculated Distance:" + pid.calculate(rl.getAngle()));
    }

    /* Make this return true when this Command no longer needs to run execute()
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#isFinished()
     * @return the command stops when true
     */
    protected boolean isFinished() {
    	if(DriverStation.getInstance().isAutonomous()) {
    		return false;
    	} else {
    		return !(Robot.oi.ctrl.getRawButton(5) ||
    				 Robot.oi.ctrl.getRawButton(2) ||
    				 Robot.oi.ctrl.getRawButton(3) ||
    				 Robot.oi.ctrl.getRawButton(4));
        }
    }

    /* Called once after isFinished returns true
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#end()
     */
    protected void end() {
        rl.stop();
        pid.disable();
    }

    /* Called when another command which requires one or more of the same
     * subsystems is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#interrupted()
     */
    protected void interrupted() {
        rl.stop();
        pid.disable();
    }
}
