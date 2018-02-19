package org.usfirst.frc.team4468.robot.Commands.Drive;

import org.usfirst.frc.team4468.robot.Constants;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Subsystems.Drivetrain;
import org.usfirst.frc.team4468.robot.Util.PID;
import edu.wpi.first.wpilibj.command.Command;

public class StraightDistance extends Command {
    private double distance;
    
    private Drivetrain dt = Robot.drive;
    private PID pid;
        
    public StraightDistance(double d) {
        System.out.println("PID Right Started");
        requires(dt);
        distance = d;
            
        pid = new PID(Constants.lineP, Constants.lineI, Constants.lineD);
        pid.setOutputRange(-1, 1);
        pid.setAbsTolerance(500);
        pid.setPoint(distance);
    }
        
    /* Called repeatedly when this Command is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
        System.out.println("Inside Line execute");
        System.out.println("Line Output: " + pid.calculate(dt.getLeftDistance()));
        System.out.println("Line Error: " + pid.getError());
        System.out.println("Line SetPoint: " + pid.getSetpoint());
        System.out.println("Distance: " + dt.getDis());
        dt.arcade(0, pid.calculate(dt.getDis()));
    }

    /* Make this return true when this Command no longer needs to run execute()
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#isFinished()
     * @return the command stops when true
     */
    protected boolean isFinished() {
        return pid.onTarget(dt.getDis());
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
