package org.usfirst.frc.team4468.robot.Commands.Manipulators;

import org.usfirst.frc.team4468.robot.Constants;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Subsystems.RotatingLift;
import org.usfirst.frc.team4468.robot.Util.PID;

import edu.wpi.first.wpilibj.command.Command;

public class AngleRotate extends Command {
    
    private RotatingLift rl = Robot.rotatingLift;
    private PID pid;

    public AngleRotate(double angle) {
        double theta = angle;
        System.out.println("PID RotateAngle");

        pid = new PID(Constants.lifterP, Constants.lifterI, Constants.lifterD);
        pid.reset();
        pid.setOutputRange(-1.0, 1.0);
        pid.setPerTolerance(.5);

        System.out.println(pid.getSetpoint());
        pid.setPoint(theta);
        System.out.println("setpoint one: " + pid.getSetpoint());
    }

    /*
     * Called repeatedly when this Command is scheduled to run (non-Javadoc)
     * 
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
        System.out.println("Executed");
        rl.rotate(pid.calculate(rl.getAngle()));
        System.out.println("Calculated Distance:" + pid.calculate(rl.getAngle()));
    }

    /*
     * Make this return true when this Command no longer needs to run execute()
     * (non-Javadoc)
     * 
     * @see edu.wpi.first.wpilibj.command.Command#isFinished()
     * 
     * @return the command stops when true
     */
    protected boolean isFinished() {
        return pid.onTarget(rl.getAngle());
    }

    /*
     * Called once after isFinished returns true (non-Javadoc)
     * 
     * @see edu.wpi.first.wpilibj.command.Command#end()
     */
    protected void end() {
        rl.stop();
    }

    /*
     * Called when another command which requires one or more of the same subsystems
     * is scheduled to run (non-Javadoc)
     * 
     * @see edu.wpi.first.wpilibj.command.Command#interrupted()
     */
    protected void interrupted() {
        rl.stop();
    }
}
