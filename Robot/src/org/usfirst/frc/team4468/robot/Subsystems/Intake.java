package org.usfirst.frc.team4468.robot.Subsystems;

import org.usfirst.frc.team4468.robot.Constants;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.VictorSP;

public class Intake extends Subsystem {
	//// Declarations
	private VictorSP left = new VictorSP(Constants.intakePort1);
	private VictorSP right = new VictorSP(Constants.intakePort2);
	private DoubleSolenoid clamp = new DoubleSolenoid(Constants.intakeClampPort1, Constants.intakeClampPort2);
	
	
	
	//// Constructor and Command
	public Intake() {
		//  The left motor needs to spin clockwise so it needs to be the inverse of the other motor, which is counter clockwise
		left.setInverted(true);
	}
	
	protected void initDefaultCommand() {}
	
	
	
	//// Motor Control
	/* Sets the motors to a speed
	 */
	public void setSpeed(double speed) {
		left.set(speed);
		right.set(speed);
	}
	
	/* When the robot is stoped the intake system is stopped
	 */
	public void stop() {
		left.stopMotor();
		right.stopMotor();
	}
	
	
	//// Pnuematic Control
	/* Sets the state of the clamp that holds the cube
	 */
	public void clamp(Value v) {
	    clamp.set(v);
	}
	
	/* This returns the state of the solenoids
	 * @return their similar state or an error is thrown
	 */
	public Value getState() {
	    return clamp.get();
	}
}
