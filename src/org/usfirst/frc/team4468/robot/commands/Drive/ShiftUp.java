package org.usfirst.frc.team4468.robot.commands.Drive;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.subsystems.Shifter;

public class ShiftUp extends Command {

	public boolean prevState;
	public Shifter sf = Robot.shift;
	
	public ShiftUp() {
		requires(sf);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		prevState = sf.isHighGear();
		
		if(!sf.isHighGear()) {
			sf.up();
		} else {
			sf.down();
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return !(prevState == sf.isHighGear()); // Runs until interrupted
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}
}