package org.usfirst.frc.team4468.robot.Commands.Routines;

import org.usfirst.frc.team4468.robot.Commands.Drive.Clamp;
import org.usfirst.frc.team4468.robot.Commands.Drive.StraightDistance;
import org.usfirst.frc.team4468.robot.Commands.Drive.TurnAngle;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.CommandGroup;


public class Run extends CommandGroup {
    public Run() {
    	System.out.println("IN RUN");
    	addSequential(new Clamp(Value.kForward));
        addSequential(new StraightDistance(-3.0, .05)); 
    }
}
