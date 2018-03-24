package org.usfirst.frc.team4468.robot.Commands.Routines;

import org.usfirst.frc.team4468.robot.Commands.Drive.StraightDistance;
import org.usfirst.frc.team4468.robot.Commands.Drive.TurnAngle;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class Run extends CommandGroup {
    public Run() {
    	addSequential(new TurnAngle(90,5));
        //addSequential(new StraightDistance(3.2, .05)); 
    }
}
