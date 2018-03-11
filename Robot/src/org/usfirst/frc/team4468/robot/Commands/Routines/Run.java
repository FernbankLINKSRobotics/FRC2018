package org.usfirst.frc.team4468.robot.Commands.Routines;

import org.usfirst.frc.team4468.robot.Commands.Drive.StraightDistance;
import edu.wpi.first.wpilibj.command.CommandGroup;


public class Run extends CommandGroup {
    public Run() { 
        addSequential(new StraightDistance(3.2, .05)); 
    }
}
