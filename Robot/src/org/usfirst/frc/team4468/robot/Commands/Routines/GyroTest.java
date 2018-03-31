package org.usfirst.frc.team4468.robot.Commands.Routines;

import org.usfirst.frc.team4468.robot.Commands.Drive.TurnAngle;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class GyroTest extends CommandGroup {

    public GyroTest() {
        addSequential(new TurnAngle(-115.0, 5));
    }
}
