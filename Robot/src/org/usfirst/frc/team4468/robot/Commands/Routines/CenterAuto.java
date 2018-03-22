package org.usfirst.frc.team4468.robot.Commands.Routines;

import org.usfirst.frc.team4468.robot.Commands.Drive.StraightDistance;
import org.usfirst.frc.team4468.robot.Commands.Drive.TurnAngle;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.AngleRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.HoldingRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeClamp;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeSpeed;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.CommandGroup;


public class CenterAuto extends CommandGroup {

	
    public CenterAuto() {
        addSequential(new IntakeClamp(Value.kReverse));
        addParallel(new AngleRotate(-140.0,.5));
        addSequential(new StraightDistance(-2.0, .5)); //initial movement forward
        if(DriverStation.getInstance().getGameSpecificMessage().charAt(0)=='L') {
            addSequential(new TurnAngle(-90, 5)); //Assuming turning left is negative, -90 degrees
            addSequential(new StraightDistance(-1.75, .05)); //-1.0 is distance from center to middle of our colored side of switch
            addSequential(new TurnAngle(90,5)); //Assuming turning right is positive, +90 degrees
            addSequential(new StraightDistance(-1.00, .5)); 
            //Assuming distance needed to travel left is 3 (distance from alliance wall to switch is 4.27m according to game manual)
            addSequential(new IntakeSpeed(-.7));
            addSequential(new IntakeClamp(Value.kForward));
        } else if (DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R') {
        	 addSequential(new TurnAngle(90, 5)); //Assuming turning left is negative, -90 degrees
             addSequential(new StraightDistance(-1.52, .05)); //-1.0 is distance from center to middle of our colored side of switch
             addSequential(new TurnAngle(-90,5)); //Assuming turning right is positive, +90 degrees
             addSequential(new StraightDistance(-1.00, .5)); 
             //Assuming distance needed to travel left is 3 (distance from alliance wall to switch is 4.27m according to game manual)
             addSequential(new IntakeSpeed(-.7));
             addSequential(new IntakeClamp(Value.kForward));
        } else {
        	new Run().start();
        }
        /*
        addSequential(new TurnAngle(mult*90, 5)); //Assuming turning left is negative, -90 degrees
        addSequential(new StraightDistance(-1.0, .05)); //-1.0 is distance from center to middle of our colored side of switch
        addSequential(new TurnAngle(-mult*90,5)); //Assuming turning right is positive, +90 degrees
        addSequential(new StraightDistance(-1.00, .5)); 
        //Assuming distance needed to travel left is 3 (distance from alliance wall to switch is 4.27m according to game manual)
        addSequential(new IntakeSpeed(-.7));
        addSequential(new IntakeClamp(Value.kForward));
        */
    }
}

// Right auto is a positive mult and the op for left
