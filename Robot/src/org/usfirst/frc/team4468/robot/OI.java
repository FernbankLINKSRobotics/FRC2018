package org.usfirst.frc.team4468.robot;

import org.usfirst.frc.team4468.robot.Commands.Drive.Clamp;
import org.usfirst.frc.team4468.robot.Commands.Drive.Shift;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.AngleRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.BreakClamp;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.ExpelCube;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.HoldingRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeClamp;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeSpeed;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    // Joystick allocation
    public XboxController drvr = new XboxController(Constants.driveController);
    public XboxController ctrl = new XboxController(Constants.operatorController);

    /*
     * Sets up the triggers for different commands and other actions for when a new
     * button is pressed and it ties together the commands and subsystems
     */
    public OI() {
    		JoystickButton Trigger = new JoystickButton(ctrl, 1); 
    		JoystickButton LeftButton = new JoystickButton(ctrl, 4); 
    		JoystickButton CenterButton = new JoystickButton(ctrl, 3);
    		JoystickButton BottomButton = new JoystickButton(ctrl, 2);
    		JoystickButton RightButton = new JoystickButton(ctrl, 5);
    		JoystickButton LeftTop = new JoystickButton(ctrl, 6);
    		JoystickButton BottomLeft = new JoystickButton(ctrl, 8);
    		JoystickButton LeftMiddle = new JoystickButton(ctrl, 7);
    		JoystickButton RightTop = new JoystickButton(ctrl, 11);
    		JoystickButton RightBottom = new JoystickButton(ctrl, 10);
    		
    		JoystickButton RightTrigger = new JoystickButton(drvr, 6);
    		JoystickButton LeftTrigger = new JoystickButton(drvr, 5);
    		//4 unclamp 5 clamp
    		
    		BottomButton.whenPressed(new HoldingRotate(-10.0));
    		CenterButton.whenPressed(new HoldingRotate(-140.0));
    		LeftButton.whenPressed(new HoldingRotate(-50.0));
    		//RightButton.whenPressed(new BreakClamp(Value.kForward));
    		LeftMiddle.whenPressed(new IntakeClamp(Value.kReverse));
    		LeftTop.whenPressed(new IntakeClamp(Value.kForward));
    		BottomLeft.whenPressed(new IntakeSpeed(0.6));
    		Trigger.whenPressed(new ExpelCube(Value.kReverse, -.5));
    		RightTop.whenReleased(new Clamp(Value.kReverse));
    		RightBottom.whenPressed(new Clamp(Value.kForward));
    		
    		RightTrigger.whenPressed(new Shift(Value.kForward));
    		LeftTrigger.whenPressed(new Shift(Value.kReverse));
    }
}
