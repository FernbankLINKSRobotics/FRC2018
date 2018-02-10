package org.usfirst.frc.team4468.robot;

import org.usfirst.frc.team4468.robot.Commands.Drive.Shift;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeClamp;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeSpeed;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.Rotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.RotateAngle;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    // Joystick allocation
    public XboxController drvr = new XboxController(Constants.driveController);
    public XboxController ctrl = new XboxController(Constants.operatorController);
    
    /* Sets up the triggers for different commands and other actions for when
     * a new button is pressed and it ties together the commands and subsystems
     */
    public OI() {
        if (drvr.getTriggerAxis(Hand.kRight) == 1) {
            new Shift(Value.kForward);
        } else if (drvr.getTriggerAxis(Hand.kLeft) == 1) {
            new Shift(Value.kReverse);
        }
        
        if (ctrl.getAButtonPressed()) {
            new RotateAngle(0);
        } else if (ctrl.getBButtonPressed()) {
            new RotateAngle(60);
        } else if (ctrl.getXButtonPressed()) {
            new RotateAngle(120);
        } else if (ctrl.getYButtonPressed()) {
            new RotateAngle(180);
        }
            
        if (ctrl.getBumperPressed(Hand.kRight)) {
            new IntakeClamp(Value.kForward);
        } else if (ctrl.getBumperPressed(Hand.kLeft)) {
            new IntakeClamp(Value.kReverse);
        }
        
        if (ctrl.getTriggerAxis(Hand.kRight) == 1) {
            new IntakeSpeed(1);
        } else if (ctrl.getTriggerAxis(Hand.kLeft) == 1) {
            new IntakeSpeed(-1);
        }
        
        if(ctrl.getAButton() == true) {
            new Rotate(false);
        } else if(ctrl.getYButton() == true){
            new Rotate(true);
        }
    }
}
