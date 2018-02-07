package org.usfirst.frc.team4468.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    // Joystick allocation
    public XboxController drvr = new XboxController(0);
    public XboxController ctrl = new XboxController(1);
    
    /* Sets up the triggers for different commands and other actions for when
     * a new button is pressed and it ties together the commands and subsystems
     */
    public OI() {
    
    }
}
