package org.usfirst.frc.team4468.robot.Util.Paths;

public class Waypoint {
    public double x, y, Vx, Vy, Ax, Ay;
    
    // This class is similar to a C++ struct
    public Waypoint(double a, double b, double c, double d, double e, double f) {
        this.x  = a; // the X position
        this.y  = b; // the Y position
        this.Vx = c; // The X component of the velocity
        this.Vy = d; // The Y component of the velocity
        this.Ax = e; // The X component of the acceleration
        this.Ay = f; // The Y component of the acceleration
    }    
}
