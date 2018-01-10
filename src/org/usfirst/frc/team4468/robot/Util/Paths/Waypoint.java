package org.usfirst.frc.team4468.robot.Util;

public class Waypoint {
    public double x, y, Vx, Vy, Ax, Ay;
    
    public Waypoint(double a, double b, double c, double d, double e, double f) {
        this.x  = a;
        this.y  = b;
        this.Vx = c;
        this.Vy = d;
        this.Ax = e;
        this.Ay = f;
    }    
}
