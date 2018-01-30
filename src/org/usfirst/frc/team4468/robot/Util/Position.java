package org.usfirst.frc.team4468.robot.Util;

public class Position {
    //// Declarations
    private double x;
    private double y;
    
    
    
    //// Constructor
    public Position(double ix, double iy) {
        x = ix;
        y = iy;
    }
    
    
    
    //// Update
    /* Updates the position based on your current state
     * @param v the robot's average velocity
     * @param t the robot's angle
     * @param dt the change in time
     */
    public void update(double v, double t, double dt) {
        x += (Math.sin(t) * v * dt);
        y += (Math.cos(t) * v * dt);
    }
    
    
    
    //// Getters
    /* Gets the X component of the robot's current position
     * @return the robot's X position
     */
    public double getX() {
        return x;
    }
    
    /* Gets the Y component of the robot's current position
     * @return the robot's Y position
     */
    public double getY() {
        return y;
    }
    
    /* Gets the robot's current position
     * @return the X,Y pair that represents the position
     */
    public Pair<Double, Double> get() {
        return new Pair<Double, Double>(x, y);
    }
}
