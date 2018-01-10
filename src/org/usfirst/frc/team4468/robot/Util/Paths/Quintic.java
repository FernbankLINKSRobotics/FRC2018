package org.usfirst.frc.team4468.robot.Util.Paths;

public class Quintic {
    private double a,b,c,d,e,f;
    
    /* This generates a 5th order polynomial
     * @param i ax^5
     * @param j bx^4
     * @param k cx^3
     * @param l dx^2
     * @param m ex
     * @param n f
     */
    public Quintic(double i, double j, double k, double l, double m, double n) {
        this.a = i;
        this.b = j;
        this.c = k;
        this.d = l;
        this.e = m;
        this.f = n;
    }
    
    /* This works better with the array outputted from PathGeneration.generate() 
     * @param a This is an array that has to be of size 6+ and it takes the first
     * 6 values
     */
    public Quintic(double[] a) {
        this.a = a[0];
        this.b = a[1];
        this.c = a[2];
        this.d = a[3];
        this.e = a[4];
        this.f = a[5];
    }
    
    /* This calculates the value of the equation given a specific value
     * @param t the parameterized variable range of 0-1
     */
    public double apply(double t) {
        return (a * Math.pow(t, 5) + 
                b * Math.pow(t, 4) +
                c * Math.pow(t, 3) +
                d * Math.pow(t, 2) +
                e * t +
                f);
    }
    
    /* This calculates the derivative of the equation given a specific value
     * @param t the parameterized variable range of 0-1
     */
    public double deriv(double t) {
        return ((a * Math.pow(t, 4) * 5) + 
                (b * Math.pow(t, 3) * 4) +
                (c * Math.pow(t, 2) * 3) +
                (d * t * 2) +
                 e);
    }
    
    /* This calculates the 2nd derivative of the equation given a specific value
     * @param t the parameterized variable range of 0-1
     */
    public double deriv2(double t) {
        return ((a * Math.pow(t, 3) * 20) + 
                (b * Math.pow(t, 2) * 12) +
                (c * t * 6) +
                (d * 2));
    }
}
