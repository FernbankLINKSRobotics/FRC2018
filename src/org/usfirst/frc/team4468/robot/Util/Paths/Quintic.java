package org.usfirst.frc.team4468.robot.Util.Paths;

public class Quintic {
    private double a,b,c,d,e,f;
    
    public Quintic(double i, double j, double k, double l, double m, double n) {
        this.a = i;
        this.b = j;
        this.c = k;
        this.d = l;
        this.e = m;
        this.f = n;
    }
    
    public Quintic(double[] a) {
        this.a = a[0];
        this.b = a[1];
        this.c = a[2];
        this.d = a[3];
        this.e = a[4];
        this.f = a[5];
    }
    
    public double apply(double t) {
        return (a * Math.pow(t, 5) + 
                b * Math.pow(t, 4) +
                c * Math.pow(t, 3) +
                d * Math.pow(t, 2) +
                e * t +
                f);
    }
    
    public double deriv(double t) {
        return ((a * Math.pow(t, 4) * 5) + 
                (b * Math.pow(t, 3) * 4) +
                (c * Math.pow(t, 2) * 3) +
                (d * t * 2) +
                 e);
    }
    
    public double deriv2(double t) {
        return ((a * Math.pow(t, 3) * 20) + 
                (b * Math.pow(t, 2) * 12) +
                (c * t * 6) +
                (d * 2));
    }
}
