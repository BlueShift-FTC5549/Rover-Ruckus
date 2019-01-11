package com.blueshiftrobotics.ftc;

/**
 * An object to keep track of the robot position on the field. The coordinate origin is placed on
 * the blue depot.
 *
 * @version 1.0
 * @author Gabriel Wong
 */
public class FieldPoint {
    private double x;
    private double y;

    public double getX() { return x; }
    public double getY() { return y; }

    public void setX(double x) { this.x = x; }
    public void setY(double y) { this.y = y; }

    public FieldPoint transform(double dx, double dy) {
        setX(getX() + dx);
        setY(getY() + dy);

        return this;
    }

    public FieldPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }
}
