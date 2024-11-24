package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Utils;

import java.util.ArrayList;

public class Offsets {
    public ArrayList<Double> x;
    public ArrayList<Double> y;

    public Offsets() {
        this.x = new ArrayList<>();
        this.y = new ArrayList<>();
    }

    public Offsets(double[] xOffsets, double[] yOffsets) {
        this();
        for (double value : xOffsets) {
            this.x.add(value);
        }
        for (double value : yOffsets) {
            this.y.add(value);
        }
    }

    public Offsets addX(double xValue) {
        this.x.add(xValue);
        return this;
    }

    public Offsets remX(double xValue) {
        this.x.add(-xValue);
        return this;
    }

    public Offsets addY(double yValue) {
        this.y.add(yValue);
        return this;
    }

    public Offsets remY(double yValue) {
        this.y.add(-yValue);
        return this;
    }

    public double getXTotalOffsets() {
        return this.x.stream().mapToDouble(Double::doubleValue).sum();
    }

    public double getYTotalOffsets() {
        return this.y.stream().mapToDouble(Double::doubleValue).sum();
    }
}