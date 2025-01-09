package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Utils;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBotVars;

public class SafePoint extends Point{
    public SafePoint(Point Previous, Point Final) {
        super(0, 0);

        double distance = MathFunctions.distance(Final, Previous);
        CompBotVars vars = new CompBotVars();

        double x = Final.getX()
                + MathFunctions.getSign(
                        Previous.getX()
                        - Final.getX())
                * Math.abs(
                        Previous.getX()
                        - Final.getX())
                * vars.Chassis.FRONT_LENGTH
                / distance;

        double y = Final.getY()
                + MathFunctions.getSign(
                        Previous.getY()
                        - Final.getY())
                * Math.abs(
                        Previous.getY()
                        - Final.getY())
                * vars.Chassis.FRONT_LENGTH
                / distance;

        setCoordinates(x, y, Point.CARTESIAN);
    }

    public SafePoint(Point Previous, Pose Final) {
        super(0, 0);

        double distance = MathFunctions.distance(Final, Previous);
        CompBotVars vars = new CompBotVars();

        double x = Final.getX()
                + MathFunctions.getSign(
                Previous.getX()
                        - Final.getX())
                * Math.abs(
                Previous.getX()
                        - Final.getX())
                * vars.Chassis.FRONT_LENGTH
                / distance;

        double y = Final.getY()
                + MathFunctions.getSign(
                Previous.getY()
                        - Final.getY())
                * Math.abs(
                Previous.getY()
                        - Final.getY())
                * vars.Chassis.FRONT_LENGTH
                / distance;

        setCoordinates(x, y, Point.CARTESIAN);
    }
}
