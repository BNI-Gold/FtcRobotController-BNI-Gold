package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.Point;

public class EasyPath extends Path {
    public EasyPath(Object startPoint, Object endPoint) {
        super(new BezierCurve(toPoint(startPoint), generateMidPoint(startPoint, endPoint), toPoint(endPoint)));
    }

    public EasyPath(Object startPoint, Object midPoint, Object endPoint) {
        super(new BezierCurve(toPoint(startPoint), toPoint(midPoint), toPoint(endPoint)));
    }

    public EasyPath(Object startPoint, Object control1, Object control2, Object finalPoint) {
        super(new BezierCurve(toPoint(startPoint), toPoint(control1), toPoint(control2), toPoint(finalPoint)));
    }

    public EasyPath(Object startPoint, Object endPoint, double[] xOffsets, double[] yOffsets) {
        super(new BezierCurve(toPoint(startPoint), generateMidPoint(startPoint, endPoint), generateOffsetPoint(new SafePoint(generateMidPoint(startPoint, endPoint), toPoint(endPoint)), xOffsets, yOffsets)));
    }

    public EasyPath(Object startPoint, Object midPoint, Object endPoint, double[] xOffsets, double[] yOffsets) {
        super(new BezierCurve(toPoint(startPoint), toPoint(midPoint), generateOffsetPoint(endPoint, xOffsets, yOffsets)));
    }

    public EasyPath(Object startPoint, Object control1, Object control2, Object endPoint, double[] xOffsets, double[] yOffsets) {
        super(new BezierCurve(toPoint(startPoint), toPoint(control1), toPoint(control2), generateOffsetPoint(endPoint, xOffsets, yOffsets)));
    }

    private static Point generateOffsetPoint(Object finalPoint, double[] xOffsets, double[] yOffsets) {
        double xOffsetSum = 0;
        double yOffsetSum = 0;

        for (int i = 0; i < xOffsets.length; i++) {
            xOffsetSum += xOffsets[i];
        }

        for (int i = 0; i < yOffsets.length; i++) {
            yOffsetSum += yOffsets[i];
        }

        Point finalPointPoint = toPoint(finalPoint);
        return new Point(finalPointPoint.getX() + xOffsetSum, finalPointPoint.getY() + yOffsetSum);
    }

    private static Point generateMidPoint(Object startPoint, Object endPoint) {
        Point start = toPoint(startPoint);
        Point end = toPoint(endPoint);

        double x = (start.getX() + end.getX()) / 2.0;
        double y = (start.getY() + end.getY()) / 2.0;

        return new Point(x, y);
    }

    private static Point toPoint(Object obj) {
        if (obj instanceof Pose) {
            return new Point((Pose) obj);
        } else if (obj instanceof Point) {
            return (Point) obj;
        } else {
            throw new IllegalArgumentException("Unsupported type. Must be Pose or Point.");
        }
    }
}
