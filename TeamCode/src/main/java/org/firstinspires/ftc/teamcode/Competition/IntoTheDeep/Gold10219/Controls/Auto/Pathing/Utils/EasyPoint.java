package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Utils;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.Point;

public class EasyPoint extends BezierPoint {
    public EasyPoint(Pose pose, Offsets offsets) {
        super(generateOffsetPoint(toPoint(pose), offsets));
    }

    public EasyPoint(Pose pose) {
        super(new Point(toPoint(pose).getX(), toPoint(pose).getY()));
    }

    public static Point generateOffsetPoint(Point finalPoint, Offsets offsets) {
        double xOffsetSum = offsets.getXTotalOffsets();
        double yOffsetSum = offsets.getYTotalOffsets();

        return new Point(finalPoint.getX() + xOffsetSum, finalPoint.getY() + yOffsetSum);
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