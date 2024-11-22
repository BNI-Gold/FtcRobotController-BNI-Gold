package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Pathing;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Offsets;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.SafeInterpolationStartHeading;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.SafePoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.Point;

import java.security.InvalidParameterException;

public class EasySafePath extends Path {
    public EasySafePath(Object startPoint, Object endPoint) {
        super(new BezierCurve(toPoint(startPoint), generateMidPoint(startPoint, endPoint), new SafePoint(generateMidPoint(startPoint, endPoint), toPoint(endPoint))));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasySafePath(Object startPoint, Object midPoint, Object endPoint) {
        super(new BezierCurve(toPoint(startPoint), toPoint(midPoint), new SafePoint(toPoint(midPoint), toPoint(endPoint))));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasySafePath(Object startPoint, Object control1, Object control2, Object finalPoint) {
        super(new BezierCurve(toPoint(startPoint), toPoint(control1), toPoint(control2), new SafePoint(toPoint(control2), toPoint(finalPoint))));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasySafePath(Object startPoint, Object control1, Object control2, Object control3, Object finalPoint) {
        super(new BezierCurve(toPoint(startPoint), toPoint(control1), toPoint(control2), toPoint(control3), new SafePoint(toPoint(control3), toPoint(finalPoint))));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasySafePath(Object startPoint, Object endPoint, Offsets offsets) {
        super(new BezierCurve(toPoint(startPoint), generateMidPoint(startPoint, endPoint), generateOffsetPoint(new SafePoint(generateMidPoint(startPoint, endPoint), toPoint(endPoint)), offsets)));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasySafePath(Object startPoint, Object midPoint, Object endPoint, Offsets offsets) {
        super(new BezierCurve(toPoint(startPoint), toPoint(midPoint), generateOffsetPoint(new SafePoint(toPoint(midPoint), toPoint(endPoint)), offsets)));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasySafePath(Object startPoint, Object control1, Object control2, Object endPoint, Offsets offsets) {
        super(new BezierCurve(toPoint(startPoint), toPoint(control1), toPoint(control2), generateOffsetPoint(new SafePoint(toPoint(control2), toPoint(endPoint)), offsets)));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasySafePath(Object startPoint, Object control1, Object control2, Object control3, Object endPoint, Offsets offsets) {
        super(new BezierCurve(toPoint(startPoint), toPoint(control1), toPoint(control2), toPoint(control3), generateOffsetPoint(new SafePoint(toPoint(control3), toPoint(endPoint)), offsets)));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasySafePath setHeading(HeadingTypes type, Pose pose) {
        if (type == HeadingTypes.CONSTANT) {
            this.setConstantHeadingInterpolation(pose.getHeading());
        } else {
            throw new InvalidParameterException("For Linear heading interpolation, pass in a path or radian, as well as a pose.");
        }
        return this;
    }

    public EasySafePath setHeading(HeadingTypes type, Path path, Pose pose) {
        if (type == HeadingTypes.LINEAR) {
            this.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(path.getEndTangent().getTheta(), pose).getValue(), pose.getHeading(), .8);
        } else {
            throw new InvalidParameterException("For Constant heading interpolation, only pass in a pose.");
        }
        return this;
    }

    public EasySafePath setHeading(HeadingTypes type, Pose pose1, Pose pose2) {
        if (type == HeadingTypes.LINEAR) {
            this.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(pose1.getHeading(), pose2).getValue(), pose2.getHeading(), .8);
        } else {
            throw new InvalidParameterException("For Constant heading interpolation, only pass in a pose.");
        }
        return this;
    }

    public EasySafePath setHeading(HeadingTypes type, double radian, Pose pose) {
        if (type == HeadingTypes.LINEAR) {
            this.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(radian, pose).getValue(), pose.getHeading(), .8);
        } else {
            throw new InvalidParameterException("For Constant heading interpolation, only pass in a pose.");
        }
        return this;
    }

    public EasySafePath setHeading(HeadingTypes type, Pose pose, double radian) {
        if (type == HeadingTypes.LINEAR) {
            this.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(pose, radian).getValue(), radian, .8);
        } else {
            throw new InvalidParameterException("For Constant heading interpolation, only pass in a pose.");
        }
        return this;
    }

    public EasySafePath setHeading(HeadingTypes type, Path path, double radian) {
        if (type == HeadingTypes.LINEAR) {
            this.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(path.getEndTangent().getTheta(), radian).getValue(), radian, .8);
        } else {
            throw new InvalidParameterException("For Constant heading interpolation, only pass in a pose.");
        }
        return this;
    }

    private static Point generateOffsetPoint(Object finalPoint, Offsets offsets) {
        double xOffsetSum = 0;
        double yOffsetSum = 0;

        for (int i = 0; i < offsets.getXAsArray().length; i++) {
            xOffsetSum += offsets.getXAsArray()[i];
        }

        for (int i = 0; i < offsets.getYAsArray().length; i++) {
            yOffsetSum += offsets.getYAsArray()[i];
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
