package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Paths;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Offsets;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.SafeInterpolationStartHeading;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import java.security.InvalidParameterException;

public class EasyPath extends Path {
    public EasyPath(Object startPoint, Object endPoint) {
        super(new BezierCurve(toPoint(startPoint), generateMidPoint(startPoint, endPoint), toPoint(endPoint)));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasyPath(Object startPoint, Object midPoint, Object endPoint) {
        super(new BezierCurve(toPoint(startPoint), toPoint(midPoint), toPoint(endPoint)));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasyPath(Object startPoint, Object control1, Object control2, Object finalPoint) {
        super(new BezierCurve(toPoint(startPoint), toPoint(control1), toPoint(control2), toPoint(finalPoint)));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasyPath(Object startPoint, Object control1, Object control2, Object control3, Object finalPoint) {
        super(new BezierCurve(toPoint(startPoint), toPoint(control1), toPoint(control2), toPoint(control3), toPoint(finalPoint)));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasyPath(Object startPoint, Object endPoint, Offsets offsets) {
        super(new BezierCurve(toPoint(startPoint), generateMidPoint(startPoint, endPoint), generateOffsetPoint(endPoint, offsets)));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasyPath(Object startPoint, Object midPoint, Object endPoint, Offsets offsets) {
        super(new BezierCurve(toPoint(startPoint), toPoint(midPoint), generateOffsetPoint(endPoint, offsets)));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasyPath(Object startPoint, Object control1, Object control2, Object endPoint, Offsets offsets) {
        super(new BezierCurve(toPoint(startPoint), toPoint(control1), toPoint(control2), generateOffsetPoint(endPoint, offsets)));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasyPath(Object startPoint, Object control1, Object control2, Object control3, Object endPoint, Offsets offsets) {
        super(new BezierCurve(toPoint(startPoint), toPoint(control1), toPoint(control2), toPoint(control3), generateOffsetPoint(endPoint, offsets)));
        this.setPathEndTimeoutConstraint(3);
    }

    public EasyPath setHeading(HeadingTypes type, Pose pose) {
        if (type == HeadingTypes.CONSTANT) {
            this.setConstantHeadingInterpolation(pose.getHeading());
        } else {
            throw new InvalidParameterException("For Linear heading interpolation, pass in a path or radian, as well as a pose.");
        }
        return this;
    }

    public EasyPath setHeading(HeadingTypes type, double radian) {
        if (type == HeadingTypes.CONSTANT) {
            this.setConstantHeadingInterpolation(radian);
        } else {
            throw new InvalidParameterException("For Linear heading interpolation, pass in a path or radian, as well as a pose.");
        }
        return this;
    }

    public EasyPath setHeading(HeadingTypes type, Path path, Pose pose) {
        if (type == HeadingTypes.LINEAR) {
            this.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(path.getEndTangent().getTheta(), pose).getValue(), pose.getHeading(), .8);
        } else {
            throw new InvalidParameterException("For Constant heading interpolation, only pass in a pose.");
        }
        return this;
    }

    public EasyPath setHeading(HeadingTypes type, Pose pose1, Pose pose2) {
        if (type == HeadingTypes.LINEAR) {
            this.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(pose1.getHeading(), pose2).getValue(), pose2.getHeading(), .8);
        } else {
            throw new InvalidParameterException("For Constant heading interpolation, only pass in a pose.");
        }
        return this;
    }

    public EasyPath setHeading(HeadingTypes type, double radian, Pose pose) {
        if (type == HeadingTypes.LINEAR) {
            this.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(radian, pose).getValue(), pose.getHeading(), .8);
        } else {
            throw new InvalidParameterException("For Constant heading interpolation, only pass in a pose.");
        }
        return this;
    }

    public EasyPath setHeading(HeadingTypes type, Pose pose, double radian) {
        if (type == HeadingTypes.LINEAR) {
            this.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(pose, radian).getValue(), radian, .8);
        } else {
            throw new InvalidParameterException("For Constant heading interpolation, only pass in a pose.");
        }
        return this;
    }

    public EasyPath setHeading(HeadingTypes type, Path path, double radian) {
        if (type == HeadingTypes.LINEAR) {
            this.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(path.getEndTangent().getTheta(), radian).getValue(), radian, .8);
        } else {
            throw new InvalidParameterException("For Constant heading interpolation, only pass in a pose.");
        }
        return this;
    }

    public EasyPath setHeading(HeadingTypes type, Path path, Pose pose, double end) {
        if (type == HeadingTypes.LINEAR) {
            this.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(path.getEndTangent().getTheta(), pose).getValue(), pose.getHeading(), end);
        } else {
            throw new InvalidParameterException("For Constant heading interpolation, only pass in a pose.");
        }
        return this;
    }

    public EasyPath setHeading(HeadingTypes type, Pose pose1, Pose pose2, double end) {
        if (type == HeadingTypes.LINEAR) {
            this.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(pose1.getHeading(), pose2).getValue(), pose2.getHeading(), end);
        } else {
            throw new InvalidParameterException("For Constant heading interpolation, only pass in a pose.");
        }
        return this;
    }

    public EasyPath setHeading(HeadingTypes type, double radian, Pose pose, double end) {
        if (type == HeadingTypes.LINEAR) {
            this.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(radian, pose).getValue(), pose.getHeading(), end);
        } else {
            throw new InvalidParameterException("For Constant heading interpolation, only pass in a pose.");
        }
        return this;
    }

    public EasyPath setHeading(HeadingTypes type, Pose pose, double radian, double end) {
        if (type == HeadingTypes.LINEAR) {
            this.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(pose, radian).getValue(), radian, end);
        } else {
            throw new InvalidParameterException("For Constant heading interpolation, only pass in a pose.");
        }
        return this;
    }

    public EasyPath setHeading(HeadingTypes type, Path path, double radian, double end) {
        if (type == HeadingTypes.LINEAR) {
            this.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(path.getEndTangent().getTheta(), radian).getValue(), radian, end);
        } else {
            throw new InvalidParameterException("For Constant heading interpolation, only pass in a pose.");
        }
        return this;
    }

    private static Point generateOffsetPoint(Object finalPoint, Offsets offsets) {
        double xOffsetSum = offsets.getXTotalOffsets();
        double yOffsetSum = offsets.getYTotalOffsets();

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
