package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.MathFunctions;

public class SafeInterpolationStartHeading {
    private final double value;

    public SafeInterpolationStartHeading(Pose startPose, Pose endPose) {
        double startHeading = startPose.getHeading();
        double endHeading = endPose.getHeading();
        this.value = startHeading
                - 0.1
                * MathFunctions.getTurnDirection(startHeading, endHeading)
                * MathFunctions.getSmallestAngleDifference(startHeading, endHeading);
    }

    public SafeInterpolationStartHeading(double startHeading, Pose endPose) {
        double endHeading = endPose.getHeading();
        this.value = startHeading
                - 0.1
                * MathFunctions.getTurnDirection(startHeading, endHeading)
                * MathFunctions.getSmallestAngleDifference(startHeading, endHeading);
    }

    public double getValue() {
        return value;
    }
}