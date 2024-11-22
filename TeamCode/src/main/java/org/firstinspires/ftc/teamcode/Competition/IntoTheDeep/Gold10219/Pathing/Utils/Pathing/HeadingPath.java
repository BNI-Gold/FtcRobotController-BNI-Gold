package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Pathing;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.Path;

public interface HeadingPath {
    HeadingPath setHeading(HeadingTypes type, Pose pose);

    HeadingPath setHeading(HeadingTypes type, Path path, Pose pose);

    HeadingPath setHeading(HeadingTypes type, Pose pose1, Pose pose2);

    HeadingPath setHeading(HeadingTypes type, double radian, Pose pose);

    HeadingPath setHeading(HeadingTypes type, Pose pose, double radian);
}