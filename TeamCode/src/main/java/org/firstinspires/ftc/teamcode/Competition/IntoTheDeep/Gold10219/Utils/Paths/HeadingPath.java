package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Utils.Paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;

public interface HeadingPath {
    HeadingPath setHeading(HeadingTypes type, Pose pose);

    HeadingPath setHeading(HeadingTypes type, Path path, Pose pose);

    HeadingPath setHeading(HeadingTypes type, Pose pose1, Pose pose2);

    HeadingPath setHeading(HeadingTypes type, double radian, Pose pose);

    HeadingPath setHeading(HeadingTypes type, Pose pose, double radian);
}