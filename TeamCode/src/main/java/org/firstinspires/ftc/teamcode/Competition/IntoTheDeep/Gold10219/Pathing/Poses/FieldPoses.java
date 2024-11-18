package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Poses;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.localization.Pose;

public class FieldPoses {

    public static final class SampleLines {

        public static final class SampleLines {


            public static final class Audience {
                public Pose Neutral1 = new FCPose()
            }
        }
    }

}

// FCPose class that extends Pose
class FCPose extends Pose {
    public FCPose(double xField, double yField) {
        super(xField + 72, yField + 72);
    }
}

//https://ftc-resources.firstinspires.org/file/ftc/game/manual-09