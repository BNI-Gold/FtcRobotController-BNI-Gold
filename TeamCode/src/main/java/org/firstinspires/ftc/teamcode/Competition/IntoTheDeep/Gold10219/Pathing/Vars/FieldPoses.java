package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Vars;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.localization.Pose;

public class FieldPoses {
    public SampleLines SampleLines = new SampleLines();
    public Chambers Chambers = new Chambers();
    public Nets Nets = new Nets();
    public Observations Observations = new Observations();

    public static final class SampleLines {
        private static final double x1 = 48.5;
        private static final double x2 = 58.5;
        private static final double x3 = 68.5;
        private static final double y = 25.75;
        public Audience Audience = new Audience();
        public Backstage Backstage = new Backstage();
        public static final class Audience {
            public Pose Neutral1 = new FCPose(x1, y);
            public Pose Neutral2 = new FCPose(x2, y);
            public Pose Neutral3 = new FCPose(x3, y);

            public Pose Red1 = new FCPose(x1, -y);
            public Pose Red2 = new FCPose(x2, -y);
            public Pose Red3 = new FCPose(x3, -y);
        }

        public static final class Backstage {
            public Pose Blue1 = new FCPose(-x1, y);
            public Pose Blue2 = new FCPose(-x2, y);
            public Pose Blue3 = new FCPose(-x3, y);

            public Pose Neutral1 = new FCPose(-x1, -y);
            public Pose Neutral2 = new FCPose(-x2, -y);
            public Pose Neutral3 = new FCPose(-x3, -y);
        }
    }
    public static final class Chambers {
        private static final double x = 0;
        private static final double y = 24;

        public Pose Blue = new FCPose(x, y);
        public Pose Red = new FCPose(x, -y);
    }
    public static final class Nets {
        private static final double x = 60;
        private static final double y = 60;
        private static final double blueHeading = -45;
        private static final double redHeading = 135;

        public Pose Blue = new FCPose(x, y, blueHeading);
        public Pose Red = new FCPose(-x, -y, redHeading);
    }
    public static final class Observations {
        private static final double x = 60;
        private static final double y = 58;
        private static final double blueHeading = -90;
        private static final double redHeading = 90;

        public Pose Blue = new FCPose(-x, y, blueHeading);
        public Pose Red = new FCPose(x, -y, redHeading);
    }
    public static final class Starts {
        private static final double x = 0;
        private static final double y = 0;
        private static final double blueHeading = 0;
        private static final double redHeading = 0;

        //TODO: Add robot positions
    }
}

//https://ftc-resources.firstinspires.org/file/ftc/game/manual-09