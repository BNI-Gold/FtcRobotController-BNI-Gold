package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Vars;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBotVars;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.localization.Pose;

public class FieldPoses {
    public SampleLines SampleLines = new SampleLines();
    public Chambers Chambers = new Chambers();
    public Nets Nets = new Nets();
    public Observations Observations = new Observations();
    public Recalibration Recalibration = new Recalibration();

    private static CompBotVars vars = new CompBotVars();

    private static final int f = 144;

    public static final class SampleLines {
        public Audience Audience = new Audience();
        public Rear Rear = new Rear();

        public double pushApproachAngle = Math.toRadians(90);

        private static final double x1 = 23.5;
        private static final double x2 = 13.5;
        private static final double x3 = 3.5;
        private static final double y = 46.25;

        public static final class Audience {
            public Blue Blue = new Blue();

            public Pose Neutral1 = new Pose(x1, y);
            public Pose Neutral2 = new Pose(x2, y);
            public Pose Neutral3 = new Pose(x3, y);

            public static final class Blue {
                public B1 B1 = new B1();
                public B2 B2 = new B2();
                public B3 B3 = new B3();

                public static final class B1 {
                    public Pose Sample = new Pose(x1, f-y);

                    public Pose Slip = new Pose(27, f - 18);
                    public Pose Pre = new Pose(33, f - 24);
                    public Pose Post = new Pose(40, f - 66);
                }

                public static final class B2 {
                    public Pose Sample = new Pose(x2, f-y);

                    public Pose Pre = new Pose(28, f-24);
                    public Pose Post = new Pose(30, f-66);
                }

                public static final class B3 {
                    public Pose Sample = new Pose(x3, f-y);

                    public Pose Pre = new Pose(13, f-24);
                    public Pose Post = new Pose(20, f-66);
                }
            }
        }

        public static final class Rear {
            public Midpoints Midpoints = new Midpoints();

            public Pose Neutral1 = new Pose(f - x1, f - y);
            public Pose Neutral2 = new Pose(f - x2, f - y);
            public Pose Neutral3 = new Pose(f - x3, f - y);

            public Pose Red1 = new Pose(f - x1, y);
            public Pose Red2 = new Pose(f - x2, y);
            public Pose Red3 = new Pose(f - x2, y);

            public static final class Midpoints {
                public Pose Pre = new Pose(f - 36, f - 36);
                public Pose Post = new Pose(f - 36, f - 60);
            }
        }
    }

    public static final class Chambers {
        private static final double x = 72;
        private static final double y = 48;

        public Pose Blue = new Pose(x, f - y, Math.toRadians(-90));
        public Pose Red = new Pose(x, y);

        public Midpoints Midpoints = new Midpoints();

        public static final class Midpoints {
            private static final double x = 36;
            private static final double y = 30;

            public Pose Blue = new Pose(x, f - y);
            public Pose Red = new Pose(x, y);
        }
    }

    public static final class Nets {
        private static final double x = 12;
        private static final double y = 12;

        public Pose Blue = new Pose(f - x, f - y);
        public Pose Red = new Pose(x, y);
    }

    public static final class Observations {
        private static final double x = 15;
        private static final double y = 14;

        public Approaches Approaches = new Approaches();
        public Grabs Grabs = new Grabs();
        public Retreats Retreats = new Retreats();

        public Pose Blue = new Pose(x, f - y, Math.toRadians(90));
        public Pose Red = new Pose(f - x, y);

        //NOTE: X value for Approaches and Grabs must be same in order for offset calculation to work in auto
        public static final class Approaches {
            private static final double y = 22;
            public Pose Blue = new Pose(x, f-y, Math.toRadians(90));
            public Pose Red = new Pose(f-x, y);
        }

        public static final class Grabs {
            private static final double y = vars.Chassis.FRONT_LENGTH + 3.5;
            public Pose Blue = new Pose(x, f-y, Math.toRadians(90));
            public Pose Red = new Pose(f-x, y);
        }

        public static final class Retreats {
            private static final double y = 20;
            public Pose Blue = new Pose(x, f-y);
            public Pose Red = new Pose(f-x, y);
        }
    }

    public static final class Recalibration {
        public Single Single = new Single();
        public Double Double = new Double();

        public static final class Single {
            public Pose A11 = new Pose(48, 120, Math.toRadians(-180));
            public Pose A12 = new Pose(72, 120, Math.toRadians(90));
            public Pose A13 = new Pose(96, 120, Math.toRadians(0));
            public Pose A14 = new Pose(96, 24, Math.toRadians(0));
            public Pose A15 = new Pose(72, 24, Math.toRadians(-90));
            public Pose A16 = new Pose(48, 24, Math.toRadians(180));
        }

        public static final class Double {

        }
    }
}