package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Vars;

import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBotVars;

public class FieldPoses {
    public SampleLines SampleLines = new SampleLines();
    public Chambers Chambers = new Chambers();
    public Nets Nets = new Nets();
    public Observations Observations = new Observations();
    public Ascents Ascents = new Ascents();
    public Recalibration Recalibration = new Recalibration();

    private static CompBotVars vars = new CompBotVars();

    private static final int f = 144;

    public static final class SampleLines {
        private static final double x1 = 23.5;
        private static final double x2 = 13.5;
        private static final double x3 = 3.5;
        private static final double y = 46.25;

        public Blue Blue = new Blue();
        public Red Red = new Red();

        public static final class Blue {
            public double pushApproachAngle = Math.toRadians(90);

            public B1 B1 = new B1();
            public B2 B2 = new B2();

            public static final class B1 {
                public Pose Sample = new Pose(x1, f - y);

                public Pose Slip = new Pose(30, f - 18);
                public Pose Pre = new Pose(33, f - 24);
                public Pose Post = new Pose(36, f - 60);
            }

            public static final class B2 {
                public Pose Sample = new Pose(x2, f - y);

                public Pose Slip = new Pose(20, f - 18);
                public Pose Pre = new Pose(23, f - 24);
                public Pose Post = new Pose(26, f - 60);
            }
        }

        public static final class Red {
            public double pushApproachAngle = Math.toRadians(-90);

            public R1 R1 = new R1();
            public R2 R2 = new R2();

            public static final class R1 {
                public Pose Sample = new Pose(f - x1, y);

                public Pose Slip = new Pose(f - 30, 18);
                public Pose Pre = new Pose(f - 33, 24);
                public Pose Post = new Pose(f - 36, 60);
            }

            public static final class R2 {
                public Pose Sample = new Pose(f - x2, y);

                public Pose Slip = new Pose(f - 20, 18);
                public Pose Pre = new Pose(f - 23, 24);
                public Pose Post = new Pose(f - 26, 60);
            }
        }
    }

    public static final class Chambers {
        private static final double x = 72;
        private static final double y = 48;

        public Pose Blue = new Pose(x, f - y, Math.toRadians(-90));
        public Pose Red = new Pose(x, y, Math.toRadians(90));

        public Retreats Retreats = new Retreats();
        public Midpoints Midpoints = new Midpoints();

        public static final class Retreats {
            private static final double x = 72;
            private static final double y = 42;

            public Pose Blue = new Pose(x, f - y, Math.toRadians(-90));
            public Pose Red = new Pose(x, y, Math.toRadians(90));
        }

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

        public Pose Blue = new Pose(f - x, f - y, Math.toRadians(45));
        public Pose Red = new Pose(x, y, Math.toRadians(-135));
    }

    public static final class Observations {
        private static final double x = 12;
        private static final double y = 14;

        public Approaches Approaches = new Approaches();
        public Grabs Grabs = new Grabs();
        public Retreats Retreats = new Retreats();

        public Pose Blue = new Pose(x, f - y, Math.toRadians(90));
        public Pose Red = new Pose(f - x, y, Math.toRadians(-90));

        //NOTE: X value for Approaches and Grabs must be same in order for offset calculation to work in auto
        public static final class Approaches {
            private static final double y = 24;
            public Pose Blue = new Pose(x, f - y, Math.toRadians(90));
            public Pose Red = new Pose(f - x, y, Math.toRadians(-90));
        }

        public static final class Grabs {
            private static final double y = 8;
            public Pose Blue = new Pose(x, f - y, Math.toRadians(90));
            public Pose Red = new Pose(f - x, y, Math.toRadians(-90));
        }

        public static final class Retreats {
            private static final double y = 24;
            public Pose Blue = new Pose(x, f - y, Math.toRadians(90));
            public Pose Red = new Pose(f - x, y, Math.toRadians(-90));
        }
    }

    public static final class Ascents {
        public static final double x = 57;
        public static final double y = 72;

        private static final double offset = 12;

        public Blue Blue = new Blue();
        public Red Red = new Red();
        public Pre Pre = new Pre();
        public Post Post = new Post();

        public static final class Blue {
            public Pose Left = new Pose(f - x, y - offset, Math.toRadians(180));
            public Pose Right = new Pose(f - x, y + offset, Math.toRadians(180));
        }

        public static final class Red {
            public Pose Left = new Pose(x, y + offset, Math.toRadians(0));
            public Pose Right = new Pose(x, y - offset, Math.toRadians(0));
        }

        public static final class Pre {
            private static final double x = 24;
            private static final double y = 24;

            public Pose Blue = new Pose(f - x, f - y);
            public Pose Red = new Pose(x, y);
        }

        public static final class Post {
            private static final double x = 24;
            private static final double y = 72;

            public Pose Blue = new Pose(f - x, f - y);
            public Pose Red = new Pose(x, y);
        }
    }

    public static final class Recalibration {
        public Single Single = new Single();
        public Double Double = new Double();

        public static final class Single {
            public Pose A11 = new Pose(24, 120, Math.toRadians(-180));
            public Pose A12 = new Pose(72, 120, Math.toRadians(90));
            public Pose A13 = new Pose(108, 120, Math.toRadians(0));
            public Pose A14 = new Pose(108, 24, Math.toRadians(0));
            public Pose A15 = new Pose(72, 24, Math.toRadians(-90));
            public Pose A16 = new Pose(24, 24, Math.toRadians(180));
        }

        public static final class Double {

        }
    }
}