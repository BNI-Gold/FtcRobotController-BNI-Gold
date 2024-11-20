package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Vars;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.Point;

public class FieldPoints {
    public SampleLines SampleLines = new SampleLines();
    public Chambers Chambers = new Chambers();
    public Nets Nets = new Nets();
    public Observations Observations = new Observations();

    private static final int f = 144;

    public static final class SampleLines {
        public Audience Audience = new Audience();
        public Rear Rear = new Rear();

        private static final double x1 = 23.5;
        private static final double x2 = 13.5;
        private static final double x3 = 3.5;
        private static final double y = 46.25;

        public static final class Audience {
            public Point Blue1 = new Point(x1, f-y);
            public Point Blue2 = new Point(x2, f-y);
            public Point Blue3 = new Point(x3, f-y);

            public Point Neutral1 = new Point(x1, y);
            public Point Neutral2 = new Point(x2, y);
            public Point Neutral3 = new Point(x3, y);
        }

        public static final class Rear {
            public Point Neutral1 = new Point(f-x1, f-y);
            public Point Neutral2 = new Point(f-x2, f - y);
            public Point Neutral3 = new Point(f-x3, f - y);

            public Point Red1 = new Point(f-x1, y);
            public Point Red2 = new Point(f-x2, y);
            public Point Red3 = new Point(f-x2, y);
        }
    }

    public static final class Chambers {
        private static final double x = 72; // Adjusted for the rotated field
        private static final double y = 36;

        public Point Blue = new Point(x, f - y);
        public Point Red = new Point(x, y);
    }

    public static final class Nets {
        private static final double x = 12;
        private static final double y =  12;

        public Point Blue = new Point(f - x, f - y);
        public Point Red = new Point(x, y);
    }

    public static final class Observations {
        private static final double x = 12;
        private static final double y = 14;

        public Point Blue = new Point(x, f - y);
        public Point Red = new Point(f - x, y);
    }
}