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

        private static final double x = 46.25;
        private static final double y1 = 120.5; // Updated y values to match new field orientation
        private static final double y2 = 130.5;
        private static final double y3 = 140.5;

        public static final class Audience {
            public Point Blue1 = new Point(x, y1);
            public Point Blue2 = new Point(x, y2);
            public Point Blue3 = new Point(x, y3);

            public Point Neutral1 = new Point(f - x, y1);
            public Point Neutral2 = new Point(f - x, y2);
            public Point Neutral3 = new Point(f - x, y3);
        }

        public static final class Rear {
            public Point Neutral1 = new Point(x, f - y1);
            public Point Neutral2 = new Point(x, f - y2);
            public Point Neutral3 = new Point(x, f - y3);

            public Point Red1 = new Point(f - x, f - y1);
            public Point Red2 = new Point(f - x, f - y2);
            public Point Red3 = new Point(f - x, f - y3);
        }
    }

    public static final class Chambers {
        private static final int x = 72; // Adjusted x to match updated orientation
        private static final int y = 48; // Adjusted y to match updated orientation

        public Point Blue = new Point(x, f - y);
        public Point Red = new Point(f - x, y);
    }

    public static final class Nets {
        private static final int x = 12;
        private static final int y = 12;

        public Point Blue = new Point(f - y, x);
        public Point Red = new Point(y, f - x);
    }

    public static final class Observations {
        private static final int x = 12; // Adjusted x to match updated orientation
        private static final int y = 14; // Adjusted y to match updated orientation

        public Point Blue = new Point(f - y, x);
        public Point Red = new Point(y, f - x);
    }
}