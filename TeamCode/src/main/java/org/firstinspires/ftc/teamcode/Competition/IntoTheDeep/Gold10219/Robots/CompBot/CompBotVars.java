package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

public class CompBotVars {

    public Motors Motors = new Motors();
    public IMUDirections IMU = new IMUDirections();
    public Chassis Chassis = new Chassis();
    public Mechanisms Mechanisms = new Mechanisms();

    public CompBotVars() {
    }

    public double sample = 3;

    public static final class Motors {

        public Motor FrontLeft = new Motor("front_left_motor", DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        public Motor FrontRight = new Motor("front_right_motor", DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        public Motor RearLeft = new Motor("rear_left_motor", DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        public Motor RearRight = new Motor("rear_right_motor", DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);

        public static final class Motor {
            public String name;
            public DcMotor.Direction direction;
            public DcMotor.ZeroPowerBehavior zeroPowerBehavior;

            public Motor(String name, DcMotor.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
                this.name = name;
                this.direction = direction;
                this.zeroPowerBehavior = zeroPowerBehavior;
            }
        }
    }

    public static final class IMUDirections {
        private final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        private final RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        public RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);
    }

    public static final class Chassis {
        public double FRONT_LENGTH = 8;
        public double BACK_LENGTH = 9;
        public double WIDTH = 16;
    }

    public static final class Mechanisms {
        public Grabber Grabber = new Grabber();
        public static final class Grabber {
            public AtChambers AtChambers = new AtChambers();
            public AtObservation AtObservation = new AtObservation();

            public static final class AtChambers {
                public double GRABBER_RETRACTED_POSITION = 2;
                public double OUT = 1;
            }

            public static final class AtObservation {
                public double OUT = 7.5;
            }
        }
    }
}