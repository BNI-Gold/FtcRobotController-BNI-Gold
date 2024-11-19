package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

public class CompBotVars {

    public Motors Motors = new Motors();
    public IMUDirections IMU = new IMUDirections();
    public Chassis Chassis = new Chassis();

    public CompBotVars() {
    }

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
        public double ROBOT_FRONT_LENGTH = 0;
        public double ROBOT_BACK_LENGTH = 0;
        public double ROBOT_WIDTH = 0;
    }

    public static final class Mechanisms {
        public Arm Arm = new Arm();
        public static final class Arm {
            public double ARM_RETRACTED_POSITION = 0;
            public double ARM_EXTENDED_POSITION = 0;
        }
    }
}