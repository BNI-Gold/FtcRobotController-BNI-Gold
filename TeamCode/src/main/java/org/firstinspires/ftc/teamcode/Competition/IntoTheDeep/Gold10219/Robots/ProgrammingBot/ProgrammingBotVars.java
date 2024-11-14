package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.ProgrammingBot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ProgrammingBotVars {

    public Motors Motors = new Motors();
    public IMUDirections IMU = new IMUDirections();
    public LLDimensions LL = new LLDimensions();

    public ProgrammingBotVars() {
    }

    public static final class Motors {

        public Motor FrontLeft = new Motor("front_left_motor", DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        public Motor FrontRight = new Motor("front_right_motor", DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        public Motor RearLeft = new Motor("rear_left_motor", DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        public Motor RearRight = new Motor("rear_right_motor", DcMotor.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);

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

    public static final class LLDimensions {
        public double width = 0;
        public double length = 0;
        public double forward = 0;
        public double right = 0;
        public double up = 0;
    }
}