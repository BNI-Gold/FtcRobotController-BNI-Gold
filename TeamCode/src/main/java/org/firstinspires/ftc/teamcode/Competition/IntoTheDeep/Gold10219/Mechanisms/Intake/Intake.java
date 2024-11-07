package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Intake;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    public HardwareMap hwBot = null;
    public CRServo intake = null;
    public Servo rotator = null;
    public RevColorSensorV3 sensor = null;

//    IndicatorStrip indicator = new IndicatorStrip();

    double intakePower = 1;
    double intakeRotatorCenter = 0.3494;
    double intakeRotatorStep = 0.001;
    double sampleSecuredDistance = 1.45;

    public boolean isCollecting = false;
    public boolean isDropping = false;

    double position = 0.5;

    public Intake() {}

    public void initIntake(HardwareMap hwMap) {
        hwBot = hwMap;

        intake = hwBot.crservo.get("intake");
        rotator = hwBot.servo.get("intake_rotator");

        intake.setDirection(CRServo.Direction.FORWARD);
        rotator.setDirection(Servo.Direction.REVERSE);

        sensor = hwBot.get(RevColorSensorV3.class, "sample_sensor");

//        indicator.initIndicatorStrip(hwBot);
    }

    public void start(IntakeDirections direction) {
        switch (direction) {
            case IN:
                intake.setPower(intakePower);
                break;
            case OUT:
                intake.setPower(-intakePower);
                break;
        }
    }

    public void stop() {
        intake.setPower(0);
    }

    public void intakeUntilSample() {
        isDropping = false;

        double distance = sensor.getDistance(DistanceUnit.INCH);
        if (distance > sampleSecuredDistance) {
            start(IntakeDirections.IN);
            isCollecting = true;
        } else {
            stop();
            isCollecting = false;

            calcIntake();
        }
//
//        while (distance > sampleSecuredDistance && Op) {
//            start(IntakeDirections.IN);
//            isCollecting = true;
//            if (distance <= sampleSecuredDistance) break;
//        }
//        stop();
    }

    public void calcIntake() {
//        NormalizedRGBA colors = sensor.getNormalizedColors();
//        double red = colors.red;
//        double green = colors.green;
//        double blue = colors.blue;
//
//        RevBlinkinLedDriver.BlinkinPattern color;
//
//        double max = Math.max(red, Math.max(green, blue));
//
//        if (max == red) color = RevBlinkinLedDriver.BlinkinPattern.RED;
//        else if (max == blue) color = RevBlinkinLedDriver.BlinkinPattern.BLUE;
//        //TODO: Add calc for yellow
//        else color = RevBlinkinLedDriver.BlinkinPattern.BLACK;
//
//        indicator.capture(color);
    }

    public void dropSample() {
        isCollecting = false;

        double distance = sensor.getDistance(DistanceUnit.INCH);
        if (distance < sampleSecuredDistance) {
            start(IntakeDirections.OUT);
            isDropping = true;
        } else {
            stop();
            isDropping = false;
        }
    }

    public void center() {
        rotator.setPosition(intakeRotatorCenter);
    }

    public void rotateRight() {
        position = rotator.getPosition();
        rotator.setPosition(position + intakeRotatorStep);
    }

    public void rotateLeft() {
        position = rotator.getPosition();
        rotator.setPosition(position - intakeRotatorStep);
    }
}
