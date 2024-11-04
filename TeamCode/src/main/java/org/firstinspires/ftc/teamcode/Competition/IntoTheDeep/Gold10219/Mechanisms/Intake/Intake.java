package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;

public class Intake {
    public CompBot Bot;
    private LinearOpMode LinearOp;
    public CRServo intake;
    public Servo rotator;
    public ColorRangeSensor sensor;

    double intakePower = 1;
    double intakeRotatorCenter = 0;
    double intakeRotatorStep = 0.0005;
    double sampleSecuredDistance = 0;

    public Intake(CompBot Bot, LinearOpMode LinearOp) {
        this.Bot = Bot;
        this.LinearOp = LinearOp;
        intake = Bot.intake;
        rotator = Bot.intakeRotator;
        sensor = Bot.sampleSensor1;
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
        double distance = sensor.getDistance(DistanceUnit.INCH);
        while (distance > sampleSecuredDistance) {
            start(IntakeDirections.IN);
        }
        stop();
    }

    public void dropSample() {
        double distance = sensor.getDistance(DistanceUnit.INCH);
        while (distance < sampleSecuredDistance) {
            start(IntakeDirections.OUT);
        }
        stop();
    }

    public void center() {
        rotator.setPosition(intakeRotatorCenter);
    }

    public void rotateRight() {
        double position = rotator.getPosition();
        rotator.setPosition(position + intakeRotatorStep);
    }

    public void rotateLeft() {
        double position = rotator.getPosition();
        rotator.setPosition(position - intakeRotatorStep);
    }
}
