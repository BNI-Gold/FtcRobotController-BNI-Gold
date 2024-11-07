//Shortest file in this repo lol
package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Intake.IntakeDirections;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;

@TeleOp(name = "B - Intake Tester")
public class IntakeTester extends OpMode {

    public CompBot Bot = new CompBot();

    Intake intake = new Intake();



    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        intake.initIntake(hardwareMap);
        intake.center();
    }

    @Override
    public void loop() {
        intakeControl();
        telemetry();
    }

    public void intakeControl() {
        if (gamepad1.dpad_right) intake.rotateRight();
        else if (gamepad1.dpad_left) intake.rotateLeft();

        if (gamepad1.a) intake.intakeUntilSample();
        else if (gamepad1.x) intake.dropSample();
        else if (gamepad1.b) intake.stop();

        if (gamepad1.y) intake.center();
    }

    public void telemetry() {
        telemetry.addData("Sensor Distance: ", intake.sensor.getDistance(DistanceUnit.INCH));
        telemetry.addLine();
        telemetry.addData("Sensor Color - Red: ", intake.sensor.getNormalizedColors().red);
        telemetry.addData("Sensor Color - Blue: ", intake.sensor.getNormalizedColors().blue);
        telemetry.addData("Sensor Color - Green", intake.sensor.getNormalizedColors().green);
        telemetry.addLine();
        telemetry.addData("Intake Rotation Position: ", intake.rotator.getPosition());

        telemetry.update();
    }

}
