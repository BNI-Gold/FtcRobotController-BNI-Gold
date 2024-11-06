//Shortest file in this repo lol
package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;

@TeleOp(name = "B - Intake Tester")
public class IntakeTester extends OpMode {

    public CompBot Bot = new CompBot();

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
    }

    @Override
    public void loop() {
        intakeControl();
        telemetry();
    }

    Intake intake = new Intake(Bot);

    public void intakeControl() {
        if (gamepad1.dpad_right) intake.rotateRight();
        else if (gamepad2.dpad_left) intake.rotateLeft();
    }

    public void telemetry() {
        telemetry.addData("Sensor Distance: ", intake.sensor.getDistance(DistanceUnit.INCH));
        telemetry.addLine();
        telemetry.addData("Sensor Color - Red: ", intake.sensor.getNormalizedColors().red);
        telemetry.addData("Sensor Color - Blue: ", intake.sensor.getNormalizedColors().blue);
        telemetry.addData("Sensor Color - Green", intake.sensor.getNormalizedColors().green);
        telemetry.addLine();
        telemetry.addData("Intake Rotation Position: ", Bot.intakeRotator.getPosition());

        telemetry.update();
    }

}
