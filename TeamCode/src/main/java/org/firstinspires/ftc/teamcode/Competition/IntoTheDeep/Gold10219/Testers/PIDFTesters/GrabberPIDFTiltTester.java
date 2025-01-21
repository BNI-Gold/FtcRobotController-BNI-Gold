package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.PIDFTesters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Grabber.Grabber;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;

@TeleOp(name = "B - Grabber PIDF Tilt Tester", group = "testers")
public class GrabberPIDFTiltTester extends OpMode {

    public CompBot Bot = new CompBot();
    Grabber grabber = new Grabber();

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        grabber.initGrabber(hardwareMap);
    }

    @Override
    public void loop() {
        grabber.tiltStateCheck();
        tester();
        sendTelemetry();
    }

    public void tester() {
        if (gamepad1.a) {
            grabber.setGrabberState(Grabber.grabberStates.TUCK);
        } else if (gamepad1.b) {
            grabber.setGrabberState(Grabber.grabberStates.HOOK);
        } else if (gamepad1.x) {
            grabber.setGrabberState(Grabber.grabberStates.DOWN);
        } else if (gamepad1.y) {
            grabber.setGrabberState(Grabber.grabberStates.OUT);
        }
    }

    public void sendTelemetry() {
        telemetry.addData("Target Angle", grabber.targ);
        telemetry.addData("Current Angle", grabber.ang);
        telemetry.addData("Error", grabber.diff);
        telemetry.addData("Servo Position", grabber.grabber.getPosition());
        telemetry.addLine();
        telemetry.addLine("PIDF Output");
        telemetry.addLine(">> " + grabber.getPIDFValues());
        telemetry.update();
    }
}