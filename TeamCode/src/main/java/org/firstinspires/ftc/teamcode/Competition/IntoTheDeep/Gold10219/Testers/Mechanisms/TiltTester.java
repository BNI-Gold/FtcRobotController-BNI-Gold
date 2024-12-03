//Shortest file in this repo lol
package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Grabber.Grabber;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;

@TeleOp(name = "B - Tilt Tester", group = "testers")
public class TiltTester extends OpMode {

    public CompBot Bot = new CompBot();

    Grabber grabber = new Grabber();



    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        grabber.initGrabber(hardwareMap);
        grabber.doTuck();
    }

    @Override
    public void loop() {
        tiltControl();
        telemetry();
        grabber.tiltStateCheck();
    }

    private double lastTiltAngle = 0;

    public void tiltControl() {
        if (gamepad1.dpad_up) {
            grabber.tiltUp();
        } else if (gamepad1.dpad_down) {
            grabber.tiltDown();
        } else if (gamepad1.a) {
            grabber.setCurrentState(Grabber.grabberStates.OUT);
        } else if (gamepad1.b) {
            grabber.setCurrentState(Grabber.grabberStates.DOWN);
        } else if (gamepad1.y) {
            grabber.setCurrentState(Grabber.grabberStates.CONTROL);
        }
    }

    public void telemetry() {
        telemetry.addData("Current IMU Angle: ", grabber.getTilt());
        telemetry.addData("Servo Position: ", grabber.tilt.getPosition());
        telemetry.addLine();
        telemetry.addData("IMU Angle: ", grabber.ang);
        telemetry.addData("Diff: ", grabber.diff);
        telemetry.addData("Normalized Diff: ", grabber.diff1);
        telemetry.addData("Pos Ch: ", grabber.pch);
        telemetry.addData("Current Servo Position: ", grabber.csp);
        telemetry.addData("New Servo Position: ", grabber.nsp);
        telemetry.addData("Clamped Servo Position: ", grabber.nsp2);
        telemetry.update();
    }

}
