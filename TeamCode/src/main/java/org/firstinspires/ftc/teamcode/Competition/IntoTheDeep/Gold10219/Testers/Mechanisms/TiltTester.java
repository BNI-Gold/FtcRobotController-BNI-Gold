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
    }

    @Override
    public void loop() {
        tiltControl();
        telemetry();
    }

    private double lastTiltAngle = 0;

    public void tiltControl() {
        if (gamepad1.dpad_up) {
            grabber.tiltUp();
        } else if (gamepad1.dpad_down) {
            grabber.tiltDown();
        } else if (gamepad1.a) {
            grabber.tiltToAngle(Grabber.grabberStates.OUT);
        } else if (gamepad1.b) {
            grabber.tiltToAngle(Grabber.grabberStates.DOWN);
        } else if (gamepad1.y) {
            grabber.tiltToAngle(Grabber.grabberStates.CONTROL);
        }
    }

    public void telemetry() {
        telemetry.addData("Current IMU Angle: ", grabber.getTilt());
        telemetry.addData("Servo Position: ", grabber.tilt.getPosition());
        telemetry.update();
    }

}
