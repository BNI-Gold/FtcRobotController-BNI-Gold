//Shortest file in this repo lol
package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Grabber.Grabber;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;

@TeleOp(name = "B - Grabber Tester", group = "testers")
public class GrabberTester extends OpMode {

    public CompBot Bot = new CompBot();

    Grabber grabber = new Grabber();



    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        grabber.initGrabber(hardwareMap);
    }

    @Override
    public void loop() {
        grabberControl();
        telemetry();
    }

    public void grabberControl() {
        if (gamepad1.a) {
            grabber.goOpen();
        } else if (gamepad1.b) {
            grabber.goClose();
        }
    }

    public void telemetry() {
        telemetry.addData("Grabber Position: ", grabber.grabber.getPosition());
        telemetry.update();
    }

}
