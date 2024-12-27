//Shortest file in this repo lol
package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Outgrabber.Outgrabber;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;

@TeleOp(name = "B - Outgrabber Tester", group = "testers")
public class OutgrabberTester extends OpMode {

    public CompBot Bot = new CompBot();

    Outgrabber outgrabber = new Outgrabber();



    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        outgrabber.initGrabber(hardwareMap);
    }

    @Override
    public void loop() {
        grabberControl();
        telemetry();
    }

    public void grabberControl() {
        if (gamepad1.a) {
            outgrabber.goOpen();
        } else if (gamepad1.b) {
            outgrabber.goClose();
        }
    }

    public void telemetry() {
        telemetry.addData("Grabber Position: ", outgrabber.outgrabber.getPosition());
        telemetry.update();
    }

}
