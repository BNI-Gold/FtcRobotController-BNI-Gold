//Shortest file in this repo lol
package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Outgrabber.Outgrabber;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;

@TeleOp(name = "B - Outgrabber Rotate Tester", group = "testers")
public class OutgrabberRotateTester extends OpMode {

    public CompBot Bot = new CompBot();

    Outgrabber outgrabber = new Outgrabber();



    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        outgrabber.initOutgrabber(hardwareMap);
    }

    @Override
    public void loop() {
        rotateControl();
        telemetry();
    }

    public void rotateControl() {
        if (gamepad1.right_bumper) {
            outgrabber.rotateRight();
        } else if (gamepad1.left_bumper) {
            outgrabber.rotateLeft();
        }
    }

    public void telemetry() {
        telemetry.addData("Rotate Angle: ", outgrabber.rotate.getPosition());
        telemetry.update();
    }

}
