//Shortest file in this repo lol
package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Outgrabber.Outgrabber;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;

@Disabled
@TeleOp(name = "B - Outgrabber Tilt Tester", group = "testers")
public class OutgrabberTiltTester extends OpMode {

    public CompBot Bot = new CompBot();

    Outgrabber outgrabber = new Outgrabber();



    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        outgrabber.initOutgrabber(hardwareMap);
    }

    @Override
    public void loop() {
        tiltControl();
        telemetry();
    }

    public void tiltControl() {
        if (gamepad1.dpad_up) {
            outgrabber.tiltUp();
        } else if (gamepad1.dpad_down) {
            outgrabber.tiltDown();
        }
    }

    public void telemetry() {
        telemetry.addData("Servo Position: ", outgrabber.tilt.getPosition());
        telemetry.update();
    }

}
