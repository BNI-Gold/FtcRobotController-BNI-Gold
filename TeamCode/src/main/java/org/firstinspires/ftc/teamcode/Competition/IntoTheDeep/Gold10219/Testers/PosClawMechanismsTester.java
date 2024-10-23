package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;

@TeleOp(name = "B - Claw Mechanisms Tester")
public class PosClawMechanismsTester extends OpMode {

    public CompBot Bot = new CompBot();

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
    }

    double primaryExtenderJump = .0005;
    double secondaryExtenderJump = .0005;
    double clawJump = .0005;

    @Override
    public void loop() {
        control();
        telemetry();
    }

    public void control() {
        double primaryExtenderPos = Bot.primaryExtender.getPosition();
        double secondaryExtenderPos = Bot.secondaryExtender.getPosition();
        double clawPos = Bot.claw.getPosition();
        if (gamepad1.right_bumper) {
            Bot.primaryExtender.setPosition(primaryExtenderPos + primaryExtenderJump);
        } else if (gamepad1.left_bumper) {
            Bot.primaryExtender.setPosition(primaryExtenderPos - primaryExtenderJump);
        }
        if (gamepad1.a) { //Should be bottom button
            Bot.secondaryExtender.setPosition(secondaryExtenderPos + secondaryExtenderJump);
        } else if (gamepad1.y) { //Should be top button
            Bot.secondaryExtender.setPosition(secondaryExtenderPos - secondaryExtenderJump);
        }
        if (gamepad1.x) { //Should be left button
            Bot.claw.setPosition(clawPos + clawJump);
        } else if (gamepad1.b) { //Should be right button
            Bot.claw.setPosition(clawPos - clawJump);
        }
        if (gamepad1.dpad_up) {
            Bot.claw.setPosition(0);
            Bot.primaryExtender.setPosition(0);
            Bot.secondaryExtender.setPosition(0);
        }
    }

    public void telemetry() {
        telemetry.addData("Primary Extender Position: ", Bot.primaryExtender.getPosition());
        telemetry.addData("Secondary Extender Position: ", Bot.secondaryExtender.getPosition());
        telemetry.addData("Claw Position: ", Bot.claw.getPosition());
        telemetry.update();
    }

}
