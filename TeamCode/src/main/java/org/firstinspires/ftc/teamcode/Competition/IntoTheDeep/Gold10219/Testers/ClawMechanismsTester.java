package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;

@TeleOp(name = "B - Claw Mechanisms Tester")
public class ClawMechanismsTester extends OpMode {

    public CompBot Bot = new CompBot();

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
    }

    double primaryExtenderJump = .05;
    double secondaryExtenderJump = .05;
    double clawJump = .05;

    @Override
    public void loop() {
        control();
        telemetry();
    }

    public void control() {
//        double primaryExtenderPos = Bot.primaryExtender.getPosition();
//        double secondaryExtenderPos = Bot.secondaryExtender.getPosition();
        //TODO: set extender powers
        double primaryExtenderPower = 0;
        double secondaryExtenderPower = 0;
        double clawPos = Bot.claw.getPosition();
//        if (gamepad1.right_bumper) {
//            Bot.primaryExtender.setPosition(primaryExtenderPos + primaryExtenderJump);
//        } else if (gamepad1.left_bumper) {
//            Bot.primaryExtender.setPosition(primaryExtenderPos - primaryExtenderJump);
//        }
//        if (gamepad1.a) { //Should be bottom button
//            Bot.secondaryExtender.setPosition(secondaryExtenderPos + secondaryExtenderJump);
//        } else if (gamepad1.y) { //Should be top button
//            Bot.secondaryExtender.setPosition(secondaryExtenderPos - secondaryExtenderJump);
//        }
        if (gamepad2.right_bumper) {
            Bot.primaryExtender.setPower(primaryExtenderPower);
        } else if (gamepad2.left_bumper) {
            Bot.primaryExtender.setPower(-primaryExtenderPower);
        } else {
            Bot.primaryExtender.setPower(0);
        }
        if (gamepad2.a) { //Should be bottom button
            Bot.secondaryExtender.setPower(secondaryExtenderPower);
        } else if (gamepad2.y) { //Should be top button
            Bot.secondaryExtender.setPower(-secondaryExtenderPower);
        } else {
            Bot.secondaryExtender.setPower(0);
        }
        if (gamepad1.x) { //Should be left button
            Bot.claw.setPosition(clawPos + clawJump);
        } else if (gamepad1.b) { //Should be right button
            Bot.claw.setPosition(clawPos - clawJump);
        }
    }

    public void telemetry() {
//        telemetry.addData("Primary Extender Position: ", Bot.primaryExtender.getPosition());
//        telemetry.addData("Primary Extender Position: ", Bot.primaryExtender.getPosition());
//        telemetry.addData("Secondary Extender Position: ", Bot.secondaryExtender.getPosition());
//        telemetry.addData("Secondary Extender Position: ", Bot.secondaryExtender.getPosition());
        telemetry.addData("Claw Position: ", Bot.claw.getPosition());
        telemetry.addData("Claw Position: ", Bot.claw.getPosition());
        telemetry.update();
    }

}
