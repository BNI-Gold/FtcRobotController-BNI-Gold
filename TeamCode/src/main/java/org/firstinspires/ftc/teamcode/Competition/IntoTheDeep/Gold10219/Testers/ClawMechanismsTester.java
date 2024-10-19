package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.Bot;

@TeleOp(name = "B - Claw Mechanisms Tester")
public class ClawMechanismsTester extends OpMode {

    public Servo primaryExtender = null;
    public Servo secondaryExtender = null;
    public Servo claw = null;

    public Bot Bot = new Bot();

    public void init() {
        Bot.initRobot(hardwareMap);
    }

    double primaryExtenderJump = .05;
    double secondaryExtenderJump = .05;
    double clawJump = .05;

    public void loop() {
        double primaryExtenderPos = Bot.primaryExtender.getPosition();
        double secondaryExtenderPos = Bot.secondaryExtender.getPosition();
        double clawPos = Bot.claw.getPosition();
        if (gamepad1.right_bumper) {
            Bot.primaryExtender.setPosition(primaryExtenderPos + primaryExtenderJump);
            telemetry.addData("Primary Extender Position: ", Bot.primaryExtender.getPosition());
        } else if (gamepad1.left_bumper) {
            Bot.primaryExtender.setPosition(primaryExtenderPos + primaryExtenderJump);
            telemetry.addData("Primary Extender Position: ", Bot.primaryExtender.getPosition());
        } else if (gamepad1.a) { //Should be bottom button
            Bot.secondaryExtender.setPosition(secondaryExtenderPos + secondaryExtenderJump);
            telemetry.addData("Secondary Extender Position: ", Bot.secondaryExtender.getPosition());
        } else if (gamepad1.y) { //Should be top button
            Bot.secondaryExtender.setPosition(secondaryExtenderPos - secondaryExtenderJump);
            telemetry.addData("Secondary Extender Position: ", Bot.secondaryExtender.getPosition());
        } else if (gamepad1.x) { //Should be left button
            Bot.claw.setPosition(clawPos + clawJump);
            telemetry.addData("Claw Position: ", Bot.claw.getPosition());
        } else if (gamepad1.b) { //Should be right button
            Bot.claw.setPosition(clawPos - clawJump);
            telemetry.addData("Claw Position: ", Bot.claw.getPosition());
        }
        telemetry.update();
    }

}
