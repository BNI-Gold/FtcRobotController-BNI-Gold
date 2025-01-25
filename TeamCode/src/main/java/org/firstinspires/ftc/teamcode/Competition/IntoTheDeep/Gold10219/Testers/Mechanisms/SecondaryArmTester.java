package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.SecondaryArm.SecondaryArm;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;

@Disabled
@TeleOp(name = "B - Secondary Arm Tester", group = "testers")
public class SecondaryArmTester extends OpMode {

    public CompBot Bot = new CompBot();
    public SecondaryArm arm = new SecondaryArm();

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        arm.initSecondaryArm(hardwareMap, Bot.LinearOp);
    }

    double speedMultiplier = 1;

    @Override
    public void loop() {
        speedControl();
        primaryArmControl();
        telemetry();
    }

    public void speedControl() {
        if (gamepad1.a) {
            speedMultiplier = 0.35;
        } else {
            speedMultiplier = 1;
        }
    }

    public void primaryArmControl() {
        if (gamepad1.right_bumper) arm.extend();
        else if (gamepad1.left_bumper) arm.retract();
    }

    public void telemetry() {
        double extenderPos = arm.extender.getPosition();
        telemetry.addData("Extender Position: ", extenderPos);

        telemetry.update();
    }

}
