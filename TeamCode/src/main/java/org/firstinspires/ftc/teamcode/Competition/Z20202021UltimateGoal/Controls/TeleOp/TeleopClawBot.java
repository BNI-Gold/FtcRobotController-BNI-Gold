package org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Robots.ClawBot;
@Disabled
@TeleOp (name = "Teleop_ClawBot", group = "3")
//@Disabled

public class TeleopClawBot extends OpMode {
    public ClawBot Bot = new ClawBot();

    @Override
    public void init() {
        Bot.initRobot(hardwareMap, "TeleOp", "TeleOp");

    }

    @Override
    public void init_loop() {
    }
    @Override
    public void loop() {

        clawMovement();
        clawControl();
    }

    public void clawMovement(){
        if (gamepad1.left_stick_y > 0.1){
            Bot.ClawForward(0.2);
        }
        else if (gamepad1.left_stick_y < -0.1){
            Bot.ClawBackward(0.2);
        }
        else{
            Bot.ClawBackward(0);
            Bot.ClawForward(0);
        }
        if (gamepad1.left_stick_x < -0.1){
            Bot.ClawLeftSide(0.2);
        }
        else if (gamepad1.left_stick_x > 0.1){
            Bot.ClawRightSide(0.2);
        }
        else{
            Bot.ClawLeftSide(0);
            Bot.ClawRightSide(0);
        }
        if (gamepad1.right_stick_y > 0.1){
            Bot.ClawUp(0.2);
        }
        else if (gamepad1.right_stick_y < -0.1){
            Bot.ClawDown(0.2);
        }
        else{
            Bot.ClawUp(0);
            Bot.ClawDown(0);
        }
    }

    public void clawControl() {
        if (gamepad1.left_bumper) {
            Bot.ClawOpen();
        }
        if (gamepad1.right_bumper) {
            Bot.ClawClose();
        }
    }
}
