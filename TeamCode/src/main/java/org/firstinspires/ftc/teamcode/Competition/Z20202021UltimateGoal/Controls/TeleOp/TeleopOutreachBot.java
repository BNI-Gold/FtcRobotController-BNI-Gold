package org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Robots.OutreachBot;
@Disabled
@TeleOp (name = "Teleop_OutreachBot", group = "3")
//@Disabled
public class TeleopOutreachBot extends OpMode {
//    double leftStickYVal;
//    double leftStickXVal;
//    double rightStickXVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;
    double powerThreshold = 0;

    double speedMultiply = 1;
    boolean forwardMode = true;

    double leftJoyStickPos;
    double rightJoyStickPos;

    boolean reverseModeToggle = false;

    public OutreachBot Bot = new OutreachBot();

    @Override
    public void init() {
        Bot.initRobot(hardwareMap, "TeleOp", "TeleOp");

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void loop() {
        drive();
        driveMode();

    }

    public void driveMode() {
        if (gamepad1.dpad_right) {
            reverseModeToggle = true;
        }
        if (gamepad1.dpad_left) {
            reverseModeToggle = false;
        }

    }
    public void drive() {
        leftJoyStickPos = gamepad1.left_stick_y;
        rightJoyStickPos = gamepad1.right_stick_y;
        Bot.setMotorSpeeds(leftJoyStickPos, rightJoyStickPos);
        if (gamepad1.b == true){
            Bot.stopMotors(0,0);
        }
    }
}
