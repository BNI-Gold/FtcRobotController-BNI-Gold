package org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.Archive.Lab;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Robots.LabBot;

@Autonomous(name = "Auto Roll Out")
@Disabled
public class AutoRollOut extends LinearOpMode {

    public LabBot Bot = new LabBot();

    public enum Square {A, B, C}


    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Bot.initRobot(hardwareMap);
        Bot.setLinearOp(this);
        Bot.servoClosed();
//        Call vision tracking Class - set wobbleTarget to correct Square

        Square wobbleTarget = Square.B;

        waitForStart();

        while (opModeIsActive()){

            switch (wobbleTarget) {
                case A:
                    Bot.driveForward(0.5,7.75);
                    sleep(500);
                    Bot.strafeRight(0.5,.5);
                    sleep(500);
                    Bot.servoOpened();
                    sleep(500);
                    Bot.servoClosed();
                    break;

                case B:
                    Bot.driveForward(.5,10.5);
                    sleep(500);
                    Bot.strafeRight(0.5,3.5);
                    sleep(500);
                    Bot.servoOpened();
                    sleep(500);
                    Bot.servoClosed();
                    sleep(500);
                    Bot.driveBackward(0.5, 2.5);

                    break;


                case C:
                    Bot.driveForward(.5,12.5);
                    sleep(500);
                    Bot.strafeRight(0.5,.5);
                    sleep(500);
                    Bot.servoOpened();
                    sleep(500);
                    Bot.servoClosed();
                    sleep(500);
                    Bot.driveBackward(0.5, 4.5);

                    break;

            }

         */
        requestOpModeStop();

    }
}
