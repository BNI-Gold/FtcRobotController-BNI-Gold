package org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.ComptitionAutoPaths.BlueLeftPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.Robots.CompetionBot;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.ComptitionAutoPaths.AutoMain;
@Disabled
@Autonomous (name = "BlueTerminalPark - DONT USE")

public class BlueTerminalPark extends AutoMain {

    public CompetionBot Bot = new CompetionBot();

    @Override
    public void runOpMode() throws InterruptedException {
        Bot.initRobot(hardwareMap);

        Bot.setLinearOp(this);

        waitForStart();

        while (opModeIsActive()) {

            //competition mechanism start

            Bot.closeGrabberArms();

            sleep(1000);

            Bot.extendGrabberLift(0.5);

            sleep(200);

            Bot.stopGrabberLift();

            sleep(sleepTime);

            //end competition mechanism start

            Bot.driveForward(0.1, 0.1);
            sleep(sleepTime);

            Bot.strafeRight(0.4, 3);
            sleep(sleepTime);

            idle();
            requestOpModeStop();

        }

    }

}
