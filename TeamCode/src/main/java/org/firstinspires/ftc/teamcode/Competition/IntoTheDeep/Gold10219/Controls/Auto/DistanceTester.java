package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "DistanceTester")
public class DistanceTester extends AutoMain{

    @Override
    public void runOpMode() throws InterruptedException {

        //Global Method for Initializing Auto
        autoStart();

        //wait for player to press start
        waitForStart();
        while (opModeIsActive()) {

            //START AUTO PATH SEQUENCE
            Bot.strafeRight(0.5, 1);
            sleep(500);
            Bot.strafeLeft(0.5, 0.5);
            sleep(1000);

            Bot.driveForward(0.5, 1);
            sleep(500);
            Bot.driveBack(0.5, 0.5);
        }
    }
}
