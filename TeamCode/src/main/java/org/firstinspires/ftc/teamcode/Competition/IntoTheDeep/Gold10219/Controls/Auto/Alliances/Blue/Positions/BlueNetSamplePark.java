package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Alliances.Blue.Positions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Alliances.Blue.AutoBlueAlliance;

@Autonomous(name = "Blue:Net:DropSample:Park")
public class BlueNetSamplePark extends AutoBlueAlliance {

    @Override
    public void runOpMode() throws InterruptedException {

        // Global Method for Initializing Auto
        autoStart();

        //wait for player to press start
        waitForStart();
        while (opModeIsActive()) {

            // START AUTO PATH SEQUENCE
            Bot.strafeRight(0.5, 0.4);
            sleep(500);
            safeExtendAndRaise();
            sleep(500);
            Bot.driveForward(0.5, 4.5);
            sleep(500);
            Bot.rotateLeft(0.5, 0.8);
            sleep(500);
            Bot.strafeRight(0.5, 0.4);
            sleep(500);
            Bot.driveForward(.5, .5);
            sleep(500);
            dropSampleAndRetreat();

            // END AUTO PATH SEQUENCE
            requestOpModeStop();
        }
        idle();
    }
}