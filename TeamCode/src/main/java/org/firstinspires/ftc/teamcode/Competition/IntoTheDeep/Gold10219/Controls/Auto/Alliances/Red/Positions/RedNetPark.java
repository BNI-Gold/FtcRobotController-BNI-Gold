package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Alliances.Red.Positions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Alliances.Red.AutoRedAlliance;

@Autonomous(name = "Red:Net:Park")
public class RedNetPark extends AutoRedAlliance {

    @Override
    public void runOpMode() throws InterruptedException {

        // Global Method for Initializing Auto
        autoStart();

        waitForStart();
        //wait for player to press start
        while (opModeIsActive()) {

            // START AUTO PATH SEQUENCE
            Bot.strafeRight(0.5, 0.4);
            sleep(500);
            Bot.driveForward(0.5, 4.5);
            sleep(500);
            Bot.strafeLeft(0.5, 0.4);

            // END AUTO PATH SEQUENCE
            requestOpModeStop();
        }
        idle();
    }
}

