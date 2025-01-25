package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Alliances.Red.Positions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Alliances.Blue.AutoBlueAlliance;

@Disabled
@Autonomous(name = "Red:Net:Park", group = "blue")
public class RedNetPark extends AutoBlueAlliance {

    @Override
    public void runOpMode() throws InterruptedException {

        // Global Method for Initializing Auto
        autoStart();

        //wait for player to press start
        waitForStart();
        while (opModeIsActive()) {

            // START AUTO PATH SEQUENCE
            Bot.strafeRightInches(0.5, 4.8);
            sleep(500);
            Bot.driveForwardInches(0.5, 54);
            sleep(500);
            Bot.strafeLeftInches(0.5, 4.8);

            // END AUTO PATH SEQUENCE
            requestOpModeStop();
        }
        idle();
    }
}

