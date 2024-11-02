package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.positions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.AutoBlueAlliance;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.AutoRedAlliance;

@Disabled
@Autonomous(name = "Blue:Net:DropSample:Park")
public class BlueNetSamplePark extends AutoBlueAlliance {

    @Override
    public void runOpMode() throws InterruptedException {

        // Global Method for Initializing Auto
        autoStart();

        waitForStart();
        //wait for player to press start
        while (opModeIsActive()) {

            // START AUTO PATH SEQUENCE

            Bot.strafeRight(0.5, 1);
            sleep(1000);
            //stop for 1000ms=1sec
            Bot.driveForward(0.5, 11.5);
            sleep(1000);
          // put the claw mechanism here

            // Park in Observation
            Bot.driveBack(0.5,34.5);

            // END AUTO PATH SEQUENCE
            requestOpModeStop();

        }
        idle();
    }
}