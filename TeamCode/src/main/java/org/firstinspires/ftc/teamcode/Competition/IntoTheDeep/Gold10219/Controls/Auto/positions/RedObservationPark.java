package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.positions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.AutoRedAlliance;

@Autonomous(name = "Red:Observation:Park")
public class RedObservationPark extends AutoRedAlliance {

    @Override
    public void runOpMode() throws InterruptedException {

        //Global Method for Initializing Auto
        autoStart();

        waitForStart();
        //wait for player to press start
        while(opModeIsActive()) {

            // START AUTO PATH SEQUENCE
            Bot.strafeRight(0.5, 0.4);
            sleep(500);
            Bot.driveBack(0.5, 6);

            // END AUTO PATH SEQUENCE
            requestOpModeStop();
        }
    }
}
