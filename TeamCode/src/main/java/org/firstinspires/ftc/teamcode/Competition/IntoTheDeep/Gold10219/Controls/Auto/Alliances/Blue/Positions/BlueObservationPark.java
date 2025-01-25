package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Alliances.Blue.Positions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Alliances.Blue.AutoBlueAlliance;

@Disabled
@Autonomous(name = "Blue:Observation:Park", group = "blue")
public class BlueObservationPark extends AutoBlueAlliance {

    @Override
    public void runOpMode() throws InterruptedException {

        //Global Method for Initializing Auto
        autoStart();

        //wait for player to press start
        waitForStart();
        while(opModeIsActive()) {

            // START AUTO PATH SEQUENCE
            Bot.strafeRightInches(0.5, 4.8);
            sleep(500);
            Bot.driveBackInches(0.5, 66);

            // END AUTO PATH SEQUENCE
            requestOpModeStop();
        }
        idle();
    }
}
