package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.positions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.AutoBlueAlliance;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.AutoRedAlliance;

@Disabled
@Autonomous(name = "Red:Net:DropSample:TouchBar")
public class RedNetSampleBar extends AutoRedAlliance {

    @Override
    public void runOpMode() throws InterruptedException {
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
            Bot.driveBack(0.5,5.75);
            Bot.strafeRight(0.8,20.125);
            Bot.rotateLeft(0.5,0.5);
            //turn 180 degrees hopefully
            Bot.driveForward(0.5,2.875);
            //extend claw here

            // END AUTO PATH SEQUENCE
            requestOpModeStop();

        }
        idle();
    }
}