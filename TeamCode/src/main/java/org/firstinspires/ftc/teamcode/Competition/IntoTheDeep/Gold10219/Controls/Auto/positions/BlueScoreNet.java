package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.positions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.AutoBlueAlliance;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.extenderDirections;

@Disabled
@Autonomous(name = "BlueScoreNet")
public class BlueScoreNet extends AutoBlueAlliance {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        //wait for player to press start
        while (opModeIsActive()) {

            // START AUTO PATH SEQUENCE

            Bot.strafeRight(0.5, 1);
            sleep(1000);
            //stop for 1000ms=1sec
            Bot.stopMotors();
            Bot.driveForward(0.5, 1);
            Bot.stopMotors();
            Bot.useSecondaryExtender(true, extenderDirections.RETRACT);

            // END AUTOPATH SEQUENCE

        }
        idle();
    }
}