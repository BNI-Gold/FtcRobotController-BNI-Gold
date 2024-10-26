package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.positions;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.AutoBlueAlliance;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.extenderDirections;

public class BlueScoreNet extends AutoBlueAlliance {
    public void runOpMode() throws InterruptedException {
waitForStart();
        Bot.strafeRight(0.5,1);
        sleep(1000);
        Bot.stopMotors();
        Bot.driveForward(0.5,1);
        Bot.stopMotors();
        Bot.useSecondaryExtender(true, extenderDirections.RETRACT);;

    }
}
