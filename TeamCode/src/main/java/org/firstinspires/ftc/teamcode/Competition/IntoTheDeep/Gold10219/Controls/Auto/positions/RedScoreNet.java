package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.positions;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.AutoBlueAlliance;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.AutoRedAlliance;

public class RedScoreNet extends AutoRedAlliance {
    public void runOpMode() throws InterruptedException {

        Bot.strafeRight(0.5,1);
        sleep(1000);
        Bot.stopMotors();
        Bot.driveForward(0.5,48.2);
        Bot.stopMotors();
        Bot.strafeLeft(0.5,5.5);
        Bot.rotateRight(5,180);
        Bot.stopMotors();
        Bot.driveForward(0.5,24.1);
        Bot.rotateLeft(5,90);

    }
}
