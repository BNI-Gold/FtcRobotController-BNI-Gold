package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Alliances.Blue;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Intake.IntakeDirections;

public abstract class  AutoBlueAlliance extends AutoMain {

    // Helper Method to Drop Sample in Net Area
    public void dropSampleAndRetreat() {
        intake.start(IntakeDirections.OUT);
        sleep(1000);
        intake.stop();
        sleep(500);
        Bot.driveBack(0.5, 0.5);
        sleep(500);
        Bot.rotateRight(0.5, 1);
        sleep(500);
        Bot.strafeLeft(0.5, 1.25);
        sleep(500);
        Bot.driveBack(0.5, 0.4);
        sleep(500);
        arm.retract(.75, 5.1);
    }

    //Helper Method to Raise, Extend, and finish Raising Arm
    public void safeExtendAndRaise() {
        arm.up(3, false);
        sleep(500);
        //stop for 1000ms=1sec
        arm.extend(.75, 5.1);
        sleep(500);
        arm.up(5.5, true);
    }
}
