package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto;

public abstract class  AutoBlueAlliance extends AutoMain {

    // Helper Method to Drop Sample in Net Area
    public void dropSample() {
        Bot.strafeRight(0.5, 1);
        sleep(1000);
        Bot.driveForward(0.5, 11.5);
        sleep(1000);
        // put the claw mechanism here
    }

    // Helper Method to Park in Observation
    public void parkObservation() {

    }

    // Helper Method to Touch Bar
    public void touchBar() {

    }


}
