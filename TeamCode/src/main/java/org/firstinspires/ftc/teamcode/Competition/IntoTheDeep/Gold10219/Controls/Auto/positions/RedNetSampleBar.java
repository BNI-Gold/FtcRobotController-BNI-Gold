package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.positions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.AutoRedAlliance;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Intake.IntakeDirections;

@Autonomous(name = "A - Red:Net:DropSample:TouchBar")
public class RedNetSampleBar extends AutoRedAlliance {

    @Override
    public void runOpMode() throws InterruptedException {

        // Global Method for Initializing Auto
        autoStart();

        waitForStart();
        //wait for player to press start
        while (opModeIsActive()) {

            // START AUTO PATH SEQUENCE

            Bot.strafeRight(0.5, 0.4);
            sleep(500);
            arm.up(3, false);
            sleep(500);
            //stop for 1000ms=1sec
            arm.extend(.75, 5.1);
            sleep(500);
            arm.up(5.5, true);
            sleep(500);
            Bot.driveForward(0.5, 4.5);
            sleep(500);
            Bot.rotateLeft(0.5, 0.8);
            sleep(500);
            Bot.strafeRight(0.5, 0.4);
            sleep(500);
            Bot.driveForward(.5, .5);
            sleep(500);
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


//
//            sleep(500);
//            // put the claw mechanism here
//            arm.up(50, false);
//            arm.extend(.5, 50);
//            Bot.driveBack(0.5,1.5);
//            Bot.strafeRight(0.8,1.5);
//            Bot.rotateLeft(0.5,0.5);
            //turn 180 degrees hopefully
//            Bot.driveForward(0.5,2.875);
            //extend claw here

            // END AUTO PATH SEQUENCE
            requestOpModeStop();

        }
        idle();
    }
}