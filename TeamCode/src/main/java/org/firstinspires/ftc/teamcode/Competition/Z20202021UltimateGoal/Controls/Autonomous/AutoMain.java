package org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Modules.EasyOpenCVWebcam;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Robots.CompetitionBot;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Robots.LabBot;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Robots.StraferKit;

public abstract class AutoMain extends LinearOpMode {

    public int sleepTimeDrive = 200;

    public TargetZone zone = null;

//    This will later detect our 0, 1, or 4 ring stacks!
    public TargetZone detectStarterStack (CompetitionBot Bot) {

//         Following 2 lines are for Hard Coding the Target Zone.  Uncomment to not use EOCV.  AND comment out the lines below them.
//                zone = TargetZone.B;
//                return zone;
//    Line below are to use with EOCV

        Bot.pipeline.getAnalysis();
        if (Bot.pipeline.position == EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.NONE) {
            return TargetZone.A;
        }
        else if (Bot.pipeline.position == EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.ONE) {
            return TargetZone.B;
        }
        else {
            return TargetZone.C;
        }


    }

    public TargetZone detectStarterStack (LabBot Bot) {

//         Following 2 lines are for Hard Coding the Target Zone.  Uncomment to not use EOCV.  AND comment out the lines below them.
        zone = TargetZone.A;
        return zone;
//    Line below are to use with EOCV

//        Bot.pipeline.getAnalysis();
//        if (Bot.pipeline.position == EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.NONE) {
//            return TargetZone.A;
//        }
//        else if (Bot.pipeline.position == EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.ONE) {
//            return TargetZone.B;
//        }
//        else {
//            return TargetZone.C;
//        }


    }

    public TargetZone detectStarterStack (StraferKit Bot) {

//         Following 2 lines are for Hard Coding the Target Zone.  Uncomment to not use EOCV.  AND comment out the lines below them.
        zone = TargetZone.C;
        return zone;
//    Line below are to use with EOCV

//        Bot.pipeline.getAnalysis();
//        if (Bot.pipeline.position == EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.NONE) {
//            return TargetZone.A;
//        }
//        else if (Bot.pipeline.position == EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.ONE) {
//            return TargetZone.B;
//        }
//        else {
//            return TargetZone.C;
//        }


    }

    public void driveToLaunch (CompetitionBot Bot) throws InterruptedException {

//        Bot.driveGyroForward(.8, 3.8);
//        sleep(sleepTimeDrive);
//        Bot.gyroCorrection(0.2,0);
//        sleep(sleepTimeDrive);


//        old drive to launch for high goal double wobble
        Bot.LauncherOn(1360);  //to engage launcher before getting to launch point.
        //1340 power before
        Bot.driveGyroForward(.8,3.90);
        sleep(sleepTimeDrive);
        Bot.CameraDetect();
        sleep(sleepTimeDrive);
        Bot.gyroCorrection(0.2,0);
        sleep(sleepTimeDrive);
        Bot.strafeLeft(0.6, 1.3);
        sleep(sleepTimeDrive);
//        2.35 before
        Bot.gyroCorrection(0.2,0);
        sleep(sleepTimeDrive);

    }

    public void driveToLaunch (StraferKit Bot) throws InterruptedException {
        Bot.driveForward(0.35, 6.4);
        sleep(sleepTimeDrive);
//        Bot.rotateLeft(0.3, 1.75);
//        sleep(sleepTimeDrive);
        Bot.strafeLeft(0.4,2);
        sleep(sleepTimeDrive);
        Bot.gyroCorrection(0.2,0);
        sleep(1000);

    }




//    Lower servo to score it, and then raise it to not damage anything.
    public void ScoreWobble (CompetitionBot Bot){
        Bot.WobbleArmLower(1);
        sleep(300);
        Bot.WobbleOpen();
        sleep(800);
        Bot.WobbleArmRaised(1);
        sleep(sleepTimeDrive);
        Bot.WobbleClosed();
        sleep(sleepTimeDrive);
    }


    public void ScoreWobbleSensor (CompetitionBot Bot) {
//        Bot.WobbleArmLowerColorSensor();
//        sleep(50);
        sleep(sleepTimeDrive);
        Bot.WobbleOpen();
        sleep(500);
//        Bot.WobbleArmStopClose();
//        sleep(500);
        Bot.driveForward(.5,.3);
        sleep(sleepTimeDrive);
        Bot.WobbleArmRaiseColorSensor();
        sleep(sleepTimeDrive);

    }


    public void CollectDoubleWobble (CompetitionBot Bot){
        Bot.WobbleArmLower(1);
        sleep(sleepTimeDrive);
        Bot.WobbleClosed();
        sleep(sleepTimeDrive);
        Bot.WobbleArmRaised(1);
        sleep(sleepTimeDrive);


    }
    public void CollectWobble (CompetitionBot Bot){
        Bot.WobbleArmLower(1);
        sleep(sleepTimeDrive);
        Bot.WobbleOpen();
        sleep(sleepTimeDrive);
        Bot.strafeRight(.3,.7);
        sleep(sleepTimeDrive);
        Bot.gyroCorrection(.2,0);
        sleep(sleepTimeDrive);
        Bot.strafeLeft(.3,.04);
        sleep(sleepTimeDrive);
        Bot.WobbleClosed();
        sleep(sleepTimeDrive);
        Bot.WobbleArmRaised(1);
        sleep(sleepTimeDrive);
    }
    public void RingPusher (CompetitionBot Bot) {
        Bot.RingPush();
        sleep(sleepTimeDrive);
        Bot.RingPull();
        sleep(sleepTimeDrive);
    }


        public void ScoreLaunch (CompetitionBot Bot){

        Bot.LauncherOn(1);
        sleep(3000);
        Bot.RingPush();
        sleep(100);
        Bot.RingPull();
        sleep(100);
        Bot.RingPush();
        sleep(100);
        Bot.RingPull();
        sleep(100);
        Bot.RingPush();
        sleep(100);
        Bot.RingPull();
        sleep(100);
//        Bot.IntakeOn(1);

    }

        public void StopLaunch (CompetitionBot Bot){
        Bot.LauncherOff(0);
        sleep(1000);
//        Bot.IntakeOff(0);
    }

        public void LEDs (LabBot Bot, TargetZone target) {
        if (TargetZone.A == target){
            Bot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
        else if (TargetZone.B == target){
            Bot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        }
        else {
            Bot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }

    public void LEDs (CompetitionBot Bot, TargetZone target) {
        if (TargetZone.A == target){
            Bot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
        else if (TargetZone.B == target){
            Bot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        }
        else {
            Bot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }
    public void RedPark (CompetitionBot Bot, TargetZone targetZone) {
        Bot.driveForward(1, 1);
        sleep(sleepTimeDrive);
        Bot.strafeLeft(0.7, 2);
        sleep(sleepTimeDrive);
        Bot.driveForward(1, 4);
        sleep(sleepTimeDrive);
        Bot.gyroCorrection(0.2, 0);
        sleep(sleepTimeDrive);
    }
}
