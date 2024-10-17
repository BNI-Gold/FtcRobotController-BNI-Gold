package org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.Archive.Lab.DuValTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.StartPosition;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.TargetZone;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Robots.CompetitionBot;

@Autonomous (name = "Remote:Blue:Left:TEST:", group = "BLUE")
@Disabled

public class AutoBlueLeftTEST extends BlueLeftTEST {

    // Initiailize our variables.
    public CompetitionBot Bot = new CompetitionBot();
    public StartPosition startPosition = null;
    public TargetZone targetZone = null;
    public long sleepTime = 100;



    @Override
    public void runOpMode() throws InterruptedException {
//        Constructor to set up our hardware mapping.
        Bot.initRobot(hardwareMap, "BlueLeft", "auto");
        Bot.initCamera();
        Bot.setLinearOp(this);
//        This is hard-coded for this auto.  May or may not use, but here just in case.
        sleep(1000);
        startPosition = StartPosition.BlueLeft;
        targetZone = detectStarterStack(Bot);
        telemetry.addData("SAMPLING VALUE #: ", Bot.pipeline.avg1);
        telemetry.addData("NUMBER OF RINGS: ", Bot.pipeline.position);
        telemetry.addData("TARGET ZONE: ", targetZone);

        telemetry.addLine("WAITING FOR START >");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
//            Change value in detectStarterStack to test different Auto paths.
//            select the function call below and use "Cmd + B" to go direcrtly to that function.
            targetZone = detectStarterStack(Bot);
            telemetry.addData("SAMPLING VALUE #: ", Bot.pipeline.avg1);
            telemetry.addData("NUMBER OF RINGS: ", Bot.pipeline.position);
            telemetry.addData("TARGET ZONE: ", targetZone);
            telemetry.update();

            //STOP THE CAMERA! - closeCameraDevice does close the camera on RC
            Bot.webcam.closeCameraDevice();  //This does stop the camera.  Uncomment when ready to use Webcam.
//OR
            //Bot.webcam.pauseViewport();   // This line hasn't been used - so leave commented out.

            sleep(1000);
            LEDs(Bot, targetZone);
            gyroTEST(Bot);
//            Drives robot to target Zone
//            driveToTargetZone (Bot, targetZone);
//            sleep(sleepTime);
//
//
////            Lower and raise the Servo to score the Wobble.
//            ScoreWobble(Bot);
//            sleep(sleepTime);
//            driveToWobble(Bot, targetZone);
//            sleep(sleepTime);
//
//            //to collect 2nd wobble
//            CollectWobble(Bot);
//            sleep(sleepTime);
//
//            ScoreWobble(Bot);
//            sleep(sleepTime);


//            Required to stop Autonomous!
            requestOpModeStop();
        }
        idle();
    }
}
