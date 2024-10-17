package org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.BlueRightPowerShot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.StartPosition;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.TargetZone;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Robots.CompetitionBot;


@Autonomous(name = "Remote:Blue:Right:LaunchPowerShot:Wobble", group = "BLUE")
@Disabled

public class AutoBlueRightPowerShot extends BlueRightPowerShot {


    public CompetitionBot Bot = new CompetitionBot();
    public StartPosition startPosition = null;
    public TargetZone targetZone = null;
    public long sleepTime = 95;



    @Override
    public void runOpMode() throws InterruptedException {

        Bot.initRobot(hardwareMap, "TeleOp", "auto");
        Bot.initCamera();
        Bot.setLinearOp(this);


        startPosition = StartPosition.BlueRight;


        waitForStart();


        while (opModeIsActive()){

            driveToLaunch (Bot);
            sleep(sleepTime);

            ScoreRings(Bot,targetZone);
            sleep(sleepTime);

//            targetZone = detectStarterStack(Bot);
//            telemetry.addData("SAMPLING VALUE #: ", Bot.pipeline.avg1);
//            telemetry.addData("NUMBER OF RINGS: ", Bot.pipeline.position);
//            telemetry.addData("TARGET ZONE: ", targetZone);
//            telemetry.update();
//
//            Bot.webcam.closeCameraDevice();
//            sleep(250);
//
//            targetZone = detectStarterStack(Bot);
//            sleep(sleepTime);
//
//            Bot.WobbleArmLowerColorSensor();
//            sleep(sleepTime);
//
//            driveToZoneOne(Bot, targetZone);
//            sleep(sleepTime);
//
//            ScoreWobbleSensor(Bot);
//            sleep(sleepTime);
//
////            Bot.WobbleArmLower(1);
////            sleep(200);
////            Bot.WobbleArmStopMotors();
////            sleep(sleepTime);
////
////            driveToLeftWobble(Bot, targetZone);
////            sleep(sleepTime);
//
//            ParkLaunchLine(Bot,targetZone);
//            sleep(sleepTime);

            requestOpModeStop();

        }
        idle();
    }
}
