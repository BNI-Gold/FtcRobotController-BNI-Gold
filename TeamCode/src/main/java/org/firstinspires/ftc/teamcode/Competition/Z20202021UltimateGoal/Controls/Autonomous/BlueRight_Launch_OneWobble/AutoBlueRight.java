package org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.BlueRight_Launch_OneWobble;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.StartPosition;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.TargetZone;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Robots.CompetitionBot;


@Autonomous(name = "Remote:Blue:Right:Launch", group = "BLUE")
@Disabled

public class AutoBlueRight extends BlueRight {


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
            RedPark(Bot, targetZone);
            sleep(sleepTime);

//            driveToLaunch (Bot);
//            sleep(sleepTime);
//
//            ScoreRings(Bot,targetZone);
//            sleep(sleepTime);
//
//            targetZone = detectStarterStack(Bot);
//            telemetry.addData("SAMPLING VALUE #: ", Bot.pipeline.avg1);
//            telemetry.addData("NUMBER OF RINGS: ", Bot.pipeline.position);
//            telemetry.addData("TARGET ZONE: ", targetZone);
//            telemetry.update();
//
//            Bot.webcam.closeCameraDevice();
//            sleep(250);
////
//            targetZone = detectStarterStack(Bot);
//            sleep(sleepTime);
//
//            driveToZoneOne(Bot, targetZone);
//            sleep(sleepTime);
////
//            ScoreWobbleSensor(Bot);
//            sleep(sleepTime);
//
//            ParkLaunchLine(Bot,targetZone);
//            sleep(sleepTime);


            requestOpModeStop();

        }
        idle();
    }
}
