package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.ProgrammingBot.ProgrammingBot;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Vision.PoseTypes;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Vision.Vision;

import java.math.BigDecimal;
import java.math.RoundingMode;

@Disabled
@TeleOp(name = "BotPoseTester", group = "testers")
public class BotPoseTester extends LinearOpMode {

    Vision vision = new Vision();

    //    public CompBot Bot = new CompBot();
    public ProgrammingBot Bot = new ProgrammingBot();

    public void autoStart() {
        Bot.initRobot(hardwareMap);
        Bot.setLinearOp(this);

//        vision.initVision(hardwareMap, Bot.imu, true, 4, "tester_");
        vision.setLinearOp(this);

        Bot.imu.resetYaw();
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        autoStart();

        telemetry.setMsTransmissionInterval(11);

        vision.start();

        waitForStart();

//        Pipelines[] pipelinesToTest = new Pipelines[]{Pipelines.RED, Pipelines.YELLOW, Pipelines.BLUE};
//        int closestPipeline = vision.getClosestPipeline(pipelinesToTest);

        vision.setPipeline(3);

        //This should fix bot outside of field error.
        //For example, if bot is facing away from audience (towards hallway in comp room), this should be 180.
//        vision.setStartingRotation(0);

        while (opModeIsActive()) {
            telemetry.addLine("OpMode Active");

//            vision.updateYaw(); Uncommenting this to see if same two lines in vision.updateYaw()
//            method work in vision.getResult();
            vision.getResult();

            if (vision.lastResultValid()) {
                telemetry.addLine("Result Valid");
                Pose3D MT1 = vision.getPose(PoseTypes.MT1);
                Pose3D MT2 = vision.getPose(PoseTypes.MT2);

                BigDecimal MT1x = new BigDecimal(MT1.getPosition().x).setScale(4, RoundingMode.DOWN);
                BigDecimal MT1y = new BigDecimal(MT2.getPosition().y).setScale(4, RoundingMode.DOWN);
                telemetry.addData("MT1 X: ", MT1x);
                telemetry.addData("MT1 Y: ", MT1y);
                telemetry.addLine();

                BigDecimal MT2x = new BigDecimal(MT2.getPosition().x).setScale(4, RoundingMode.DOWN);
                BigDecimal MT2y = new BigDecimal(MT2.getPosition().y).setScale(4, RoundingMode.DOWN);
                telemetry.addData("MT2 X: ", MT2x);
                telemetry.addData("MT2 Y: ", MT2y);
                telemetry.addLine();

                telemetry.addData("Tag count: ", vision.getTagCount());
            } else {
                telemetry.addLine("Invalid Result");
            }

            telemetry.update();
        }
        vision.stop();
    }
}
