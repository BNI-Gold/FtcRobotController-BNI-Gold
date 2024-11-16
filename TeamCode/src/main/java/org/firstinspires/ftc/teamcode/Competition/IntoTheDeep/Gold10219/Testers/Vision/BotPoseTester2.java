package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.ProgrammingBot.ProgrammingBot;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Vision.PoseTypes;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Vision.Vision;

import java.math.BigDecimal;
import java.math.RoundingMode;

@TeleOp(name = "BotPoseTester2", group = "testers")
public class BotPoseTester2 extends LinearOpMode {

    Vision vision = new Vision();

    //    public CompBot Bot = new CompBot();
    public ProgrammingBot Bot = new ProgrammingBot();
    public Pinpoint pinpoint = new Pinpoint();

    public void autoStart() {
        Bot.initRobot(hardwareMap);
        Bot.setLinearOp(this);

        pinpoint.setLinearOp(this);
        pinpoint.initPinpoint(hardwareMap);

        vision.initVision(hardwareMap, pinpoint, true, 4, "tester_");
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

        while (opModeIsActive()) {
            telemetry.addLine("OpMode Active");

//            vision.updateYaw(); Uncommenting this to see if same two lines in vision.updateYaw()
//            method work in vision.getResult();
            vision.getResult();
            pinpoint.update();

            if (vision.lastResultValid()) {
                telemetry.addLine("Result Valid");
                Pose3D MT1 = vision.getPose(PoseTypes.MT1);
                Pose3D MT2 = vision.getPose(PoseTypes.MT2);

                BigDecimal MT1x = new BigDecimal(MT1.getPosition().x).setScale(4, RoundingMode.DOWN);
                BigDecimal MT1y = new BigDecimal(MT1.getPosition().y).setScale(4, RoundingMode.DOWN);
                double MT1o = MT1.getOrientation().getYaw();
                telemetry.addData("MT1 X: ", MT1x);
                telemetry.addData("MT1 Y: ", MT1y);
                telemetry.addData("MT1 O: ", MT1o);
                telemetry.addLine();

                BigDecimal MT2x = new BigDecimal(MT2.getPosition().x).setScale(4, RoundingMode.DOWN);
                BigDecimal MT2y = new BigDecimal(MT2.getPosition().y).setScale(4, RoundingMode.DOWN);
                double MT2o = MT2.getOrientation().getYaw();
                telemetry.addData("MT2 X: ", MT2x);
                telemetry.addData("MT2 Y: ", MT2y);
                telemetry.addData("MT2 O: ", MT2o);
                telemetry.addLine();

                double tagCount = vision.getTagCount();
                telemetry.addData("Tag count: ", tagCount);
                telemetry.addLine();

                double pinpointYaw = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
                telemetry.addData("Pinpoint Heading: ", pinpointYaw);

                if (tagCount == 2) {
                    //Tag count is two; heading should be determined from MT1 and updated in pinpoint
                    double yaw = MT1.getOrientation().getYaw(AngleUnit.DEGREES);
                    pinpoint.updateHeading(yaw);
                }

                pinpoint.updateXYPosition(MT2.getPosition().x, MT2.getPosition().y);
            } else {
                telemetry.addLine("Invalid Result");
                telemetry.addLine();
            }

            Pose2D p = pinpoint.getPosition();
            telemetry.addData("p2d x: ", p.getX(DistanceUnit.INCH));
            telemetry.addData("p2d y: ", p.getY(DistanceUnit.INCH));
            telemetry.addData("p2d o: ", p.getHeading(AngleUnit.DEGREES));

            telemetry.update();
        }
        vision.stop();
    }
}
