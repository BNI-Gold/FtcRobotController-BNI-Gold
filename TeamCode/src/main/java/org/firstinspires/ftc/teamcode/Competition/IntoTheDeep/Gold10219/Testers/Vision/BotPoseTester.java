package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Pinpoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.PoseHelper;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.PoseHelperResultTypes;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Vision;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.ProgrammingBot.ProgrammingBot;

import java.math.BigDecimal;
import java.math.RoundingMode;

@Disabled
@TeleOp(name = "BotPoseTester", group = "testers")
public class BotPoseTester extends LinearOpMode {

    public Vision vision = new Vision();
    //    public CompBot Bot = new CompBot();
    public ProgrammingBot Bot = new ProgrammingBot();
    public Pinpoint pinpoint = new Pinpoint();
    public PoseHelper pose = new PoseHelper();

    public void autoStart() {
        Bot.initRobot(hardwareMap);
        Bot.setLinearOp(this);

        pinpoint.setLinearOp(this);
        pinpoint.initPinpoint(hardwareMap);

        vision.setLinearOp(this);
        vision.initVision(hardwareMap, pinpoint, true, 4, "tester_");

        pose.setLinearOp(this);
        pose.setDevices(vision, pinpoint);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        autoStart();

        telemetry.setMsTransmissionInterval(11);

        vision.start();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("OpMode Active");

            //pose.updatePose() must be called every loop, even if position data isn't needed.
            //pose.updatePose() updates Pinpoint with data depending on number of april tags available.
            //pose.updatePose() also calls vision.getResult() and pinpoint.update(), so no need to call those here
            pose.updatePose();

            if (gamepad1.a) {
                pose.updateHeading();

                pose.syncPose();
            }

            //pose.getPose() returns the pose that is stored in the PoseHelper class when pose.updatePose() is called.
            Pose2D currentPose = pose.getPose();

            PoseHelperResultTypes resultType = pose.getResultType();

            BigDecimal x = BigDecimal.valueOf(currentPose.getX(DistanceUnit.INCH)).setScale(4, RoundingMode.DOWN);
            BigDecimal y = BigDecimal.valueOf(currentPose.getY(DistanceUnit.INCH)).setScale(4, RoundingMode.DOWN);
            double heading = currentPose.getHeading(AngleUnit.DEGREES);

            telemetry.addData("Result Type: ", resultType);
            telemetry.addData("X Position: ", x);
            telemetry.addData("Y Position: ", y);
            telemetry.addData("Heading: ", heading);

            telemetry.update();
        }
        vision.stop();
    }
}
