package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.PoseHelperResultTypes;

@Disabled
@Autonomous(name = "BotPoseAutoTester")
public class BotPoseAutoTester extends AutoMain{

    @Override public void runOpMode() {
        boolean viewingAprilTags = true;
        DistanceUnit du = DistanceUnit.INCH;
        AngleUnit au = AngleUnit.DEGREES;

        double linearTolerance = 2;
        double angularTolerance = 2;

        autoStart();

        telemetry.setMsTransmissionInterval(11);

        vision.start();

        waitForStart();

        while(opModeIsActive()) {
            pose.updatePose();

            //Robot will start with view of two tags. Update Pinpoint heading and position.
            if (viewingAprilTags) {
                pose.updateHeading();
                pose.syncPose();

                viewingAprilTags = false;
            }

            Pose2D currentPose = pose.getPose();
            PoseHelperResultTypes resultType = pose.getResultType();

            double x = currentPose.getX(du);
            double y = currentPose.getY(du);
            double h = currentPose.getHeading(au);

            double desiredX = 0;
            double desiredY = 36;
            double desiredHeading = 90;

            double xDiff = x-desiredX;
            double yDiff = y-desiredY;
            double headingDiff = h-desiredHeading;

            telemetry.addData("XD: ", xDiff);
            telemetry.addData("YD: ", yDiff);
            telemetry.addData("HD: ", headingDiff);

            telemetry.update();

            while (Math.abs(xDiff) > linearTolerance && opModeIsActive()) {
                if (xDiff > 0) {
                    Bot.driveForwardInches(0.5, xDiff);
                }
            }

            while (Math.abs(yDiff) > linearTolerance && opModeIsActive()) {
                if (yDiff > 0) {
                    Bot.strafeRightInches(0.5, yDiff);
                }
            }

            while (Math.abs(headingDiff) > angularTolerance && opModeIsActive()) {
                if (headingDiff >= 0) {
                    Bot.rotateRightDegrees(0.5, headingDiff);
                }
            }



        }
    }
}
