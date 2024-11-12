package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.ProgrammingBot;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Vision.Vision;

@TeleOp(name = "LLTester", group = "testers")
public class LLTester extends LinearOpMode {

    Vision vision = new Vision();

//    public CompBot Bot = new CompBot();
    public ProgrammingBot Bot = new ProgrammingBot();

    public void autoStart() {
        Bot.initRobot(hardwareMap);
        Bot.setLinearOp(this);

        vision.initVision(hardwareMap, true, 4, "tester_");
        vision.setLinearOp(this);
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        autoStart();

        telemetry.setMsTransmissionInterval(11);

        vision.setPipeline(0);

        vision.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = vision.getResult();
            Pose3D pose = vision.getPose();
            double[] offsets = vision.getOffsets();

            double tx = offsets[0];
            double ty = offsets[1];

            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);

            double rotationSpeed = tx * vision.errorMultiplier;
            if (rotationSpeed > vision.minimumCommand) {
                rotationSpeed = Range.clip(rotationSpeed, vision.minimumCommand, 0.45);
            } else {
                rotationSpeed = vision.minimumCommand;
            }

            if (tx < 0-vision.errorOffset) {
                Bot.rotateRight(rotationSpeed);
            } else if (tx > 0+ vision.errorOffset) {
                Bot.rotateLeft(rotationSpeed);
            } else {
                Bot.stopMotors();
            }

            telemetry.addData("pose", pose.toString());

            telemetry.update();
        }
        vision.stop();
    }
}
