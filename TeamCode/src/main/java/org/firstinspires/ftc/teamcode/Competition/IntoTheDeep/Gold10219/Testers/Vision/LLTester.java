package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.Vision;

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
//            telemetry.addLine("OpMode Active");
            Pipelines[] pipelinesToTest = new Pipelines[]{Pipelines.RED, Pipelines.YELLOW, Pipelines.BLUE};
            double[] offsets = vision.getClosestOffsets(pipelinesToTest);



            if (vision.lastResultValid()) {
//                telemetry.addLine("Result Valid");
                Pose3D pose = vision.getPose();

                double tx = offsets[0];
                double ty = offsets[1];

//                telemetry.addData("tx", tx);
//                telemetry.addData("ty", ty);

                double rotationSpeed = Math.abs(tx) * vision.errorMultiplier;
                if (rotationSpeed > vision.minimumCommand && rotationSpeed < vision.maximumCommand) {
                    rotationSpeed = Range.clip(rotationSpeed, vision.minimumCommand, vision.maximumCommand);
                } else if (rotationSpeed > vision.maximumCommand) {
                    rotationSpeed = vision.maximumCommand;
                } else {
                    rotationSpeed = vision.minimumCommand;
                }

                if (tx < 0-vision.errorOffset) {
                    Bot.rotateLeft(rotationSpeed);
                } else if (tx > 0+ vision.errorOffset) {
                    Bot.rotateRight(rotationSpeed);
                } else {
                    Bot.stopMotors();
                }

//                telemetry.addData("pose", pose.toString());
            } else {
//                telemetry.addLine("Invalid Result");
            }

//            telemetry.update();
        }
        vision.stop();
    }
}
