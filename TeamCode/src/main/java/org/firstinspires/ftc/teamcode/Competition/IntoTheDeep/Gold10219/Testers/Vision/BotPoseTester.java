package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.ProgrammingBot.ProgrammingBot;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Vision.PoseTypes;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Vision.Vision;

@TeleOp(name = "BotPoseTester", group = "testers")
public class BotPoseTester extends LinearOpMode {

    Vision vision = new Vision();

    //    public CompBot Bot = new CompBot();
    public ProgrammingBot Bot = new ProgrammingBot();

    public void autoStart() {
        Bot.initRobot(hardwareMap);
        Bot.setLinearOp(this);

        vision.initVision(hardwareMap, Bot.imu, true, 4, "tester_");
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

            vision.getResult();

            if (vision.lastResultValid()) {
                telemetry.addLine("Result Valid");
//                double[] offsets = vision.getOffsets();
                Pose3D MT1 = vision.getPose(PoseTypes.MT1);
                Pose3D MT2 = vision.getPose(PoseTypes.MT2);
//Ï
//                double tx = offsets[0];
//                double ty = offsets[1];
//
//                telemetry.addData("tx", tx);
//                telemetry.addData("ty", ty);
//
//                double rotationSpeed = Math.abs(tx) * vision.errorMultiplier;
//                if (rotationSpeed > vision.minimumCommand && rotationSpeed < vision.maximumCommand) {
//                    rotationSpeed = Range.clip(rotationSpeed, vision.minimumCommand, vision.maximumCommand);
//                } else if (rotationSpeed > vision.maximumCommand) {
//                    rotationSpeed = vision.maximumCommand;
//                } else {
//                    rotationSpeed = vision.minimumCommand;
//                }

//                if (tx < 0-vision.errorOffset) {
//                    Bot.rotateLeft(rotationSpeed);
//                } else if (tx > 0+ vision.errorOffset) {
//                    Bot.rotateRight(rotationSpeed);
//                } else {
//                    Bot.stopMotors();
//                }

                double MT1x = MT1.getPosition().x;
                double MT1y = MT2.getPosition().y;
                telemetry.addData("MT1 Location:", "(" + MT1x + ", " + MT1y + ")");

                double MT2x = MT2.getPosition().x;
                double MT2y = MT2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + MT2x + ", " + MT2y + ")");
            } else {
                telemetry.addLine("Invalid Result");
            }

            telemetry.update();
        }
        vision.stop();
    }
}
