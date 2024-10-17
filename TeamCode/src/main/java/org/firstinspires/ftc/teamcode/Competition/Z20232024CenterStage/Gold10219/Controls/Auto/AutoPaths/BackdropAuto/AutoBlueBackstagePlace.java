package org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Controls.Auto.AutoPaths.BackdropAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Controls.Auto.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Robots.CompBot;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
@Disabled
@Autonomous (name = "AAATTT - Blue Backstage Auto - INIT TEST")
public class AutoBlueBackstagePlace extends AutoMain {

    public static final boolean USE_WEBCAM = true;

    public static int oneSecond = 1000;

  //  public TfodProcessor tFod;

    public VisionPortal visionPortal;

  public   CompBot Bot = new CompBot();


    @Override
    public void runOpMode() {
        Bot.initRobot(hardwareMap);
        Bot.setLinearOp(this);

        telemetry.addLine("Robot Awaiting Start Procedure");
        telemetry.update();

       // Bot.endgameArmRotator.setPosition(0.8);

        Bot.drivePosition();

        waitForStart();



        while (opModeIsActive()) {

            telemetry.addLine("Robot Autonomous Control Initialized");

            sleep(oneSecond);

            sleep(oneSecond);
            Bot.driveForward(0.5, 0.5);
            sleep(oneSecond);
            Bot.rotateLeft(0.4,2.2);
            sleep(oneSecond);
            Bot.driveForward(0.5,2.8);
            sleep(oneSecond);
            Bot.strafeRight(0.5, 2.5);
            sleep(oneSecond);
            Bot.driveForward(0.15, 1);
            sleep(oneSecond);

            Bot.viperSlideRight.setPower(-1);
            sleep(600);
            Bot.viperSlideRight.setPower(-0.2);
            sleep(oneSecond);

            sleep(oneSecond);

            Bot.rightPixelClawOpen();
            sleep(oneSecond * 2);

            Bot.rightPixelClawClose();
            sleep(oneSecond);

            Bot.rightPixelClawOpen();
            sleep(oneSecond * 2);

            Bot.viperSlideRight.setPower(0.5);
            sleep(600);

            Bot.viperSlideRight.setPower(0);
            sleep(oneSecond);


            Bot.rightPixelClawClose();
            sleep(oneSecond);

            Bot.strafeLeft(0.6, 1.5);
            sleep(oneSecond);

            telemetry.addLine("Robot Autonomous Control Complete");

            requestOpModeStop();
        }

        idle();


    }





    public void telemetryUpdate(String comment) {

        telemetry.addLine(comment);
        telemetry.addData("Front Lef Motor:", Bot.frontLeftMotor.getPower());
        telemetry.addData("Front Rig Motor:", Bot.frontRightMotor.getPower());
        telemetry.addData("Rear Lef Motor:", Bot.rearLeftMotor.getPower());
        telemetry.addData("Rear Rig Motor:", Bot.rearRightMotor.getPower());
        telemetry.addData("Encoder Count: ", Bot.frontLeftMotor.getCurrentPosition());
        telemetry.update();
    }
}
