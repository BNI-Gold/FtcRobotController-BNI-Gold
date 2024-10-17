package org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Controls.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Robots.CompBot;

@Disabled
@Autonomous (name = "Encoder Tester")
public class EncoderTester extends LinearOpMode {

    public CompBot testBot = new CompBot();
    @Override
    public void runOpMode(){
        testBot.initRobot(hardwareMap);
        testBot.setLinearOp(this);


        telemetry.addLine("Robot Awaiting Start Procedure");
        telemetry.update();


        waitForStart();




        while (opModeIsActive()) {
            telemetryUpdate("Drive Forward");
            testBot.driveForward(1,2);
            sleep(2000);

            telemetryUpdate("Drive Back");
            testBot.driveBack(1,2);
            sleep(2000);

            telemetryUpdate("Strafe Left");
            testBot.strafeLeft(1,.5);
            sleep(2000);

            telemetryUpdate("Strafe Right");
            testBot.strafeRight(1,1);
            sleep(200);
            testBot.strafeLeft(1,.5);
            sleep(2000);

            telemetryUpdate("Rotate Left");
            testBot.rotateLeft(1,1);
            sleep(2000);

            telemetryUpdate("Rotate Right");
            testBot.rotateRight(1,1);
            sleep(2000);

            telemetryUpdate("Diagonal Left Forward");
            testBot.diagonalLeftForward(1,1);
            sleep(2000);

            telemetryUpdate("Diagonal Left Back");
            testBot.diagonalLeftBack(1,1);
            sleep(2000);

            telemetryUpdate("Diagonal Right Forward");
            testBot.diagonalRightForward(1,1);
            sleep(2000);


            telemetryUpdate("Diagonal Right Back");
            testBot.diagonalRightBack(1,1);
            sleep(2000);
        }
    }

    public void telemetryUpdate(String comment) {

        telemetry.addLine(comment);
        telemetry.addData("Front Lef Motor:", testBot.frontLeftMotor.getPower());
        telemetry.addData("Front Rig Motor:", testBot.frontRightMotor.getPower());
        telemetry.addData("Rear Lef Motor:", testBot.rearLeftMotor.getPower());
        telemetry.addData("Rear Rig Motor:", testBot.rearRightMotor.getPower());
        telemetry.addData("Encoder Count: ", testBot.frontLeftMotor.getCurrentPosition());
        telemetry.update();
    }
}
