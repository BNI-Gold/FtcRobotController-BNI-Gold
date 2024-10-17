package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.Tank.BlueParkWarehouseDirect;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.Tank.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.TankBot;

@Autonomous (name = "Blue: Warehouse Park", group = "BLUE")
@Disabled

public class AutoBlueParkWarehouseDirect extends AutoMain {
    public TankBot Bot =  new TankBot();

    public long sleepTime = 250;

    private double straightSpd = 0.6;
    private double turnEncoderSpd = 0.5;
    //        Speed .2 == too low for gyro turn
    private double turnGyro1 = 0.25;
    private double turnGyro2 = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        Bot.initRobot(hardwareMap);
//        Bot.initWebcam();
        Bot.setLinearOp(this);

        telemetry.addLine("WAITING FOR START >");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            Bot.driveForward(1, 4);
            sleep(sleepTime);
            Bot.rotateRight(1, 1.5);
            sleep(sleepTime);

            Bot.gyroCorrection(turnGyro1,-90);
            sleep(sleepTime);

            Bot.driveBackward(1, 8.5);
            sleep(sleepTime);

            requestOpModeStop();
        }
        idle();
    }
}
