package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.Tank.TestLab;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.TankBot;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.mechanisms.TSELocation;
@Autonomous (name = "Lab Testing", group = "zLAB")
@Disabled
public class AutoTestLab extends TestLab {
    public TankBot Bot =  new TankBot();

    public long sleepTime = 250;

    public String Alliance;
    public String AutoPath;

    @Override
    public void runOpMode() throws InterruptedException {
        Alliance = "Blue";
        AutoPath = "CameraShippingHubWarehouse";
//        AutoPath = "DuckToStorage";

        Bot.initRobot(hardwareMap);
        Bot.initWebcam();
        Bot.setLinearOp(this);

        telemetry.addLine("WAITING FOR START >");
        telemetry.addLine("All my telemetry will be on FTC Dashboard");
        telemetry.addLine("http://192.168.43.1:8080/dash");
        telemetry.update();

        Bot.detectBarcode();

        TSELocation location = null;


        telemetry.addLine("WAITING FOR START >");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {


            sleep(1000);

            switch (AutoPath) {
                case "CameraShippingHubWarehouse":
                    location = Bot.detectBarcode();  // uses webcam -- only midpoint telemetry shows
//            location = locator(Bot);            // does not use webcam - new telemetry shows
//            Bot.detectBarcode();

                    DriveShippingHubScore(Bot, Alliance, location);  // use to test if robot functioning!
                    sleep(sleepTime);
                    ShippingHubToWarehosue (Bot, Alliance, location);
                    sleep(sleepTime);
                    break;
                case "DuckToStorage":
                    DriveToDuckSpinner(Bot, Alliance);
                    sleep(sleepTime);
                    spinDuckBlue(Bot);
                    sleep(sleepTime);
                    DuckSpinnerToStorageUnit (Bot, Alliance);
                    break;
            }

            if (AutoPath.equals("CameraShippingHubWarehouse")) {

            }


            if (AutoPath.equals("DuckToStorage")) {

            }

            sleep(1000);
            requestOpModeStop();
        }
        idle();
    }
}
