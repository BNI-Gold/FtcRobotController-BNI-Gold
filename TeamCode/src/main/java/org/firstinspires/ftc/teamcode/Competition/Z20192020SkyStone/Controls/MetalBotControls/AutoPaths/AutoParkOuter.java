package org.firstinspires.ftc.teamcode.Competition.Z20192020SkyStone.Controls.MetalBotControls.AutoPaths;//package org.firstinspires.ftc.teamcode.Compitition.ZCompetitionSkyStone.Controls.MetalBotControls.AutoPaths;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import org.firstinspires.ftc.teamcode.Compitition.ZCompetitionSkyStone.Controls.MetalBotControls.AutoBuilding;
//import org.firstinspires.ftc.teamcode.Compitition.ZCompetitionSkyStone.robots.MetalBot;
//
//@Autonomous(name = "Park: Outer")
//@Disabled
//public class AutoParkOuter extends AutoBuilding {
//
//    public MetalBot Bot = new MetalBot();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        Bot.initRobot(hardwareMap, "Build");
//        Bot.setLinearOp(this);
//        //Bot.HookRelease();
//
//        setLinearOp(this);
//
//
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            sleep(6000);
//
//            Bot.driveForward(midSpeed, 2);
//
//            idle();
//            requestOpModeStop();
//        }
//        idle();
//
//    }
//}
