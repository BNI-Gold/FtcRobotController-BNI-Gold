package org.firstinspires.ftc.teamcode.Competition.Z20192020SkyStone.Controls.WoodBotControls.AutoPathsWood;//package org.firstinspires.ftc.teamcode.Compitition.ZCompetitionSkyStone.Controls.WoodBotControls.AutoPathsWood;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import org.firstinspires.ftc.teamcode.Compitition.ZCompetitionSkyStone.Controls.WoodBotControls.AutoLoadingWood;
//import org.firstinspires.ftc.teamcode.Compitition.ZCompetitionSkyStone.robots.WoodBot;
//
//@Autonomous(name = "Bot:Wood Auto:Red Loading:Inner: SkyStone")
//@Disabled
//public class AutoRedLoadingSkyStoneInnerWood extends AutoLoadingWood {
//
//    public WoodBot Bot = new WoodBot();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        Bot.initRobot(hardwareMap);
//        Bot.setLinearOp(this);
//
//        setLinearOp(this);
//
//
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//
//            detectStoneDistance(Bot); //drives forward to find any stone
//            sleep(sleepTime);
//
//            detectSkyStone (Bot, "Red"); //drive back until detects SKyStone
//            sleep(sleepTime);
//
//            manipulateStone(Bot, "grab"); //Grabs skystone
//            sleep(sleepTime);
//
//            removeSkyStoneInner(Bot, "Red");
//            sleep(sleepTime);
//
//            adjustToDropSkyStone(Bot, "Red");
//            sleep(sleepTime);
//
//            goToFirstLocation(Bot, "Red");
//            sleep(sleepTime);
//
//            parkSkyStone(Bot);
//
//
//
//            requestOpModeStop();
//        }
//        idle();
//
//    }
//}
