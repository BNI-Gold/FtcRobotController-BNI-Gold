package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.BrightLights;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BrightLights.RGBIndicator;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;

@Disabled
@TeleOp(name = "B - RGB Indicator Tester", group = "testers")
public class RGBIndicatorTester extends OpMode {
    public CompBot Bot = new CompBot();
    RGBIndicator indicator = new RGBIndicator();

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        indicator.initIndicator(hardwareMap);
    }

    @Override
    public void loop() {
        indicatorControl();
    }

    public void indicatorControl() {
        if (gamepad1.a) indicator.setColor(RGBIndicator.LightOptions.RED);
        if (gamepad1.b) indicator.setColor(RGBIndicator.LightOptions.GREEN);
        if (gamepad1.x) indicator.setColor(RGBIndicator.LightOptions.BLUE);
    }
}
