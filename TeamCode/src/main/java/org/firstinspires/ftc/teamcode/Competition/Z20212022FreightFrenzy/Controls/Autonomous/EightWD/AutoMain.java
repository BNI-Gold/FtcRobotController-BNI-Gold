package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.EightWD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.EightWheelBot;

public abstract class AutoMain extends LinearOpMode {

    int sleepTime = 100;

    public void blue_carousel_spinDuck(EightWheelBot Bot) {

        Bot.driveBackward(0.5, 1.2);
        sleep(sleepTime);

        Bot.rotateLeft(1, 1.3);
        sleep(sleepTime);

        Bot.driveForward(0.5, 1.5);
        sleep(sleepTime);

        Bot.rotateRight(1,.6);
        sleep(sleepTime);

//        Bot.driveForward(0.4, .3);
//        sleep(sleepTime);

        Bot.duckspincounterclockwise();
        sleep(3000);

        Bot.duckspinstop();
        sleep(sleepTime);

        Bot.rotateRight(.8,.2);
        sleep(sleepTime);

        Bot.driveBackward(.6,1.5);
        sleep(sleepTime);
        //second right/left is the pole the duck spinner is on


    }

    public void red_carousel_spinDuck(EightWheelBot Bot) {

        Bot.driveBackward(0.5, .9);
        sleep(sleepTime);

        Bot.rotateRight(1, 1);
        sleep(sleepTime);

        Bot.driveForward(0.5, 2.25);
        sleep(sleepTime);

        Bot.driveForward(0.3, .4);
        sleep(sleepTime);

        Bot.duckspinclockwise();
        sleep(5000);

        Bot.duckspinstop();
//
        Bot.driveBackward(.7,.8);
        sleep(sleepTime);

        Bot.rotateLeft(.8,.8);
        sleep(sleepTime);
//
        Bot.driveBackward(.6,.5);
        sleep(sleepTime);
        //second right/left is the pole the duck spinner is on


    }

    public void Shipping_Hub_Score (EightWheelBot Bot) {

        Bot.senseLyftExtend();
        sleep(1000);

        Bot.setboxHolder2down();
        sleep(2000);

        Bot.setboxHolder2up();
        sleep(500);

        Bot.senseLyftcolapse();
        sleep(sleepTime);

        Bot.lyftextenderstop();
    }
}
