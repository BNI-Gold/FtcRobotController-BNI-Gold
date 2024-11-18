package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Poses;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.localization.Pose;

public class FieldPoses {

    //Spike mark locations
    public Pose pose1 = new FCPose(13, 42); // Automatically converted to corner-centric
}

// FCPose class that extends Pose
class FCPose extends Pose {
    public FCPose(double xField, double yField) {
        super(xField + 72, yField + 72);
    }
}

//https://ftc-resources.firstinspires.org/file/ftc/game/manual-09