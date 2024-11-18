package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.math.BigDecimal;
import java.math.BigInteger;
import java.math.RoundingMode;

public class PoseHelper {
    public Vision vision = null;
    public Pinpoint pinpoint = null;
    public LinearOpMode LinearOp = null;
    public Telemetry telemetry = null;
    public boolean LLInUse = false;
    public Pose2D pose = null;
    public PoseHelperResultTypes resultType = null;
    public double headingTolerance = 3;

    public PoseHelper() {
    }

    public void setLinearOp(LinearOpMode LinearOp) {
        this.LinearOp = LinearOp;
        telemetry = LinearOp.telemetry;
    }

    public void setDevices(Vision vision, Pinpoint pinpoint) {
        this.vision = vision;
        this.pinpoint = pinpoint;
    }

    public void updateLLUsage(boolean inUse) {
        LLInUse = inUse;
    }

    public PoseHelperResultTypes getResultType() {
        return resultType;
    }

    public void updateHeading() {
        if (!LLInUse) {
            vision.setPipeline(3);
            vision.getResult();

            if (vision.lastResultValid()) {
                int tagCount = vision.getTagCount();
                if (tagCount == 2) {
                    double heading = vision.getPose(PoseTypes.MT1).getOrientation().getYaw(AngleUnit.DEGREES);
                    pinpoint.updateHeading(heading);
                    resultType = PoseHelperResultTypes.MT2Tag;
                }
            }
        }
        pose = ShortenXY(pinpoint.getPosition());
    }

    public void syncPose() {
        pinpoint.update();
        if (!LLInUse) {
            vision.setPipeline(3);
            vision.getResult();

            if (vision.lastResultValid()) {
                int tagCount = vision.getTagCount();
                if (tagCount == 2) {
                    resultType = PoseHelperResultTypes.MT2Tag;
                } else {
                    resultType = PoseHelperResultTypes.MT;
                }
                Position MT2Position = vision.getPose(PoseTypes.MT2).getPosition().toUnit(DistanceUnit.INCH);
                pinpoint.updateXYPosition(MT2Position.x, MT2Position.y);
            } else {
                resultType = PoseHelperResultTypes.PINPOINT;
            }
        } else {
            resultType = PoseHelperResultTypes.PINPOINT;
        }
        pose = ShortenXY(pinpoint.getPosition());
    }

    public void updatePose() {
        pinpoint.update();
        pose = ShortenXY(pinpoint.getPosition());
    }

    private Pose2D ShortenXY(Pose2D oldPose) {
        double x = BigDecimal.valueOf(oldPose.getX(DistanceUnit.INCH)).setScale(4, RoundingMode.DOWN).doubleValue();
        double y = BigDecimal.valueOf(oldPose.getY(DistanceUnit.INCH)).setScale(4, RoundingMode.DOWN).doubleValue();
        double heading = oldPose.getHeading(AngleUnit.DEGREES);

        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
    }

    public Pose2D getPose() {
        return pose;
    }
}
