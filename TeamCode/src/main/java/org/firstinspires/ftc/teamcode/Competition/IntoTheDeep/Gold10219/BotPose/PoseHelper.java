package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Offsets;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class PoseHelper {
    public Vision vision = null;
    public Pinpoint pinpoint = null;
    public Telemetry telemetry = null;
    public boolean LLInUse = false;
    public Pose2D pose = null;
    public PoseHelperResultTypes resultType = null;
    public double headingTolerance = 3;

    public enum StartTypes {
        BLUE, RED
    }

    private Offsets A11Offsets = new Offsets().remX(.3).addY(.5);
    private Offsets A12Offsets = new Offsets();
    private Offsets A13Offsets = new Offsets().addY(.25);
    private Offsets A14Offsets = new Offsets().remY(.25);
    private Offsets A15Offsets = new Offsets();
    private Offsets A16Offsets = new Offsets().remX(.25).remY(1.2);

    public PoseHelper() {}

    public void setLinearOp(LinearOpMode LinearOp) {
        telemetry = LinearOp.telemetry;
    }

    public void setOp(OpMode Op) {
        telemetry = Op.telemetry;
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

    public enum Alliances {
        BLUE, RED
    }

    public Pose2D getSmartPose(Alliances alliance) {
        if (!LLInUse) {
            vision.setPipeline(3);
            while (!vision.lastResultValid() || vision.Position.getTagCount() == 0) {
                vision.getResult();
            }

            double heading = 0;

            int tagCount = vision.Position.getTagCount();
            telemetry.addData("TagCount: ", tagCount);
            Offsets offsets = new Offsets();
            if (tagCount == 2) {
                //Robot is facing two tags. Should get heading using MT, then set pinpoint heading. Then get position using MT2.
                heading = vision.Position.getPose(PoseTypes.MT1).getOrientation().getYaw(AngleUnit.DEGREES);
                telemetry.addData("H: ", heading);
                if (heading > 180) {
                    heading -= 360;
                }
            } else if (tagCount == 1) {
                //Robot is only facing one tag. Should determine heading based off of known possible starts, then set pinpoint heading. Then get position using MT2.
                int tag = vision.Position.getCurrentTags().get(0).getFiducialId();
                telemetry.addData("Tag: ", tag);

                switch (alliance) {
                    case BLUE:
                        switch (tag) {
                            case 11:
                                heading = 180;
                                offsets = A11Offsets;
                                break;
                            case 12:
                                double yaw = vision.Position.getPose(PoseTypes.MT1).getOrientation().getYaw(AngleUnit.DEGREES);
                                if (yaw > 180) {
                                    yaw -= 360;
                                }

                                if (yaw > -90 && yaw < 90) {
                                    heading = 0;
                                } else if (yaw < -90 || yaw > 90) {
                                    heading = 180;
                                }

                                offsets = A12Offsets;
                                break;
                            case 13:
                                heading = 0;
                                offsets = A13Offsets;
                                break;
                            case 15:
                                heading = -90;
                                offsets = A15Offsets;
                                break;
                        }
                        break;
                    case RED:
                        switch (tag) {
                            case 12:
                                heading = 90;
                                offsets = A12Offsets;
                                break;
                            case 14:
                                heading = 0;
                                offsets = A14Offsets;
                                break;
                            case 15:
                                double yaw = vision.Position.getPose(PoseTypes.MT1).getOrientation().getYaw(AngleUnit.DEGREES);
                                if (yaw > 180) {
                                    yaw -= 360;
                                }

                                if (yaw > -90 && yaw < 90) {
                                    heading = 0;
                                } else if (yaw < -90 || yaw > 90) {
                                    heading = 180;
                                }

                                offsets = A15Offsets;
                                break;
                            case 16:
                                heading = 180;
                                offsets = A16Offsets;
                                break;
                        }
                        break;
                }
            }
            pinpoint.updateHeading(heading);
            pinpoint.update();

            vision.Position.updateYaw();
            vision.getResult();

            Position MT2 = vision.Position.getPose(PoseTypes.MT2).getPosition().toUnit(DistanceUnit.INCH);
            pinpoint.updateXYPosition(MT2.x + offsets.getXTotalOffsets(), MT2.y + offsets.getYTotalOffsets());

            updatePose();
            return getPose();
        }

        return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    }

//    public Pose2D getPose() {
//        if (!LLInUse) {
//            vision.setPipeline(3);
//            while (!vision.lastResultValid() || vision.Position.getTagCount() == 0) {
//                vision.getResult();
//            }
//
//            double heading = 0;
//
//            int tagCount = vision.Position.getTagCount();
//            if (tagCount == 2) {
//                heading = vision.Position
//            }
//        }
//    }

    public void updateHeading() {
        if (!LLInUse) {
            vision.setPipeline(3);
            vision.getResult();

            if (vision.lastResultValid()) {
                int tagCount = vision.Position.getTagCount();
                if (tagCount == 2) {
                    double heading = vision.Position.getPose(PoseTypes.MT1).getOrientation().getYaw(AngleUnit.DEGREES);
                    if (heading > 180) {
                        heading -= 360;
                    }
                    pinpoint.updateHeading(heading);
                    resultType = PoseHelperResultTypes.MT2Tag;
                }
            }
        }
    }

    public void syncPose() {
        if (!LLInUse) {
            vision.setPipeline(3);
            vision.getResult();

            if (vision.lastResultValid()) {
                int tagCount = vision.Position.getTagCount();
                if (tagCount == 2) {
                    resultType = PoseHelperResultTypes.MT2Tag;
                } else {
                    resultType = PoseHelperResultTypes.MT;
                }
                Position MT2Position = vision.Position.getPose(PoseTypes.MT2).getPosition().toUnit(DistanceUnit.INCH);
                pinpoint.updateXYPosition(MT2Position.x, MT2Position.y);
            } else {
                resultType = PoseHelperResultTypes.PINPOINT;
            }
        } else {
            resultType = PoseHelperResultTypes.PINPOINT;
        }
    }

    public void updatePose() {
        pinpoint.update();
        pose = ShortenXY(pinpoint.getPosition());
    }

    private Pose2D ShortenXY(Pose2D oldPose) {
        try {
            double x = BigDecimal.valueOf(oldPose.getX(DistanceUnit.INCH)).setScale(4, RoundingMode.DOWN).doubleValue() + 72;
            double y = BigDecimal.valueOf(oldPose.getY(DistanceUnit.INCH)).setScale(4, RoundingMode.DOWN).doubleValue() + 72;
            double heading = oldPose.getHeading(AngleUnit.DEGREES);

            return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
        } catch (Exception e) {
            return oldPose;
        }
    }

    public Pose2D getPose() {
        return pose;
    }
}
