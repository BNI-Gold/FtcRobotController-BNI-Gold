package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.Vision.Pipelines;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.Vision.SamplePipelineResult;

import java.util.ArrayList;

public class Vision {
    public HardwareMap hwBot = null;
    public Limelight3A cam = null;
    public LinearOpMode LinearOp = null;
    public IMU imu = null;

    public double errorMultiplier = 0.025;
    public double errorOffset = 4;
    public double minimumCommand = 0.1;
    public double maximumCommand = 0.4;

    public boolean captureSnapshots = false;
    public int snapshotLimit = 0;
    public String snapshotPrefix = "pov_";
    private int numSnapshots = 0;

    private LLResult result = null;

    public Vision() {}

    public void setLinearOp(LinearOpMode LinearOp) {this.LinearOp = LinearOp;}

    public void initVision(HardwareMap hwMap, IMU imu, boolean captureSnapshots, int snapshotLimit, String snapshotPrefix) {
        hwBot = hwMap;
        this.imu = imu;
        this.captureSnapshots = captureSnapshots;
        this.snapshotLimit = snapshotLimit;
        this.snapshotPrefix = snapshotPrefix;

        cam = hwBot.get(Limelight3A.class, "limelight");
        cam.pipelineSwitch(0);
    }

    public void start() {
        cam.start();

        LinearOp.telemetry.addLine("Vision Started");
        LinearOp.telemetry.update();
    }

    public void setPipeline(int pipeline) {
        cam.pipelineSwitch(pipeline);
    }

    public void getResult() {
        result = cam.getLatestResult();
    }

    public boolean lastResultValid() {
        return result != null && result.isValid();
    }

    public Pose3D getPose() {
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw();
        cam.updateRobotOrientation(robotYaw);
        if (result != null) {
            return result.getBotpose_MT2();
        }
        return null;
    }

    public double[] getOffsets() {
        if (result != null && result.isValid()) {
            if (captureSnapshots && numSnapshots < snapshotLimit) {
                captureSnapshot();
                numSnapshots ++;
            }

            double[] coords = new double[3];
            coords[0] = result.getTx();
            coords[1] = result.getTy();
            coords[2] = result.getTa();
            return coords;
        }
        return null;
    }

    public int getClosestPipeline(Pipelines[] pipelines) {
        //Define arraylist of sample pipeline results
        ArrayList<SamplePipelineResult> results = new ArrayList<SamplePipelineResult>();

        //For each desired pipeline, determine tx, ty, and ta values, then store in arrayList
        for (Pipelines pipeline : pipelines) {
            int pipelineId = 0;
            switch (pipeline) {
                case RED:
                    break;
                case YELLOW:
                    pipelineId = 1;
                    break;
                case BLUE:
                    pipelineId = 2;
                    break;
            }
            //Set pipeline
            setPipeline(pipelineId);

            //Get pipeline result
            getResult();

            if (result != null && result.isValid()) {
                //Store double values
                double tx = getOffsets()[0];
                double ty = getOffsets()[1];
                double ta = getOffsets()[2];

                //Instantiate new result object with tx, ty, ta, and pipeline id
                SamplePipelineResult result = new SamplePipelineResult(tx, ty, ta, pipelineId);

                //Add result to array
                results.add(result);
            }
        }

//        LinearOp.telemetry.update();

        //For each result, determine smallest ta value, then output pipeline id
        double closestTa = 0;
        int closestPipeline = 0;
        for (SamplePipelineResult result : results) {
            if (result.ta > closestTa) {
                closestTa = result.ta;
                closestPipeline = result.pipeline;
            }
        }

        return closestPipeline;
    }

    public void captureSnapshot() {
        LLResult res = cam.getLatestResult();
        if (res != null && res.isValid()) {
            cam.captureSnapshot("pov_" + snapshotPrefix);
        }
    }

    public void stop() {
        cam.stop();
    }
}
