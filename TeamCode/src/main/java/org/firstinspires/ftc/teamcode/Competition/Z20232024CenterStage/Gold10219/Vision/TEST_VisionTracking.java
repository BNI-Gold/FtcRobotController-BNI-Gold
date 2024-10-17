package org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Vision;

//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
////import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.vision.VisionPortal;
////import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//import java.util.List;
//
//@Disabled
//@TeleOp (name = "TEST_Vision Tracking Center Stage")
//public class TEST_VisionTracking extends LinearOpMode {
//
////
////    public static final boolean USE_WEBCAM = true;
////
////  //  public TfodProcessor tFod;
////
////    public VisionPortal visionPortal;
////
////
////    @Override
////    public void runOpMode(){
////
////        initTfod();
////
////        waitForStart();
////
////        if (opModeIsActive()) {
////            while ( opModeIsActive()) {}
////
////            telemetrytFod();
////
////            telemetry.update();
////
////            if (gamepad1.dpad_down) {
////                visionPortal.stopStreaming();
////            } else if (gamepad1.dpad_up) {
////                visionPortal.resumeStreaming();
////            }
////        }
////    }
////
////
////    public void initTfod() {
////
////
////   //     tFod = TfodProcessor.easyCreateWithDefaults();
////
////
////        if (USE_WEBCAM) {
////            visionPortal = VisionPortal.easyCreateWithDefaults(
////                    hardwareMap.get(WebcamName.class, "Webcam 1"), tFod);
////        } else {
////            visionPortal = VisionPortal.easyCreateWithDefaults(
////                    BuiltinCameraDirection.BACK, tFod);
////        }
////
////    }
////
////    //Hello Connor.  I am messing with your code because I AM THE LEAD PROGRAMMER... Camden
////    public void telemetrytFod(){
////  //      List<Recognition> currentRecognitions = tFod.getRecognitions();
////     //   telemetry.addData("# Objects Detected", currentRecognitions.size());
////
////
//////        for (Recognition recognition : currentRecognitions) {
//////            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//////            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
////
//////            telemetry.addData(""," ");
//////            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//////            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//////            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
////        }
////    }
//}
