//package org.firstinspires.ftc.teamcode.OpenCVTest;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.Util.HardwarePresets;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import org.firstinspires.ftc.teamcode.OpenCVTest.*;
//
//
///**
// * This is an example of how to use the ExamplePipeLine
// * For more examples such as using a webcam and switching between different cameras
// * visit https://github.com/OpenFTC/EasyOpenCV/tree/master/examples/src/main/java/org/openftc/easyopencv/examples
// */
//@TeleOp
//@Disabled
//public class Camera extends HardwarePresets {
//
//    UGRectDetector webcam;
//    UGRectRingPipeline visionPipeLine;
//
//    @Override
//    public void runOpMode() {
//
//        webcam = new UGRectDetector(HardwarePresets.HwMap, "webcam");
//
//        webcam.init();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            telemetry.addData("Frame Count", webcam.camera.getFrameCount());
//            telemetry.addData("FPS", String.format("%.2f", webcam.camera.getFps()));
//            telemetry.addData("Total frame time ms", webcam.camera.getTotalFrameTimeMs());
//            telemetry.addData("Pipeline time ms", webcam.camera.getPipelineTimeMs());
//            telemetry.addData("Overhead time ms", webcam.camera.getOverheadTimeMs());
//            telemetry.addData("Theoretical max FPS", webcam.camera.getCurrentPipelineMaxFps());
//            telemetry.addData("Ring Amount", webcam.getStack());
//            telemetry.update();
//
//            //webcam.process
//
//
//            if(gamepad1.a) {
//               webcam.camera.stopStreaming();
//                //webcam.closeCameraDevice();
//            }
//            else if(gamepad1.x) {
//                webcam.camera.pauseViewport();
//            }
//            else if(gamepad1.y) {
//                webcam.camera.resumeViewport();
//            }
//
//            sleep(100);
//        }
//    }
//}
