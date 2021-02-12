package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Rolling;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.Locale;

@Autonomous(name= "BlueComplete", group= "Autonomous")

public class BlueComplete extends LinearOpMode{

    public Shooter autoShooter;
    public Intake autoIntake;
    public Rolling Distance1 = new Rolling(20);

    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;

    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        autoIntake = new Intake();

        Shooter.angleAdjust.setPosition(0.5);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();

        Shooter.mainShooter.shoot(0.58);

        Constants.drive.setRunMode("STOP_AND_RESET_ENCODER");
        Constants.drive.setRunMode("RUN_USING_ENCODER");

        Thread.sleep(50);
        DriveTrain.floorColorSensor.enableLed(true);

        webcam.closeCameraDevice();

        //TODO Shoot the preloaded rings

        pipeline.position = SkystoneDeterminationPipeline.RingPosition.FOUR;

        telemetry.addData("Position", pipeline.position);
        telemetry.addData("driveDistanecSensor", String.format("%.3f cm", DriveTrain.driveDistanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("Red", "%.3f", (double) DriveTrain.floorColorSensor.red());
        telemetry.addData("Average of ouh", pipeline.avg1);
        telemetry.update();

        //Drive forward and pick up ringos
        if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE || pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE){
            //TODO Spin intake motor
            Constants.drive.DriveToLine("WHITE");
            sleep(50);
            Constants.drive.Drive("REVERSE", 400, 0.3);
            sleep(50);
        }
        else{
            Constants.drive.Drive("FORWARD", 1000, 0.3);
            sleep(100);
            //TODO Spin Intake
            Constants.drive.Drive("FORWARD", 300, 0.15);
            telemetry.addData("Status", "Lineyo");
            telemetry.update();
            //TODO add color sensor for rings
            Constants.drive.Drive("FORWARD", 1000, 0.3);
            sleep(100);
            Constants.drive.setRunMode("RUN_USING_ENCODER");
            sleep(50);
            Constants.drive.DriveToLine("WHITE");
            //TODO Spin Intake
            Constants.drive.Drive("FORWARD", 300, 0.15);
            telemetry.addData("Status", "Lineyo");
            telemetry.update();
            //TODO add color sensor for rings
            sleep(100);
            Constants.drive.setRunMode("RUN_USING_ENCODER");
            sleep(50);
            Constants.drive.DriveToLine("WHITE");
            sleep(100);
            Constants.drive.Drive("REVERSE", 400, 0.3);
            sleep(50);
        }

        //Shoot extra picked up rings

        //Drive to designated location
        if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE){
            telemetry.addData("Status", "no blocko");
            telemetry.update();
            Constants.drive.setRunMode("RUN_USING_ENCODER");
            Constants.drive.DriveToLine("WHITE");
            Shooter.angleAdjust.setPosition(0.51);
            autoIntake.shootAllNoClear();
            sleep(100);
            Constants.drive.Drive("REVERSE", 200, 0.3);
            sleep(100);
            Constants.drive.Drive("STRAFE_LEFT", 1200, 0.4);
            sleep(100);
//            robot.grabber.setPosition(.5);
            sleep(100);
        }
        else if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE){
            telemetry.addData("Status", "blocko is 1'0");
            telemetry.update();
            Constants.drive.driveToRing(0.3);
            Shooter.angleAdjust.setPosition(0.5);
            autoIntake.shootAllNoClear();
            sleep(100);
            autoIntake.intake();
            Constants.drive.setRunMode("RUN_USING_ENCODER");
            Constants.drive.DriveToLine("WHITE");
            Shooter.angleAdjust.setPosition(0.51);
            autoIntake.releaseAll();
            sleep(100);
            Constants.drive.setRunMode("RUN_USING_ENCODER");
            Constants.drive.DriveToLine("RED");
            Thread.sleep(100);
//            robot.grabber.setPosition(.5);
            sleep(100);
        }
        else if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR){
            telemetry.addData("Status", "blocko is 4'0");
            telemetry.update();
            Constants.drive.driveToRing(0.3);
            Shooter.angleAdjust.setPosition(0.5);
            autoIntake.shootAllNoClear();
            sleep(100);
            autoIntake.intake();
            Constants.drive.setRunMode("RUN_USING_ENCODER");
            Constants.drive.DriveToLine("WHITE");
            Shooter.angleAdjust.setPosition(0.51);
            autoIntake.releaseAll();
            sleep(100);
            Constants.drive.Drive("FORWARD_LEFT", 250, .2);
            sleep(100);
            Constants.drive.setRunMode("RUN_USING_ENCODER");
            Constants.drive.DriveToLine("RED");
            sleep(100);
//            robot.grabber.setPosition(.5);
            sleep(100);
        }

        //Drop boyo


        //Backup to white line


        if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE){
            Constants.drive.Drive("STRAFE_LEFT", 1200, 0.4);
            Thread.sleep(100);
        }


        Constants.drive.Drive("REVERSE", 2500, 0.4);

        //left off with it indexing wrong

        Constants.drive.Drive("STRAFE_RIGHT" , 500, .3);

        //takes too long to index
//        while(Distance1.index < 10){
//            Distance1.add(DriveTrain.pewpewboi.getDistance(DistanceUnit.CM));
//            telemetry.addData("Average Distance indexing", Distance1.getAverage());
//            telemetry.update();
//        }
//        //doesnt read distance up close
//        while(Distance1.getAverage() > 15){
//            Distance1.add(DriveTrain.pewpewboi.getDistance(DistanceUnit.CM));
//            telemetry.addData("Average Distance driving", Distance1.getAverage());
//            telemetry.update();
//            Constants.drive.setRunMode("RUN_USING_ENCODER");
//            Constants.drive.leftFrontMotor.setPower(-0.4);
//            Constants.drive.leftBackMotor.setPower(-0.4);
//            Constants.drive.rightFrontMotor.setPower(-0.4);
//            Constants.drive.rightBackMotor.setPower(-0.4);
//        }




        if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE){
            Constants.drive.Drive("STRAFE_LEFT", 1200, 0.4);
            Thread.sleep(100);
        }


        //left off with it indexing wrong




        if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE){
            Constants.drive.Drive("REVERSE", 1000, 0.4);
        }
        else if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE){
            Constants.drive.Drive("REVERSE", 1750, 0.4);
        }
        else if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR){
            Constants.drive.Drive("REVERSE", 2500, 0.4);
        }


        Constants.drive.Drive("STRAFE_RIGHT" , 500, .3);

//        while(Distance1.index < 5){
//            Distance1.add(DriveTrain.pewpewboi.getDistance(DistanceUnit.CM));
//            telemetry.addData("Average Distance indexing", Distance1.getAverage());
//            telemetry.update();
//        }

//        while(Distance1.getAverage() > 10){
//            Distance1.add(DriveTrain.pewpewboi.getDistance(DistanceUnit.CM));
//            telemetry.addData("Average Distance driving", Distance1.getAverage());
//            telemetry.update();
//            Constants.drive.setRunMode("RUN_USING_ENCODER");
//            Constants.drive.leftFrontMotor.setPower(-0.4);
//            Constants.drive.leftBackMotor.setPower(-0.4);
//            Constants.drive.rightFrontMotor.setPower(-0.4);
//            Constants.drive.rightBackMotor.setPower(-0.4);
//        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(138, 110);

        static final int REGION_WIDTH = 44;
        static final int REGION_HEIGHT = 60;

        final int FOUR_RING_THRESHOLD = 140;
        final int ONE_RING_THRESHOLD = 127;//135

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile SkystoneDeterminationPipeline.RingPosition position = SkystoneDeterminationPipeline.RingPosition.NONE;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            //position = SkystoneDeterminationPipeline.RingPosition.NONE; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = SkystoneDeterminationPipeline.RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = SkystoneDeterminationPipeline.RingPosition.ONE;
            } else {
                position = SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
    }
}