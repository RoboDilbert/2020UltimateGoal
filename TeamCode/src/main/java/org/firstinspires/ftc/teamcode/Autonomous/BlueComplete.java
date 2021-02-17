package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Wobble;
import org.firstinspires.ftc.teamcode.TensorFlowClass;
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

import java.util.List;
import java.util.Locale;

@Autonomous(name= "BlueComplete", group= "Autonomous")
public class BlueComplete extends LinearOpMode{

    public Shooter autoShooter;
    public Intake autoIntake;
    public Rolling Distance1 = new Rolling(20);

    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;

    public void runOpMode() throws InterruptedException {

        DriveTrain.initDriveTrain(hardwareMap);
        Shooter.initShooter(hardwareMap);
        Intake.initIntake(hardwareMap);
        Wobble.initWobble(hardwareMap);
        TensorFlowClass.initTensorFlow(hardwareMap);


//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        pipeline = new SkystoneDeterminationPipeline();
//        webcam.setPipeline(pipeline);

        //Shooter.angleAdjust.setPosition(0.5);

//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
//            }
//        });

        waitForStart();


        //Read the stack

        //--------0----------
        if(TensorFlowClass.getLabel().equals("ZERO") || TensorFlowClass.getLabel() == null) {
            //Drive forward to white line
            Shooter.shoot(Constants.SHOOTER_POWER);
            sleep(100);
            DriveTrain.cartesianDriveTimer(0, -0.6, 42);
            sleep(1000);
            DriveTrain.driveToLine("WHITE");
            sleep(100);

            //Shoot
            Shooter.setPosition("WHITE_LINE");
            sleep(100);
            DriveTrain.cartesianDriveTimer(0, 0.3, 40);
            sleep(100);
            Intake.releaseAll();
            Shooter.shoot(0);
            sleep(100);
            Shooter.setPosition("INDEX");
            sleep(100);

            //Strafe over to wall
            DriveTrain.cartesianDriveTimer(0.8, 0, 70);

            //Drop wobb
            Wobble.drop();

            //backup along wall with timer
            DriveTrain.cartesianDriveTimer(0, 0.6, 21);
            sleep(1000);

            //strafe over until change
            DriveTrain.cartesianDriveDistance(-0.45, 0, "RED");

            //Use distance sensor to measure distance to wobble

            //Pick it up

            //Strafe and turn at the same time
            //Drop off wobble
            //Get to line
        }

        else if(TensorFlowClass.getLabel().equals("Single")) {
            //--------1-----------
            //Strafe Left
            DriveTrain.cartesianDriveTimer(-0.6, 0, 10);

            //Drive Forward to ring
            Shooter.shoot(Constants.SHOOTER_POWER);
            Shooter.setPosition("RINGS");
            DriveTrain.cartesianDriveRing(0, -0.4);

            //Shoot our shot
            Intake.releaseAll();

            //Turn on intake
            Intake.intake();

            //Drive forward to white line
//        DriveTrain.cartesianDriveTimer(0, -0.6 , 42);
//        sleep(1000);
            DriveTrain.driveToLine("WHITE");
            sleep(100);

            //Shoot
            Shooter.setPosition("WHITE_LINE");
            sleep(100);
            DriveTrain.cartesianDriveTimer(0, 0.3, 40);
            sleep(100);
            Intake.releaseAll();
            Shooter.shoot(0);
            Shooter.setPosition("INDEX");
            sleep(100);

            //Drive Forward to red Line
            DriveTrain.driveToLine("RED");

            //Drop wobb
            Wobble.drop();

            //Strafe to wall
            DriveTrain.cartesianDriveTimer(-0.6, 0, 40);

            //backup along wall with timer
            DriveTrain.cartesianDriveTimer(0, 0.6, 25);
            sleep(100);

            //strafe over until change
            DriveTrain.cartesianDriveDistance(-0.45, 0, "RED");

            //Use distance sensor to measure distance to wobble

            //Pick it up

            //Strafe and turn at the same time
            //Drop off wobble
            //Rotate
            //Get to line
        }

        else if(TensorFlowClass.getLabel().equals("Quad")) {
            //------------4------------
            //Strafe Left
            DriveTrain.cartesianDriveTimer(-0.6, 0, 10);

            //Drive Forward to ring
            Shooter.shoot(Constants.SHOOTER_POWER);
            Shooter.setPosition("RINGS");
            DriveTrain.cartesianDriveRing(0, -0.4);

            //Shoot our shot
            Intake.releaseAll();

            //Turn on intake
            Intake.intake();

            //Drive forward to white line
//        DriveTrain.cartesianDriveTimer(0, -0.6 , 42);
//        sleep(1000);
            //TODO MAKE SURE WE DON"T GET FOUR (Splay out rings, pick up 3, stop intake with sensor, then shoot and pick up the last one hopefully)
            DriveTrain.driveToLine("WHITE");
            sleep(100);

            //Shoot
            Shooter.setPosition("WHITE_LINE");
            sleep(100);
            DriveTrain.cartesianDriveTimer(0, 0.3, 40);
            sleep(100);
            Intake.releaseAll();
            Shooter.shoot(0);
            Shooter.setPosition("INDEX");
            sleep(100);

            //Drive to white line again
            DriveTrain.driveToLine("WHITE");

            //Strafe over to wall
            DriveTrain.cartesianDriveTimer(0.8, 0, 70);

            //Drive Forward to 2 red Lines
            DriveTrain.driveToLine("RED");
            sleep(100);
            DriveTrain.driveToLine("RED");

            //Drop wobb
            Wobble.drop();

            //backup along wall with timer
            DriveTrain.cartesianDriveTimer(0, 0.6, 35);
            sleep(100);

            //strafe over until change
            DriveTrain.cartesianDriveDistance(-0.45, 0, "RED");

            //Use distance sensor to measure distance to wobble

            //Pick it up

            //Strafe and turn at the same time
            //Drop off wobble
            //Rotate
            //Get to line
        }


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