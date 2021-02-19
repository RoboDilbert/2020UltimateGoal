package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Vuforia;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.MasterVision;
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
    public String label;

    private static MasterVision vision;
    private static VuforiaLocalizer.Parameters parameters;
    private static TFObjectDetector tfod;
    private static VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY = "AW/D0F3/////AAABmT6CO76ZukEWtNAvh1kty819QDEF9SG9ZxbfUcbjoxBCe0UcoTGK19TZdmHtWDwxhrL4idOt1tdJE+h9YGDtZ7U/njHEqSZ7jflzurT4j/WXTCjeTCSf0oMqcgduLTDNz+XEXMbPSlnHbO9ZnEZBun7HHr6N06kpYu6QZmG6WRvibuKCe5IeZJ21RoIeCsdp3ho/f/+QQLlnqaa1dw6i4xMFM0e2IaxujhQiWnd4by23CkMPvzKhy6YP3wPBq+awpzEPLDZcD8l1i0SqmX7HNpmw4kXBrWzEimAzp1aqONVau4kIwCGwJFusMdErw9IL7KQ5VqMKN4Xl67s0pwotoXsA+5SlWQAIodipYKZnPzwO";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static List<Recognition> tfodRecogntions;

    public void runOpMode() throws InterruptedException {

        DriveTrain.initDriveTrain(hardwareMap);
        Shooter.initShooter(hardwareMap);
        Intake.initIntake(hardwareMap);
        Wobble.initWobble(hardwareMap);

        initVuforia();
        initTfod(hardwareMap);
        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_NONE);
        vision.enable();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }

        sleep(5500);

        tfodRecogntions = tfod.getUpdatedRecognitions();

        for (Recognition recognition : tfodRecogntions) {
            if(recognition == null){
                label = "ZERO";
            }
            else{
                label = recognition.getLabel();
            }
        }

        telemetry.addData("Rings: ", label);
        telemetry.update();

        int timer = 0;
        while(timer < 1000000) {
            sleep(3000);

            tfodRecogntions = tfod.getUpdatedRecognitions();

            for (Recognition recognition : tfodRecogntions) {
                if(recognition == null){
                    label = "ZERO";
                }
                else{
                    label = recognition.getLabel();
                }
            }

            telemetry.addData("Rings: ", label);
            telemetry.update();
        }

        waitForStart();
//
//        sleep(100);
//        //Read the stack
//
//        //--------0----------
//        if(label == null || label.equals("ZERO")) {
//            //Drive forward to white line
//            Shooter.shoot(Constants.SHOOTER_POWER);
//            sleep(100);
//            DriveTrain.cartesianDriveTimer(0, -0.6, 42);
//            sleep(1000);
//            DriveTrain.driveToLine("WHITE");
//            sleep(100);
//
//            //Shoot
//            Shooter.setPosition("WHITE_LINE");
//            sleep(100);
//            DriveTrain.cartesianDriveTimer(0, 0.3, 40);
//            sleep(100);
//            Intake.releaseAll();
//            Shooter.shoot(0);
//            sleep(100);
//            Shooter.setPosition("INDEX");
//            sleep(100);
//
//            //Strafe over to wall
//            DriveTrain.cartesianDriveTimer(0.8, 0, 70);
//
//            //Drop wobb
//            Wobble.drop();
//
//            //backup along wall with timer
//            DriveTrain.cartesianDriveTimer(0, 0.6, 21);
//            sleep(1000);
//
//            //strafe over until change
//            DriveTrain.cartesianDriveDistance(-0.45, 0, "RED");
//
//            //Use distance sensor to measure distance to wobble
//
//            //Pick it up
//
//            //Strafe and turn at the same time
//            //Drop off wobble
//            //Get to line
//        }
//
//        else if(label.equals("Single")) {
//            //--------1-----------//
//            //Strafe Left
//            DriveTrain.cartesianDriveTimer(-0.6, 0, 10);
//
//            //Drive Forward to ring
//            Shooter.shoot(Constants.SHOOTER_POWER);
//            Shooter.setPosition("RINGS");
//            DriveTrain.cartesianDriveRing(0, -0.4);
//
//            //Shoot our shot
//            Intake.releaseAll();
//            //Turn on intake
//            Intake.intake();
//
//            //Drive forward to white line
////        DriveTrain.cartesianDriveTimer(0, -0.6 , 42);
////        sleep(1000);
//            DriveTrain.driveToLine("WHITE");
//            sleep(100);
//
//            //Shoot
//            Shooter.setPosition("WHITE_LINE");
//            sleep(100);
//            DriveTrain.cartesianDriveTimer(0, 0.3, 40);
//            sleep(100);
//            Intake.releaseAll();
//            Shooter.shoot(0);
//            Shooter.setPosition("INDEX");
//            sleep(100);
//
//            //Drive Forward to red Line
//            DriveTrain.driveToLine("RED");
//
//            //Drop wobb
//            Wobble.drop();
//
//            //Strafe to wall
//            DriveTrain.cartesianDriveTimer(-0.6, 0, 40);
//
//            //backup along wall with timer
//            DriveTrain.cartesianDriveTimer(0, 0.6, 25);
//            sleep(100);
//
//            //strafe over until change
//            DriveTrain.cartesianDriveDistance(-0.45, 0, "RED");
//
//            //Use distance sensor to measure distance to wobble
//
//            //Pick it up
//
//            //Strafe and turn at the same time
//            //Drop off wobble
//            //Rotate
//            //Get to line
//        }
//
//        else if(label.equals("Quad")) {
//            //------------4------------
//            //Strafe Left
//            DriveTrain.cartesianDriveTimer(-0.6, 0, 10);
//
//            //Drive Forward to ring
//            Shooter.shoot(Constants.SHOOTER_POWER);
//            Shooter.setPosition("RINGS");
//            DriveTrain.cartesianDriveRing(0, -0.4);
//
//            //Shoot our shot
//            Intake.releaseAll();
//
//            //Turn on intake
//            Intake.intake();
//
//            //Drive forward to white line
////        DriveTrain.cartesianDriveTimer(0, -0.6 , 42);
////        sleep(1000);
//            //TODO MAKE SURE WE DON"T GET FOUR (Splay out rings, pick up 3, stop intake with sensor, then shoot and pick up the last one hopefully)
//            DriveTrain.driveToLine("WHITE");
//            sleep(100);
//
//            //Shoot
//            Shooter.setPosition("WHITE_LINE");
//            sleep(100);
//            DriveTrain.cartesianDriveTimer(0, 0.3, 40);
//            sleep(100);
//            Intake.releaseAll();
//            Shooter.shoot(0);
//            Shooter.setPosition("INDEX");
//            sleep(100);
//
//            //Drive to white line again
//            DriveTrain.driveToLine("WHITE");
//
//            //Strafe over to wall
//            DriveTrain.cartesianDriveTimer(0.8, 0, 70);
//
//            //Drive Forward to 2 red Lines
//            DriveTrain.driveToLine("RED");
//            sleep(100);
//            DriveTrain.driveToLine("RED");
//
//            //Drop wobb
//            Wobble.drop();
//
//            //backup along wall with timer
//            DriveTrain.cartesianDriveTimer(0, 0.6, 35);
//            sleep(100);
//
//            //strafe over until change
//            DriveTrain.cartesianDriveDistance(-0.45, 0, "RED");
//
//            //Use distance sensor to measure distance to wobble
//
//            //Pick it up
//
//            //Strafe and turn at the same time
//            //Drop off wobble
//            //Rotate
//            //Get to line
//        }


    }
    public static void initVuforia() {

        parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    public static void initTfod(HardwareMap hardwareMap) {

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.timingBufferSize = 1;
        tfodParameters.maxFrameRate = 100;
        tfodParameters.minResultConfidence = 0.8f; // minimum confidence in object detection 80%
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}