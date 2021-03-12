package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MasterVision;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Wobble;


@Autonomous(name= "BlueCorner", group= "Autonomous")
public class BlueCorner extends LinearOpMode{
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

        Shooter.updateShooterConstants(50, 1, 2,0);

        initVuforia();
        initTfod(hardwareMap);
        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_NONE);
        vision.enable();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }

        sleep(500);

        tfodRecogntions = tfod.getUpdatedRecognitions();

        for (Recognition recognition : tfodRecogntions) {
            label = recognition.getLabel();
        }

        if(tfodRecogntions.isEmpty()){
            label = "ZERO";
        }

        telemetry.addData("Rings: ", label);
        telemetry.update();

        while(!isStarted()) {
            sleep(500);

            tfodRecogntions = tfod.getUpdatedRecognitions();


            for (Recognition recognition : tfodRecogntions) {
                label = recognition.getLabel();
            }

            if(tfodRecogntions.isEmpty()){
                label = "ZERO";
            }

            telemetry.addData("Rings: ", label);
            telemetry.update();
        }

        waitForStart();

        if(label == null || label.equals("ZERO")) {
            //Spin up flywheel
            Shooter.setPosition("WHITE_LINE");
            sleep(100);
            Shooter.shoot(Shooter.SHOOTER_POWER);

            //DriveTrain.cartesianDriveTimer(-.85, 0, 30);
            //Drive forward to blue line and drop wobble
            DriveTrain.cartesianDriveTimer(0, -0.6, 42);
            DriveTrain.cartesianDriveTimer(0, 0.7, 3);

            //Strafe to wall
            //DriveTrain.cartesianDriveTimer(.85, 0, 25);
            //Drive forward to blue line and drop wobble
            DriveTrain.driveToLine(.2, "BLUE", telemetry);

            DriveTrain.cartesianDriveTimer(0, -.3, 8);

            Wobble.drop();
            sleep(100);
            DriveTrain.cartesianDriveTimer(0, .3, 12);
//            //Rotate a little bit to shoot
//            DriveTrain.cartesianDriveTimer(0, .4, 5);
//
//            DriveTrain.gyroTurn(-Math.PI/6, 130);
//            //Shoot
//            Intake.releaseAll();
            DriveTrain.cartesianDriveTimer(-.85, 0, 30);
            //Shoot
            Intake.releaseAll();
//            Intake.intake();

            Shooter.shoot(0);
            //Park
            DriveTrain.cartesianDriveTimer(-.85, -.2, 40);

            //Close wobble claw
            Wobble.close();
        }

        else if(label.equals("Single")) {
            //Spin up flywheel
            Shooter.setPosition("WHITE_LINE");
            Shooter.shoot(Shooter.SHOOTER_POWER);

            //Strafe to wall
            //DriveTrain.cartesianDriveTimer(.85, 0, 15);
            //Drive forward to blue line and drop wobble
            DriveTrain.cartesianDriveTimer(0, -0.6, 42);
            DriveTrain.cartesianDriveTimer(0, 0.7, 3);

            DriveTrain.driveToLine(.2, "BLUE", telemetry);

            DriveTrain.cartesianDriveTimer(-.8, 0, 40);
            //Shoot
            sleep(200);
            Intake.releaseAll();
            Intake.intake();

            DriveTrain.cartesianDriveTimer(.8, 0, 15);
            sleep(100);
            DriveTrain.cartesianDriveTimer(0, 0.5, 32);
            DriveTrain.cartesianDriveTimer(0, -0.5, 22);
            sleep(200);

            DriveTrain.cartesianDriveTimer(-.8, 0, 15);

            Intake.releaseAll();
            Intake.stop();

            Shooter.shoot(0);
            //Park
            DriveTrain.cartesianDriveTimer(0, -.4, 44);

            Wobble.drop();

            DriveTrain.cartesianDriveTimer(-.85, 0.15, 35);

            Wobble.close();
        }

        else if(label.equals("Quad")) {
            Shooter.setPosition("WHITE_LINE");
            Shooter.shoot(Shooter.SHOOTER_POWER);

            //Strafe to wall
            //DriveTrain.cartesianDriveTimer(.85, 0, 15);
            //Drive forward to blue line and drop wobble
            DriveTrain.cartesianDriveTimer(0, -0.5, 80);
            DriveTrain.cartesianDriveTimer(0, 0.7, 2);

            DriveTrain.driveToLine(.2, "BLUE", telemetry);

            Wobble.drop();

            DriveTrain.cartesianDriveTimer(0, 0.6, 25);
            DriveTrain.cartesianDriveTimer(0, -0.7, 2);


            DriveTrain.driveToLine(-.2, "WHITE", telemetry);

            DriveTrain.cartesianDriveTimer(-.85, 0, 29);
            //Shoot
            Intake.releaseAll();
            Intake.intake();

            DriveTrain.cartesianDriveDistance(0.7,0,"LEFT", telemetry, "GREATER");

//            DriveTrain.cartesianDriveTimer(.85, 0, 12);

            DriveTrain.cartesianDriveTimer(0, 0.3, 45);
            DriveTrain.cartesianDriveTimer(0, -0.7, 2);
            DriveTrain.cartesianDriveTimer(0, 0.3, 20);


            sleep(200);

            Intake.releaseAll();
            sleep(200);

            DriveTrain.cartesianDriveTimer(0, 0.3, 35);
            DriveTrain.cartesianDriveTimer(0, -0.7, 2);
            DriveTrain.cartesianDriveTimer(0, 0.3, 20);
            DriveTrain.cartesianDriveTimer(0, -0.5, 35);
            sleep(200);

            Intake.releaseAll();
            Intake.stop();

            Shooter.shoot(0);

            DriveTrain.driveToLine(.3, "WHITE", telemetry);
            //Park
//            DriveTrain.cartesianDriveTimer(0.65, -0.5, 50);
//
//            Wobble.drop();

            DriveTrain.cartesianDriveTimer(-.85, 0, 37);
        }

        DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");
        while(opModeIsActive()) {
          DriveTrain.SUMO_MODE();
        }


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
        tfodParameters.minResultConfidence = 0.5f; // minimum confidence in object detection 80%
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
