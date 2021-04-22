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
import org.firstinspires.ftc.teamcode.UtilOG.Rolling;

//@Autonomous(name= "BlueComplete", group= "Autonomous")
public class BlueComplete extends LinearOpMode{

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

        //--------0----------
        if(label == null || label.equals("ZERO")) {
            //Turn on shooter
            Shooter.shoot(Shooter.SHOOTER_POWER);

            //Drive forward to white line
            DriveTrain.driveToLine(.3,"WHITE", telemetry);
            sleep(100);

            //Shoot
            Shooter.setPosition("WHITE_LINE");
            DriveTrain.cartesianDriveTimer(-0.7, 0, 10);
            sleep(100);

            DriveTrain.cartesianDriveTimer(0, 0.3, 30);
            sleep(100);

            Intake.releaseAll();
            Intake.defaultPos();
            Shooter.shoot(0);
            Shooter.setPosition("INDEX");
            sleep(100);

            //Move forward slightly to white line to drop wobble into zone
            DriveTrain.cartesianDriveTimer(0, -0.3, 1);
            sleep(100);

            //Strafe over to wall
            DriveTrain.cartesianDriveDistance(0.8, 0, "LEFT", telemetry,"GREATER");
            sleep(100);

            DriveTrain.cartesianDriveTimer(0, -0.4, 10);

            //Drop wobb
            Wobble.drop();

            //Backup along wall with timer
            DriveTrain.cartesianDriveTimer(0, 0.6, 15);
            sleep(100);
        }

        else if(label.equals("Single")) {
            //--------1-----------//

            //Drive Forward to ring
            Shooter.shoot(Shooter.SHOOTER_POWER);
            Shooter.setPosition("WHITE_LINE");
            //sleep(500);
            DriveTrain.cartesianDriveTimer(0, -0.4, 75);

            //Shoot our shot
            Intake.releaseAll();
            Intake.defaultPos();
            //Turn on intake
            Intake.intake();

            //Drive forward to white line
            DriveTrain.cartesianDriveTimer(0, -0.4, 10);

            DriveTrain.driveToLine(.3,"WHITE", telemetry);
            //sleep(100);


            //Shoot
            Shooter.setPosition("WHITE_LINE");
            DriveTrain.cartesianDriveTimer(-0.7, 0, 10);
            sleep(100);

            DriveTrain.cartesianDriveTimer(0, 0.4, 25);
            //sleep(100);

            Intake.releaseAll();
            Intake.defaultPos();
            Shooter.shoot(0);
            Shooter.setPosition("INDEX");
            //sleep(100);

            //Turn off intake
            Intake.stop();

            //Drive Forward to blue Line
//            DriveTrain.driveToLine(.25,"WHITE");
//            sleep(100);

            DriveTrain.cartesianDriveTimer(0, -0.4, 30);
            //sleep(100);

            DriveTrain.cartesianDriveTimer(0.8, 0, 10);
            sleep(100);

            DriveTrain.driveToLine(0.3,"BLUE", telemetry);
            //sleep(100);

            DriveTrain.cartesianDriveTimer(0, -0.3, 10);
            //sleep(100);

            //Drop wobb
            Wobble.drop();

            //Diagonal move over to wall
            DriveTrain.cartesianDriveDistance(0.6, 0.32, "LEFT", telemetry,"GREATER");
            sleep(100); //200

            //Backup along wall with timer
            DriveTrain.cartesianDriveTimer(0, 0.6, 15);
            //sleep(100);
        }

        else if(label.equals("Quad")) {
//            //------------4------------
//            //Drive Forward to ring
//            Shooter.shoot(Shooter.SHOOTER_POWER);
//            Shooter.setPosition("RINGS");
//            //sleep(500);
//            DriveTrain.cartesianDriveTimer(0, -0.4, 40);
//
//            //Shoot our shot
//            Intake.releaseAllRings();
//            Intake.defaultPos();
//
//            //Turn on intake
//            Intake.intake();
//
//            //Drive forward to white line
//            DriveTrain.driveToLine(.3,"WHITE");
//            sleep(100);
//
//            //Shoot
//            Shooter.setPosition("WHITE_LINE");
//            DriveTrain.cartesianDriveTimer(-0.7, 0, 10);
//            sleep(100);
//
//            DriveTrain.cartesianDriveTimer(0, 0.38, 35);
//            //sleep(100);
//
//            Intake.releaseAll();
//            Intake.defaultPos();
//
//            Shooter.shoot(0);
//            Shooter.setPosition("INDEX");

            //------------4------------
            //Drive Forward to ring
//            Shooter.shoot(Shooter.SHOOTER_POWER);
//            Shooter.setPosition("RINGS");
//            //sleep(500);
//            DriveTrain.cartesianDriveTimer(0, -0.4, 40);
//
//            //Shoot our shot
//            Intake.releaseAllRings();
//            Intake.defaultPos();
            //Drive Forward to ring
            Shooter.shoot(Shooter.SHOOTER_POWER);
            Shooter.setPosition("RINGS");
            //sleep(500);
            DriveTrain.cartesianDriveTimer(0, -0.4, 36);
            sleep(100);

//            DriveTrain.cartesianDriveTimer(-0.73, -0.1, 14);
//            DriveTrain.cartesianDriveTimer(0, -0.4, 10);

            //Shoot our shot
            Intake.releaseAllRings();
            Intake.defaultPos();


            //Turn on intake
            Intake.intake();

            DriveTrain.cartesianDriveTimer(0, -0.4, 30);
            Intake.releaseAll();
            Intake.defaultPos();

            //Drive forward to white line
            DriveTrain.driveToLine(.3,"WHITE", telemetry);
            sleep(100);

            //Shoot
            Shooter.setPosition("WHITE_LINE");
            DriveTrain.cartesianDriveTimer(-0.7, 0, 5);
            sleep(100);

            DriveTrain.cartesianDriveTimer(0, 0.38, 25);
            //sleep(100);

            Intake.releaseAll();
            Intake.defaultPos();

            Shooter.shoot(0);
            Shooter.setPosition("INDEX");






            //Drive to white line again
            DriveTrain.driveToLine(.3,"WHITE", telemetry);
            //sleep(100);

            //Strafe over to wall
            DriveTrain.cartesianDriveDistance(0.6, -0.32, "LEFT", telemetry,"GREATER");
            //sleep(200);

            DriveTrain.cartesianDriveTimer(0, -0.3, 20);
//
//            //drive to front wall
//            DriveTrain.cartesianDriveTimer(0, -0.5, 45);
//            sleep(100);

            //Drop wobb
            Wobble.drop();

            //Backup along wall with timer
            DriveTrain.cartesianDriveTimer(0, 0.6, 35);
            //sleep(100);
        }

        if(label == null || label.equals("ZERO")){
//            DriveTrain.cartesianDriveDistance(0, .36, "BACK", telemetry);
//
//            DriveTrain.cartesianDriveTimer(0, .4, 8);
//            //sleep(100);
//
//            //Open Claw
//            Wobble.open();
//
//            //Lower Arm
//            Wobble.lowerArm(Wobble.WOBBLE_DOWN_TICKS);
//            sleep(1500);
//
//            //Close Claw
//            Wobble.close();
//            sleep(300);
//
//            //Raise Claw
//            Wobble.raiseArm(Wobble.WOBBLE_UP_TICKS);
//            sleep(1500);
//            Wobble.wobbleMotor.setPower(-0.1);
//
//            DriveTrain.cartesianDriveTimer(-0.8, 0, 30);
//            sleep(100);
            Wobble.open();

            //Lower Arm
            Wobble.lowerArm(Wobble.WOBBLE_DOWN_TICKS);
            sleep(800);

            DriveTrain.cartesianDriveDistance(0, .36, "BACK", telemetry, "GREATER");

            //Close Claw
            Wobble.close();
            sleep(300);

            //Raise Claw
            Wobble.raiseArm(Wobble.WOBBLE_UP_TICKS);
            sleep(1000);
            Wobble.wobbleMotor.setPower(-0.1);

            DriveTrain.cartesianDriveTimer(-0.8, 0, 30);
            sleep(100);
        }
        else {
            //ALL autonomouses... pick up second wobble
            //Drive forward to wobble

            //Open Claw
            Wobble.open();

            //Lower Arm
            Wobble.lowerArm(Wobble.WOBBLE_DOWN_TICKS);
            sleep(800);

            DriveTrain.cartesianDriveDistance(0, .36, "BACK", telemetry,"GREATER");

            //Close Claw
            Wobble.close();
            sleep(300);

            //Raise Claw
            Wobble.raiseArm(Wobble.WOBBLE_UP_TICKS);
            sleep(1000);
            Wobble.wobbleMotor.setPower(-0.1);

//            DriveTrain.cartesianDriveTimer(-0.8, 0, 15);
//            sleep(100);
        }

        if(label.equals("ZERO")){
            //Strafe towards wall
//            DriveTrain.cartesianDriveDistance(0.3, 0, "LEFT", telemetry);
//            sleep(100);

            //Drive to white line
            DriveTrain.driveToLine(.3,"WHITE", telemetry);
            sleep(500);

            //Turn 180 degrees
//            int turnTimer = 300;
//            while (Math.abs(DriveTrain.angles.firstAngle - (-3.14)) > (Math.PI / 60) && turnTimer > 0){
//                DriveTrain.autoAlignAuto(-3.14); //-3.092333
//                turnTimer--;
//            }
//            DriveTrain.rightFrontMotor.setPower(0);
//            DriveTrain.rightBackMotor.setPower(0);
//            DriveTrain.leftFrontMotor.setPower(0);
//            DriveTrain.leftBackMotor.setPower(0);
//            sleep(100);
            //Turn 180 degrees
            int turnTimer = 250;
            while (Math.abs(DriveTrain.angles.firstAngle - (-Math.PI / 1.56)) > (Math.PI / 90) && turnTimer > 0){
                DriveTrain.autoAlignAuto(-Math.PI / 1.56); //-3.092333
                turnTimer--;
            }
            DriveTrain.rightFrontMotor.setPower(0);
            DriveTrain.rightBackMotor.setPower(0);
            DriveTrain.leftFrontMotor.setPower(0);
            DriveTrain.leftBackMotor.setPower(0);
            sleep(100);

//            DriveTrain.cartesianDriveTimer(0, -.4, 15);

            Wobble.lowerArm(Wobble.WOBBLE_DOWN_TICKS);
            sleep(1000);

            //Open claw
            Wobble.open();
            sleep(250);


            //Back UP
            DriveTrain.cartesianDriveTimer(-0.5, -.45, 30);
            //sleep(100);

            //Raise Claw
            Wobble.raiseArm(-50);
            sleep(1500);
            //Lower arm
//            Wobble.lowerArm(Wobble.WOBBLE_DOWN_TICKS);
//            sleep(500);
//
//            //Open claw
//            Wobble.open();
//            sleep(250);
//
//            //Raise Claw
//            Wobble.raiseArm(-50);
//            sleep(1500);
//
//            DriveTrain.cartesianDriveTimer(-0.8, 0, 30);
//            sleep(100);
//
//            DriveTrain.cartesianDriveTimer(0, -.4, 40);
//            sleep(100);
        }

        else if(label.equals("Single")){
            //Strafe towards wall
//            DriveTrain.cartesianDriveDistance(0.3, 0, "LEFT", telemetry);
//            sleep(100);

            //Drive to white line

            DriveTrain.cartesianDriveTimer(-.7, -.3, 40);
            //sleep(100);

            DriveTrain.driveToLine(.3,"WHITE", telemetry);
            //sleep(500);

//            //Strafe towards drop off zone
//            DriveTrain.cartesianDriveTimer(-.8, 0, 30); // -.3, 100
//            //sleep(500);

            //Turn 180 degrees
            int turnTimer = 250;
            while (Math.abs(DriveTrain.angles.firstAngle - (3.0)) > (Math.PI / 90) && turnTimer > 0){
                DriveTrain.autoAlignAuto(3.0); //-3.092333
                turnTimer--;
            }
            DriveTrain.rightFrontMotor.setPower(0);
            DriveTrain.rightBackMotor.setPower(0);
            DriveTrain.leftFrontMotor.setPower(0);
            DriveTrain.leftBackMotor.setPower(0);
            sleep(100);

            //Drive to drop off zone
            DriveTrain.cartesianDriveTimer(0, -0.60, 4); // -.3, 100
            //sleep(500);

            //Open claw
            Wobble.lowerArm(Wobble.WOBBLE_DOWN_TICKS);
            sleep(1000);

            //Open claw
            Wobble.openSmall();
            sleep(250);

            //DriveTrain.driveToLine(-.3,"WHITE");
            //Raise Claw
            DriveTrain.cartesianDriveTimer(0, .4, 20);

            Wobble.raiseArm(-50);
            sleep(800);
            //Back UP

            DriveTrain.cartesianDriveTimer(.8, -.2, 50);
            //sleep(100);



        }

        else if(label.equals("Quad")){
            //Strafe towards wall
//            DriveTrain.cartesianDriveDistance(0.3, 0, "LEFT", telemetry);
//            sleep(100);

            DriveTrain.cartesianDriveTimer(0.6, -0.4, 15);

            //Drive to white line
            DriveTrain.driveToLine(.3,"WHITE", telemetry);
            //sleep(500);

            //Turn 180 degrees
            int turnTimer = 300;
            while (Math.abs(DriveTrain.angles.firstAngle - (-3.1415)) > (Math.PI / 180) && turnTimer > 0){
                DriveTrain.autoAlignAuto(-3.1415); //-3.092333
                turnTimer--;
            }
            DriveTrain.rightFrontMotor.setPower(0);
            DriveTrain.rightBackMotor.setPower(0);
            DriveTrain.leftFrontMotor.setPower(0);
            DriveTrain.leftBackMotor.setPower(0);
            sleep(100);

            //Drive to drop off zone
            DriveTrain.cartesianDriveTimer(0.6, -0.4, 30); // -.3, 100

            DriveTrain.cartesianDriveTimer(-0.6, -0.35, 15);
            //sleep(500);

            DriveTrain.cartesianDriveTimer(0, -0.2, 15);

            //Open claw
            Wobble.open();
            sleep(500);

            //Back UP
            DriveTrain.cartesianDriveTimer(0.5, 0.6, 35);
            //sleep(100);

            //Raise Claw
            Wobble.raiseArm(-50);
            sleep(1500);
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
