package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.MasterVision;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Wobble;

import java.util.List;


//@Autonomous(name= "JustinIsBeautiful", group= "Autonomous")

public class EncoderTest extends LinearOpMode{
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
                Shooter.shoot(Shooter.SHOOTER_POWER);
                Shooter.setPosition("WHITE_LINE");

                DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");

                DriveTrain.leftFrontMotor.setTargetPosition(1150);
                DriveTrain.leftBackMotor.setTargetPosition(1150);
                DriveTrain.rightFrontMotor.setTargetPosition(1150);
                DriveTrain.rightBackMotor.setTargetPosition(1150);

                DriveTrain.setRunMode("RUN_TO_POSITION");

                DriveTrain.leftFrontMotor.setPower(.2);
                DriveTrain.leftBackMotor.setPower(.2);
                DriveTrain.rightFrontMotor.setPower(.2);
                DriveTrain.rightBackMotor.setPower(.2);

                while(DriveTrain.leftFrontMotor.isBusy()){

                }
                sleep(500);
                DriveTrain.leftFrontMotor.setPower(0);
                DriveTrain.leftBackMotor.setPower(0);
                DriveTrain.rightFrontMotor.setPower(0);
                DriveTrain.rightBackMotor.setPower(0);

                Wobble.drop();
                sleep(100);

                DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");

                DriveTrain.leftFrontMotor.setTargetPosition(580);
                DriveTrain.leftBackMotor.setTargetPosition(-580);
                DriveTrain.rightFrontMotor.setTargetPosition(-580);
                DriveTrain.rightBackMotor.setTargetPosition(580);

                DriveTrain.setRunMode("RUN_TO_POSITION");

                DriveTrain.leftFrontMotor.setPower(.8);
                DriveTrain.leftBackMotor.setPower(-.8);
                DriveTrain.rightFrontMotor.setPower(-.8);
                DriveTrain.rightBackMotor.setPower(.8);

                while(DriveTrain.leftFrontMotor.isBusy()){

                }
                sleep(600);
                DriveTrain.leftFrontMotor.setPower(0);
                DriveTrain.leftBackMotor.setPower(0);
                DriveTrain.rightFrontMotor.setPower(0);
                DriveTrain.rightBackMotor.setPower(0);

                Intake.releaseAll();

                DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");

                DriveTrain.leftFrontMotor.setTargetPosition(400);
                DriveTrain.leftBackMotor.setTargetPosition(-400);
                DriveTrain.rightFrontMotor.setTargetPosition(-400);
                DriveTrain.rightBackMotor.setTargetPosition(400);

                DriveTrain.setRunMode("RUN_TO_POSITION");

                DriveTrain.leftFrontMotor.setPower(.8);
                DriveTrain.leftBackMotor.setPower(-.8);
                DriveTrain.rightFrontMotor.setPower(-.8);
                DriveTrain.rightBackMotor.setPower(.8);

                while(DriveTrain.leftFrontMotor.isBusy()){

                }
                sleep(200);

                DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");

                DriveTrain.leftFrontMotor.setTargetPosition(100);
                DriveTrain.leftBackMotor.setTargetPosition(100);
                DriveTrain.rightFrontMotor.setTargetPosition(100);
                DriveTrain.rightBackMotor.setTargetPosition(100);

                DriveTrain.setRunMode("RUN_TO_POSITION");

                DriveTrain.leftFrontMotor.setPower(.2);
                DriveTrain.leftBackMotor.setPower(.2);
                DriveTrain.rightFrontMotor.setPower(.2);
                DriveTrain.rightBackMotor.setPower(.2);

                while(DriveTrain.leftFrontMotor.isBusy()){

                }
                sleep(200);
                DriveTrain.leftFrontMotor.setPower(0);
                DriveTrain.leftBackMotor.setPower(0);
                DriveTrain.rightFrontMotor.setPower(0);
                DriveTrain.rightBackMotor.setPower(0);
            }

            else if(label.equals("Single")) {
                Shooter.shoot(Shooter.SHOOTER_POWER);
                Shooter.setPosition("WHITE_LINE");

                DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");

                DriveTrain.leftFrontMotor.setTargetPosition(1150);
                DriveTrain.leftBackMotor.setTargetPosition(1150);
                DriveTrain.rightFrontMotor.setTargetPosition(1150);
                DriveTrain.rightBackMotor.setTargetPosition(1150);

                DriveTrain.setRunMode("RUN_TO_POSITION");

                DriveTrain.leftFrontMotor.setPower(.2);
                DriveTrain.leftBackMotor.setPower(.2);
                DriveTrain.rightFrontMotor.setPower(.2);
                DriveTrain.rightBackMotor.setPower(.2);

                while(DriveTrain.leftFrontMotor.isBusy()){

                }
                sleep(500);
                DriveTrain.leftFrontMotor.setPower(0);
                DriveTrain.leftBackMotor.setPower(0);
                DriveTrain.rightFrontMotor.setPower(0);
                DriveTrain.rightBackMotor.setPower(0);

                sleep(100);

                DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");

                DriveTrain.leftFrontMotor.setTargetPosition(580);
                DriveTrain.leftBackMotor.setTargetPosition(-580);
                DriveTrain.rightFrontMotor.setTargetPosition(-580);
                DriveTrain.rightBackMotor.setTargetPosition(580);

                DriveTrain.setRunMode("RUN_TO_POSITION");

                DriveTrain.leftFrontMotor.setPower(.8);
                DriveTrain.leftBackMotor.setPower(-.8);
                DriveTrain.rightFrontMotor.setPower(-.8);
                DriveTrain.rightBackMotor.setPower(.8);

                while(DriveTrain.leftFrontMotor.isBusy()){

                }
                sleep(600);
                DriveTrain.leftFrontMotor.setPower(0);
                DriveTrain.leftBackMotor.setPower(0);
                DriveTrain.rightFrontMotor.setPower(0);
                DriveTrain.rightBackMotor.setPower(0);

                Intake.releaseAll();

                DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");

                DriveTrain.leftFrontMotor.setTargetPosition(450);
                DriveTrain.leftBackMotor.setTargetPosition(450);
                DriveTrain.rightFrontMotor.setTargetPosition(450);
                DriveTrain.rightBackMotor.setTargetPosition(450);

                DriveTrain.setRunMode("RUN_TO_POSITION");

                DriveTrain.leftFrontMotor.setPower(.2);
                DriveTrain.leftBackMotor.setPower(.2);
                DriveTrain.rightFrontMotor.setPower(.2);
                DriveTrain.rightBackMotor.setPower(.2);

                while(DriveTrain.leftFrontMotor.isBusy()){

                }
                sleep(100);
                DriveTrain.leftFrontMotor.setPower(0);
                DriveTrain.leftBackMotor.setPower(0);
                DriveTrain.rightFrontMotor.setPower(0);
                DriveTrain.rightBackMotor.setPower(0);

                Wobble.drop();
                sleep(200);

                DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");

                DriveTrain.leftFrontMotor.setTargetPosition(-350);
                DriveTrain.leftBackMotor.setTargetPosition(-350);
                DriveTrain.rightFrontMotor.setTargetPosition(-350);
                DriveTrain.rightBackMotor.setTargetPosition(-350);

                DriveTrain.setRunMode("RUN_TO_POSITION");

                DriveTrain.leftFrontMotor.setPower(.2);
                DriveTrain.leftBackMotor.setPower(-.2);
                DriveTrain.rightFrontMotor.setPower(-.2);
                DriveTrain.rightBackMotor.setPower(.2);

                while(DriveTrain.leftFrontMotor.isBusy()){

                }
                sleep(100);

                DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");

                DriveTrain.leftFrontMotor.setTargetPosition(400);
                DriveTrain.leftBackMotor.setTargetPosition(-400);
                DriveTrain.rightFrontMotor.setTargetPosition(-400);
                DriveTrain.rightBackMotor.setTargetPosition(400);

                DriveTrain.setRunMode("RUN_TO_POSITION");

                DriveTrain.leftFrontMotor.setPower(.8);
                DriveTrain.leftBackMotor.setPower(-.8);
                DriveTrain.rightFrontMotor.setPower(-.8);
                DriveTrain.rightBackMotor.setPower(.8);

                while(DriveTrain.leftFrontMotor.isBusy()){

                }
                sleep(600);
                DriveTrain.leftFrontMotor.setPower(0);
                DriveTrain.leftBackMotor.setPower(0);
                DriveTrain.rightFrontMotor.setPower(0);
                DriveTrain.rightBackMotor.setPower(0);
            }

            else if(label.equals("Quad")) {

            }

        }

        public static void initVuforia() {

            parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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
