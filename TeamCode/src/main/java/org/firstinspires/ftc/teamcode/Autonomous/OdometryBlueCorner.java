package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.MasterVision;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Wobble;
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

import java.util.List;
@Autonomous(name= "OdometeryBlueCorner", group= "Autonomous")
public class OdometryBlueCorner extends LinearOpMode {
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

    DcMotor left_back;
    DcMotor left_front;
    DcMotor right_front;
    DcMotor right_back;
    DcMotor verticalLeft;
    DcMotor verticalRight;
    DcMotor horizontal;

    double robotGlobalXPosition = 0, robotGlobalYPosition = 0, robotOrientationRadians = 0;
    double previousTargetX = 0;
    double getPreviousTargetY = 0;

    public static Orientation angles;

    final double COUNTS_PER_INCH = 465.701027285;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    OdometryGlobalCoordinatePosition globalPositionUpdate;

    public void runOpMode() throws InterruptedException {

        DriveTrain.initDriveTrain(hardwareMap);
        Shooter.initShooter(hardwareMap);
        Intake.initIntake(hardwareMap);
        Wobble.initWobble(hardwareMap);

        Shooter.updateShooterConstants(50, 1, 2,0);

        left_front = DriveTrain.leftFrontMotor;
        left_back = DriveTrain.leftBackMotor;
        right_front = DriveTrain.rightFrontMotor;
        right_back = DriveTrain.rightBackMotor;

        verticalLeft = DriveTrain.verticalLeft;
        verticalRight = DriveTrain.verticalRight;
        horizontal = DriveTrain.horizontal;

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

//        initVuforia();
//        initTfod(hardwareMap);
//        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_NONE);
//        vision.enable();
//
//        if (tfod != null) {
//            tfod.activate();
//            tfod.setZoom(1, 16.0 / 9.0);
//        }
//
//        sleep(500);
//
//        tfodRecogntions = tfod.getUpdatedRecognitions();
//
//        for (Recognition recognition : tfodRecogntions) {
//            label = recognition.getLabel();
//        }
//
//        if(tfodRecogntions.isEmpty()){
//            label = "ZERO";
//        }
//
//        telemetry.addData("Rings: ", label);
//        telemetry.update();
//
//        while(!isStarted()) {
//            sleep(500);
//
//            tfodRecogntions = tfod.getUpdatedRecognitions();
//
//
//            for (Recognition recognition : tfodRecogntions) {
//                label = recognition.getLabel();
//            }
//
//            if(tfodRecogntions.isEmpty()){
//                label = "ZERO";
//            }
//
//            telemetry.addData("Rings: ", label);
//            telemetry.update();
//        }

        waitForStart();


        if(label == null || label.equals("ZERO")) {
            ///Drive forward to drop wobble
            goToPosition(0,67 * COUNTS_PER_INCH, 0.6, 0, 1000);
            setZeroPower(1000);

            //Drop wobble
            Wobble.drop();

            //Turn on shooter to prepare to shoot
            Shooter.shoot(Shooter.SHOOTER_POWER);
            sleep(200);

            //Drive over to shoot
            goToPosition(25 * COUNTS_PER_INCH,64 * COUNTS_PER_INCH, 0.6, 0, 1000);
            setZeroPower(1000);

            //Shoot
            Intake.releaseAll();
            sleep(200);

            //Lower arm to pick up second wobble
            Wobble.open();

            Wobble.lowerArm(Wobble.WOBBLE_DOWN_TICKS);
            sleep(800);

            //Go to second wobble
            goToPosition(25 * COUNTS_PER_INCH,40 * COUNTS_PER_INCH, 0.6, 0, 1000);
            setZeroPower(1000);

            //Pick up wobble
            Wobble.close();
            sleep(300);

            Wobble.raiseArm(Wobble.WOBBLE_UP_TICKS);
            sleep(1000);
            Wobble.wobbleMotor.setPower(-0.1);

            //Go to drop off wobble
            goToPosition(18 * COUNTS_PER_INCH,73 * COUNTS_PER_INCH, 0.65, 0, 1000);
            setZeroPower(1000);

            //Turn to drop wobble
            goToPosition(18 * COUNTS_PER_INCH,73 * COUNTS_PER_INCH, 0.7, 90, 1000);
            setZeroPower(1000);

            //Drop wobble
            Wobble.lowerArm(Wobble.WOBBLE_DOWN_TICKS);
            sleep(1000);

            Wobble.open();
            sleep(250);

        }

        else if(label.equals("Single")) {

        }

        else if(label.equals("Quad")) {

        }

//        DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");
        while(opModeIsActive()) {
//            DriveTrain.SUMO_MODE();
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

    public static double heldLeftEncoderCount;
    public static double heldRightEncoderCount;

    public void goToPosition(double targetXPos, double targetYPos, double power, double desiredOrientation, double allowedError){
        double distanceToXTarget = targetXPos - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPos - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        double previousDistance = 0;
        double previousAngle = 0;

        telemetry.addData("Distance To X", distanceToXTarget);
        telemetry.addData("Distance To Y", distanceToYTarget);
        telemetry.addData("Hypotenuse Distance", distance);
        telemetry.update();

        double initialDistance = Math.hypot(distanceToXTarget, distanceToYTarget);

        double heldLeftEncoderCount = verticalLeft.getCurrentPosition();
        double heldRightEncoderCount= verticalRight.getCurrentPosition();


        boolean turnFlag = false;

        while(opModeIsActive() && (distance > allowedError || (Math.toRadians(desiredOrientation - globalPositionUpdate.returnOrientation()) > (Math.PI)/180))){
            distanceToXTarget = targetXPos - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPos - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);



            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double PercentOfTarget = (distance/initialDistance);

            double robotMovementXComponent = calculateX(robotMovementAngle, power);
            double robotMovementYComponent = calculateY(robotMovementAngle, power);
            double pivot = Math.toRadians(desiredOrientation - globalPositionUpdate.returnOrientation());

            double pivotPower;
            double pivotFf = 0.18;

            if(Math.abs(pivot) > (Math.PI / 12)){
                pivotPower = 0.40;
            }
            else{
                if(Math.abs(pivot) > (Math.PI / 180)){
                    pivotPower = Math.abs(pivot / 0.6) + pivotFf;
                }
                else{
                    pivotPower = 0;
                }
            }

            if(pivot < 0){
                pivotPower = pivotPower * -1;
            }


            double feedForward = 0.1;
            double xMultiplier = 1.6;
            double yMultiplier = 1.08;
            if(Math.abs(distance - previousDistance) < 1){
                xMultiplier += 0.01;
                yMultiplier += 0.005;
            }
            if(Math.abs(pivot) > Math.PI / 180 && Math.abs(previousAngle - pivot) < Math.PI/180 ){
                pivotFf += 0.005;
            }


            if((targetXPos == previousTargetX && targetYPos == getPreviousTargetY) ||  pivot > (Math.PI)/6){
                left_front.setPower(pivotPower);
                right_front.setPower(-pivotPower);
                left_back.setPower(pivotPower);
                right_back.setPower(-pivotPower);
                distance = 0;
                turnFlag = true;
            }
            else{
                left_front.setPower ((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) + (xMultiplier * robotMovementXComponent)) + pivotPower); //+ feedForward
                right_front.setPower((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) - (xMultiplier * robotMovementXComponent)) - pivotPower);// + feedForward
                left_back.setPower  ((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) - (xMultiplier * robotMovementXComponent)) + pivotPower); //+ feedForward
                right_back.setPower ((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) + (xMultiplier * robotMovementXComponent)) - pivotPower); //+ feedForward
                turnFlag = false;
            }
//            if(turnFlag){
//                OdometryGlobalCoordinatePosition.setEncoderPosition(heldLeftEncoderCount, heldRightEncoderCount);
//                distance = 0;
//            }
//            else if(distanceToXTarget < 0 || distanceToYTarget < 0 || pivot > (Math.PI)/180){
//                left_front.setPower((robotMovementYComponent * (distance/targetDistance) + (xMultiplier * robotMovementXComponent)) + pivotPower);
//                right_front.setPower((robotMovementYComponent * (distance/targetDistance) - (xMultiplier * robotMovementXComponent)) - pivotPower);
//                left_back.setPower((robotMovementYComponent * (distance/targetDistance) - (xMultiplier * robotMovementXComponent))  + pivotPower);
//                right_back.setPower((robotMovementYComponent * (distance/targetDistance) + (xMultiplier * robotMovementXComponent)) - pivotPower);
//            }

            //+ pivotPower
            //- pivotPower
            //+ pivotPower
            //- pivotPower

            //* (distance/initialDistance)
            // * (distance/initialDistance)
            //* (distance/initialDistance)
            //* (distance/initialDistance)

            telemetry.addData("Vertical Left Position", -verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());

            telemetry.addData("Current Angle:", globalPositionUpdate.returnOrientation());

            telemetry.addLine();

            telemetry.addData("Command Horiz Position", robotMovementXComponent);
            telemetry.addData("Command Vert Position", robotMovementYComponent);

            telemetry.addLine();

            telemetry.addData("Distance To X", distanceToXTarget);
            telemetry.addData("Distance To Y", distanceToYTarget);
            telemetry.addData("Hypotenuse Distance", distance);

            telemetry.addLine();

            telemetry.addData("Target:", initialDistance);
            telemetry.addData("Distance:", distance);
            telemetry.addData("yeet", distance/initialDistance);

            telemetry.addData("Left Front Power:", left_front.getPower());
            telemetry.addData("Left Back Power:", left_back.getPower());
            telemetry.addData("Right Front Power:", right_front.getPower());
            telemetry.addData("Right Back Power:", right_back.getPower());

            telemetry.addData("Left Front Commanded Power:", ((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) + (xMultiplier * robotMovementXComponent)) + pivotPower));
            telemetry.addData("Left Back Commanded Power:",  ((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) - (xMultiplier * robotMovementXComponent)) - pivotPower));
            telemetry.addData("Right Front Commanded Power:",((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) - (xMultiplier * robotMovementXComponent)) + pivotPower));
            telemetry.addData("Right Back Commanded Power:", ((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) + (xMultiplier * robotMovementXComponent)) - pivotPower));

            telemetry.update();

            previousAngle = Math.toRadians(desiredOrientation - globalPositionUpdate.returnOrientation());
            previousDistance = distance;
        }

        previousTargetX = targetXPos;
        getPreviousTargetY= targetYPos;
//        if(desiredOrientation - globalPositionUpdate.returnOrientation() < desiredOrientation){
//            while(desiredOrientation - globalPositionUpdate.returnOrientation() < desiredOrientation || desiredOrientation - globalPositionUpdate.returnOrientation() > desiredOrientation)
//            double pivot = Math.toRadians(desiredOrientation - globalPositionUpdate.returnOrientation());
//
//            double pivotPower;
//
//            if(Math.abs(pivot) > (Math.PI / 12)){
//                pivotPower = 0.40;
//            }
//            else{
//                if(Math.abs(pivot) > (Math.PI / 180)){
//                    pivotPower = Math.abs(pivot / 0.6);
//                }
//                else{
//                    pivotPower = 0;
//                }
//            }
//
//            if(pivot < 0){
//                pivotPower = pivotPower * -1;
//            }
//        }
    }

    public void setZeroPower(int sleep){
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
        sleep(sleep);
    }

    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    public double distanceFormula(double x, double y){
        double distance = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
        return distance;
    }
}

