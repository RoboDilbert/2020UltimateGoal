package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.MasterVision;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Wobble;

import java.util.List;

@Autonomous(name= "RedCornerDoubleWobble", group= "Autonomous")
public class OdometryRedCornerDoubleWobble extends LinearOpMode {
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

    public static ElapsedTime timeyBoi = new ElapsedTime();

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

        DriveTrain.composeTelemetry(telemetry);
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
        super.resetStartTime();

        if(label == null || label.equals("ZERO")) {
            ///Drive forward to drop wobble
            goToPosition(5 * COUNTS_PER_INCH,0 * COUNTS_PER_INCH, 0.8, 0, 1000);//0.6

            goToPosition(5 * COUNTS_PER_INCH,50 * COUNTS_PER_INCH, 0.5, 0, 8000);//0.6

            goToPosition(5 * COUNTS_PER_INCH,62 * COUNTS_PER_INCH, 0.3, 0, 1200);//0.6

            //Drop wobble
            Wobble.drop();

            //Turn on shooter to prepare to shoot
            Shooter.shoot(Shooter.SHOOTER_POWER);
            sleep(200);
            Shooter.setPosition("WHITE_LINE");

            //Drive over to shoot
            goToPosition(-15 * COUNTS_PER_INCH,62 * COUNTS_PER_INCH, 0.4, 0, 1000);//0.6

            //Line up to shoot

            Straighten();

            //Shoot
            Intake.releaseAll();
            sleep(200);

            //Lower arm to pick up second wobble
            Wobble.open();

            Wobble.lowerArm(Wobble.WOBBLE_DOWN_TICKS);
            sleep(800);

            //Go to second wobble
            goToPosition(-37 * COUNTS_PER_INCH,47 * COUNTS_PER_INCH, 0.4, 0, 8000);//0.6

            goToPosition(-37 * COUNTS_PER_INCH,32 * COUNTS_PER_INCH, 0.4, 0, 1000);//0.6

//            goToPosition(-37 * COUNTS_PER_INCH,32 * COUNTS_PER_INCH, 0.3, 0, 1000);//0.6

            //Pick up wobble
            Wobble.close();
            sleep(300);

            Wobble.raiseArm(Wobble.WOBBLE_UP_TICKS);
            sleep(1000);
            Wobble.wobbleMotor.setPower(-0.1);

            //Go to drop off wobble
            goToPosition(-20 * COUNTS_PER_INCH,73 * COUNTS_PER_INCH, 0.4, 0, 1000); //0.65

            //Turn to drop wobble
            goToPosition(-20 * COUNTS_PER_INCH,73 * COUNTS_PER_INCH, 0.55, -88, 2000);//0.7

            //Drop wobble
            Wobble.lowerArm(Wobble.WOBBLE_DOWN_TICKS);
            sleep(1000);

            Wobble.open();
            sleep(250);

            //Lift Arm
            Wobble.raiseArm(Wobble.WOBBLE_UP_TICKS);
            sleep(1000);

            Wobble.wobbleMotor.setPower(-0.1);
            Wobble.close();
            sleep(300);

            goToPosition(-20 * COUNTS_PER_INCH,73 * COUNTS_PER_INCH, 0.55, 0, 2000);//0.7


            positionThread.interrupt();
            if(!positionThread.isAlive()){
                telemetry.addData("WE OUT THIS BaTCH", "WE OUT THIS BaTCH");
                telemetry.update();
            }
        }

        else if(label.equals("Single")) {
            //Drive forward to shoot
            goToPosition(0,50 * COUNTS_PER_INCH, 0.5, 0, 8000);//0.6

            goToPosition(0,62 * COUNTS_PER_INCH, 0.4, 0, 1300);//0.6

            //Turn on shooter to prepare to shoot
            Shooter.shoot(Shooter.SHOOTER_POWER);
            sleep(200);
            Shooter.setPosition("WHITE_LINE");

            //Go to shooting posiiton
            goToPosition(-17 * COUNTS_PER_INCH,62 * COUNTS_PER_INCH, 0.4, 0, 1300);//0.6

            Straighten();

            //Shoot
            Intake.releaseAll();
            sleep(200);

            //Drive forward to release wobble
            //goToPosition(38 * COUNTS_PER_INCH,59 * COUNTS_PER_INCH, 0.35, 0, 1000);//0.6

//            Straighten();

            goToPosition(-22 * COUNTS_PER_INCH,82 * COUNTS_PER_INCH, 0.4, 0, 1300);//0.6

            //Drop wobble
            Wobble.drop();

            Intake.intake();

            Wobble.lowerArm(Wobble.WOBBLE_DOWN_TICKS);
            sleep(1000);

            goToPosition(-23 * COUNTS_PER_INCH,62 * COUNTS_PER_INCH, 0.45, 0, 4000);//0.6

            goToPosition(-23 * COUNTS_PER_INCH,44 * COUNTS_PER_INCH, 0.32, 0, 1200);//0.6

            //Lower wobble arm
            Wobble.open();

            //Go to second wobble
            goToPosition(-37 * COUNTS_PER_INCH,47 * COUNTS_PER_INCH, 0.4, 0, 1500);//0.6

            goToPosition(-37 * COUNTS_PER_INCH,33 * COUNTS_PER_INCH, 0.4, 0, 1300);//0.6

//            goToPosition(-37 * COUNTS_PER_INCH,32 * COUNTS_PER_INCH, 0.3, 0, 1000);//0.6

            //Pick up wobble
            Wobble.close();
            sleep(300);

            Wobble.raiseArm(Wobble.WOBBLE_UP_TICKS);
            sleep(1000);
            Wobble.wobbleMotor.setPower(-0.1);

            //Line up to shoot
            goToPosition(-17 * COUNTS_PER_INCH,62 * COUNTS_PER_INCH, 0.4, 0, 1300);//0.6
            sleep(100);

            Straighten();

            Intake.stop();
            //Shoot
            Intake.releaseOne();
            sleep(200);

            //Go to drop off wobble
            goToPosition(-17 * COUNTS_PER_INCH,69 * COUNTS_PER_INCH, 0.4, 0, 1300);//0.6   //27, 75

            //Turn
            goToPosition(-17 * COUNTS_PER_INCH,69 * COUNTS_PER_INCH, 0.5, 183, 1300);//0.6    //27, 75

            positionThread.interrupt();
            if(!positionThread.isAlive()){
                telemetry.addData("WE OUT THIS BaTCH", "WE OUT THIS BaTCH");
                telemetry.update();
            }

            //Drop wobble
            Wobble.lowerArm(Wobble.WOBBLE_DOWN_TICKS);
            sleep(1000);

            Wobble.open();
            sleep(250);

            //Lift Arm
            Wobble.raiseArm(Wobble.WOBBLE_UP_TICKS);
            sleep(1000);

            Wobble.wobbleMotor.setPower(-0.1);
            Wobble.close();
            sleep(300);
        }

        else if(label.equals("Quad")) {

            //Drive to corner
            goToPosition(0 * COUNTS_PER_INCH,80 * COUNTS_PER_INCH, 0.4, 0, 8000);//0.6

            goToPosition(0 * COUNTS_PER_INCH,112 * COUNTS_PER_INCH, 0.35, 0, 1500);//0.6

            // Drop wobble
            Wobble.drop();
            Shooter.shoot(Shooter.SHOOTER_POWER);
            //drive to shooting position
            goToPosition(0 * COUNTS_PER_INCH,70 * COUNTS_PER_INCH, .3, 0, 3000);//0.6

            goToPosition(25 * COUNTS_PER_INCH,58 * COUNTS_PER_INCH, .3, 0, 1000);//0.6

            //Shoot
            Straighten();

            Intake.releaseAll();
            sleep(200);
            //Back up to collect rings
            Intake.intake();

            goToPosition(18 * COUNTS_PER_INCH,57 * COUNTS_PER_INCH, 0.25, 0, 1000);//0.6

            goToPosition(18 * COUNTS_PER_INCH,50 * COUNTS_PER_INCH, 0.2, 0, 1000);//0.6
            //Shoot
            Straighten();
            Intake.releaseAllRings();
            sleep(200);
            //Collect Rings
            goToPosition(18 * COUNTS_PER_INCH,28 * COUNTS_PER_INCH, 0.2, 0, 1000);//0.6

            //Shoot
            goToPosition(25 * COUNTS_PER_INCH,58 * COUNTS_PER_INCH, .3, 0, 1000);//0.6

            Straighten();

            Intake.releaseAll();
            sleep(200);
            Intake.stop();
            //Move to white line
            goToPosition(48 * COUNTS_PER_INCH,75 * COUNTS_PER_INCH, .35, 0, 1000);//0.6

        }
        stop();
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
    public void Straighten(){
        double turnPow = 0.32;
        while(opModeIsActive() && Math.abs(DriveTrain.angles.firstAngle) > Math.PI/90 /*&& timeyBoi.seconds() < 15*/){
            if(DriveTrain.angles.firstAngle > Math.PI/90/*verticalLeft.getCurrentPosition() < verticalRight.getCurrentPosition()*/){
                left_front.setPower(turnPow);
                right_front.setPower(-turnPow);
                left_back.setPower(turnPow);
                right_back.setPower(-turnPow);
            }
            else if(DriveTrain.angles.firstAngle < -Math.PI/90/*verticalLeft.getCurrentPosition() > verticalRight.getCurrentPosition()*/){
                left_front.setPower(-turnPow);
                right_front.setPower(turnPow);
                left_back.setPower(-turnPow);
                right_back.setPower(turnPow);
            }
            telemetry.addData("IMU Heading:", DriveTrain.angles.firstAngle);
            telemetry.addData("Right Encoder:", verticalRight.getCurrentPosition());
            telemetry.addData("Left Encoder:", verticalLeft.getCurrentPosition());
            telemetry.update();
        }

        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);
    }

    public static double heldLeftEncoderCount;
    public static double heldRightEncoderCount;

    public void goToPosition(double targetXPos, double targetYPos, double power, double desiredOrientation, double allowedError){
        double distanceToXTarget = targetXPos - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPos - globalPositionUpdate.returnYCoordinate();

//        double realDesiredOrientation = desiredOrientation;
//        if(desiredOrientation == 375){
//            realDesiredOrientation = globalPositionUpdate.returnOrientation();
//        }

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

        double pivotPower = 0;
        double pivotMultiplier = 1.25;
        double pivotFf = 0.06;

        double pivot = Double.MAX_VALUE;

        double feedForward = 0.1;
        double xMultiplier = 1.1;
        double yMultiplier = 1.08;

        boolean turnFlag = false;

        while(opModeIsActive() && (distance > allowedError /*&& timeyBoi.seconds() < 15*/ || pivot > (Math.PI)/240 || pivot < -(Math.PI)/240) /*&& timeyBoi.seconds() < 15*/){
            distanceToXTarget = targetXPos - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPos - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double PercentOfTarget = (distance/initialDistance);

            double robotMovementXComponent = calculateX(robotMovementAngle, power);
            double robotMovementYComponent = calculateY(robotMovementAngle, power);
            pivot = Math.toRadians(desiredOrientation - globalPositionUpdate.returnOrientation());

            if(Math.abs(distance - previousDistance) < 15){
                xMultiplier += 0.02;
                yMultiplier += 0.04;
            }
            if(Math.abs(pivot) > Math.PI / 360 && Math.abs(previousAngle - pivot) < Math.PI/90 && targetXPos == previousTargetX && targetYPos == getPreviousTargetY){
                pivotMultiplier += 0.01;
            }
            else if(Math.abs(previousAngle - pivot) < Math.PI/360 || Math.abs(previousAngle - pivot) > Math.PI/90){
                pivotMultiplier = 1.25;
            }

            if(Math.abs(pivot) > (Math.PI / 12)){
                pivotPower = 0.40;
            }
            else{
                if(Math.abs(pivot) > (Math.PI / 360)){
                    pivotPower = (Math.abs(pivot / 0.8) * pivotMultiplier) + pivotFf;
                }
                else{
                    pivotPower = 0;
                }
            }
            if(pivot < 0){
                pivotPower = pivotPower * -1;
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
                left_front.setPower ((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) + (xMultiplier * (distance/initialDistance + .4)* robotMovementXComponent)) + pivotPower); //+ feedForward
                right_front.setPower((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) - (xMultiplier * (distance/initialDistance + .4)* robotMovementXComponent)) - pivotPower);// + feedForward
                left_back.setPower  ((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) - (xMultiplier * (distance/initialDistance + .4)* robotMovementXComponent)) + pivotPower); //+ feedForward
                right_back.setPower ((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) + (xMultiplier * (distance/initialDistance + .4)* robotMovementXComponent)) - pivotPower); //+ feedForward
                turnFlag = false;
            }
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
            telemetry.addData("Timer", super.getRuntime());
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

            telemetry.addData("Pivot:", pivot);
            telemetry.addData("PivotMultiplier:", pivotMultiplier);

            telemetry.addLine();

            telemetry.addData("Target:", initialDistance);
            telemetry.addData("Distance:", distance);
            telemetry.addData("yeet", distance/initialDistance);

            telemetry.addData("xMultiplier", xMultiplier);
            telemetry.addData("yMultiplier", yMultiplier);
            telemetry.addData("pivotMultiplier", pivotMultiplier);

            telemetry.addData("Left Front Power:", left_front.getPower());
            telemetry.addData("Left Back Power:", left_back.getPower());
            telemetry.addData("Right Front Power:", right_front.getPower());
            telemetry.addData("Right Back Power:", right_back.getPower());

            telemetry.addData("Left Front Commanded Power:", ((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) + (xMultiplier * robotMovementXComponent)) + pivotPower));
            telemetry.addData("Left Back Commanded Power:",  ((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) - (xMultiplier * robotMovementXComponent)) - pivotPower));
            telemetry.addData("Right Front Commanded Power:",((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) - (xMultiplier * robotMovementXComponent)) + pivotPower));
            telemetry.addData("Right Back Commanded Power:", ((robotMovementYComponent * ((distance/initialDistance) * yMultiplier) + (xMultiplier * robotMovementXComponent)) - pivotPower));
            telemetry.addData("Timer Timey Timo boyo", timeyBoi.seconds());
            telemetry.update();

            previousAngle = Math.toRadians(desiredOrientation - globalPositionUpdate.returnOrientation());
            previousDistance = distance;
        }

        previousTargetX = targetXPos;
        getPreviousTargetY= targetYPos;

        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
        sleep(250);


//        while(Math.abs(pivot) > Math.PI / 240){
//
//
//            pivot = Math.toRadians(desiredOrientation - globalPositionUpdate.returnOrientation());
//
//            if(Math.abs(pivot) > Math.PI / 360 && Math.abs(previousAngle - pivot) < Math.PI/90){
//                pivotMultiplier += 0.03;
//            }
//            if(Math.abs(previousAngle - pivot) < Math.PI/360 || Math.abs(previousAngle - pivot) > Math.PI/90){
//                pivotMultiplier = 1.5;
//            }
//
////            if(Math.abs(pivot) > Math.PI / 360 && Math.abs(previousAngle - pivot) < Math.PI / 90){
////                pivotMultiplier += 0.05;
////            }
////            if(Math.abs(previousAngle - pivot) > Math.PI/90){
////                pivotMultiplier = 1.5;
////            }
//
//            if(Math.abs(pivot) > (Math.PI / 12)){
//                pivotPower = 0.40;
//            }
//            else{
//                if(Math.abs(pivot) > (Math.PI / 360)){
//                    pivotPower = (Math.abs(pivot / 0.8) * pivotMultiplier) + pivotFf;
//                }
//                else{
//                    pivotPower = 0;
//                }
//            }
//
//            if(pivot < 0){
//                pivotPower = pivotPower * -1;
//            }
//
//            telemetry.addData("PivotMultiplier", pivotMultiplier);
//            telemetry.addData("PivotPower", pivotPower);
//            telemetry.update();
//
//            left_front.setPower(pivotPower);
//            right_front.setPower(-pivotPower);
//            left_back.setPower(pivotPower);
//            right_back.setPower(-pivotPower);
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

/*
In order to reach our goal on continuous improvement, we've used the mechanical addition of dead wheel encoders to make our autonomous programs
very consistent. We use our new encoders to increment our feed forward to make sure that we reach our target position with maximum precision.
With our new Encoders, we are able to turn the playing field into a coordinate plane and drive to any position on the field using this coordinate system.
We have also used the dead wheel encoders to determine our angle on the field. This allows us to auto align ourselves with
the shooting tower in TeleOp.










 */

