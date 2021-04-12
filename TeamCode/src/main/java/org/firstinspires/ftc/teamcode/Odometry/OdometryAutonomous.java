package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Wobble;


/**
 * Created by Sarthak on 10/4/2019.
 */
@Autonomous(name = "OdometryAuto", group= "Autonomous")
public class OdometryAutonomous extends LinearOpMode {

    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    double robotGlobalXPosition = 0, robotGlobalYPosition = 0, robotOrientationRadians = 0;

    public static Orientation angles;

    final double COUNTS_PER_INCH = 465.701027285;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "rightFrontMotor", rbName = "rightBackMotor", lfName = "leftFrontMotor", lbName = "leftBackMotor";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //DriveTrain.initDriveTrain(hardwareMap);
        Shooter.initShooter(hardwareMap);
        Intake.initIntake(hardwareMap);
        Wobble.initWobble(hardwareMap);

        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        waitForStart();

        goToPosition(0 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, .75, 0, 1000);
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
        sleep(1000);
        goToPosition(24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, .9, 0, 1000);
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
        sleep(1000);
        goToPosition(24 * COUNTS_PER_INCH, 0 * COUNTS_PER_INCH, .75, 0, 1000);
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
        sleep(1000);
        goToPosition(0 * COUNTS_PER_INCH, 0 * COUNTS_PER_INCH, .8, 0, 1000);
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
        sleep(1000);
        //goToPosition(36 * COUNTS_PER_INCH, 36 * COUNTS_PER_INCH, .5, 0, 1000);
        //goToPosition(20 * COUNTS_PER_INCH, 20 * COUNTS_PER_INCH, .5, 0, 1);
//        goToPosition(20 * COUNTS_PER_INCH, 0 * COUNTS_PER_INCH, .5, 0, 1);
//        goToPosition(0 * COUNTS_PER_INCH, 0 * COUNTS_PER_INCH, .5, 0, 1);

//
//        while(opModeIsActive()){
//            //Display Global (x, y, theta) coordinates
//            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
//            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
//            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
//
//            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
//            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
//            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());
//
//            telemetry.addData("Thread Active", positionThread.isAlive());
//            telemetry.update();
//        }

        //Stop the thread
        //globalPositionUpdate.stop();

    }

    public void goToPosition(double targetXPos, double targetYPos, double power, double desiredOrientation, double allowedError){
        double distanceToXTarget = targetXPos - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPos - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        double targetDistance = Math.hypot(targetXPos, targetYPos);

        while(opModeIsActive() && distance > allowedError){
            distanceToXTarget = targetXPos - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPos - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double PercentOfTarget = (distance/targetDistance);

            double robotMovementXComponent = calculateX(robotMovementAngle, power);
            double robotMovementYComponent = calculateY(robotMovementAngle, power);
            double pivot = desiredOrientation - globalPositionUpdate.returnOrientation();

            double feedForward = 0.1;
            if(distanceToXTarget < 0 || distanceToYTarget < 0){
                left_front.setPower(((robotMovementYComponent + (2 * robotMovementXComponent)) * (distance/targetDistance) - feedForward));
                right_front.setPower(((robotMovementYComponent - (2 * robotMovementXComponent)) * (distance/targetDistance) - feedForward));
                left_back.setPower(((robotMovementYComponent - (2 * robotMovementXComponent)) * (distance/targetDistance) - feedForward));
                right_back.setPower(((robotMovementYComponent + (2 * robotMovementXComponent)) * (distance/targetDistance) - feedForward));
            }
            else{
                left_front.setPower(((robotMovementYComponent + (2 * robotMovementXComponent)) * (distance/targetDistance) + feedForward));
                right_front.setPower(((robotMovementYComponent - (2 * robotMovementXComponent)) * (distance/targetDistance) + feedForward));
                left_back.setPower(((robotMovementYComponent - (2 * robotMovementXComponent)) * (distance/targetDistance) + feedForward));
                right_back.setPower(((robotMovementYComponent + (2 * robotMovementXComponent)) * (distance/targetDistance) + feedForward));
            }


            telemetry.addData("Vertical Left Position", -verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());

            telemetry.addLine();

            telemetry.addData("Command Left Position", robotMovementXComponent);
            telemetry.addData("Command Right Position", robotMovementYComponent);

            telemetry.addLine();

            telemetry.addData("Distance To X", distanceToXTarget);
            telemetry.addData("Distance To Y", distanceToYTarget);
            telemetry.addData("Hypotenuse Distance", distance);
            telemetry.update();
        }
    }

//    private boolean goToPositionNerd(double targetX, double targetY, double targetOrientation, double maxPower, double minPower){
//        angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//
//        double xDistance = targetX - robotGlobalXPosition;
//        double yDistance = targetY - robotGlobalYPosition;
//
//        double distance = distanceFormula(xDistance, yDistance);
//
//        double robotOrientationDifference = targetOrientation - angles.firstAngle;
//
//        double robotMoveAngle;
//        robotMoveAngle = Math.toDegrees(Math.atan(xDistance/yDistance));
//        if((xDistance < 0 && yDistance < 0) || (xDistance > 0 && yDistance < 0)){
//            robotMoveAngle += 180;
//        }
//        robotMoveAngle = ((robotMoveAngle - angles.firstAngle) % 360);
//
//        if(!(Math.abs(yDistance) < 1.25 * COUNTS_PER_INCH && Math.abs(xDistance) < 1.25 * COUNTS_PER_INCH)){
//            double currentAngle = angles.firstAngle;
//
//            double[] currentMotorPowers = null;
//            double lfrbPower = 0, rflbPower = 0;
//
//            if(distance > 3*COUNTS_PER_INCH){
//                currentMotorPowers = getMotorPowers(robotMoveAngle);
//                double pivotCorrection = ((currentAngle - targetOrientation) * DEFAULT_PID[0]);
//                lfrbPower = (currentMotorPowers[0] - pivotCorrection);
//                rflbPower = (currentMotorPowers[1] + pivotCorrection);
//            }else{
//                currentMotorPowers = getMotorPowers(robotMoveAngle);
//                double pivotCorrection = ((currentAngle - targetOrientation) * DEFAULT_PID[0]);
//                lfrbPower = (currentMotorPowers[0] - pivotCorrection) * 0.35;
//                rflbPower = (currentMotorPowers[1] + pivotCorrection) * 0.35;
//            }
//
//
//            right_front.setPower(rflbPower);
//            left_back.setPower(rflbPower);
//            left_front.setPower(lfrbPower);
//            right_back.setPower(lfrbPower);
//
//            telemetry.addData("Encoder Distance", distance/COUNTS_PER_INCH);
//            telemetry.addData("X Distance", xDistance/COUNTS_PER_INCH);
//            telemetry.addData("Y Distance", yDistance/COUNTS_PER_INCH);
//            telemetry.addData("Move Angle", robotMoveAngle);
//
//            return true;
//        }else{
//            return false;
//        }
//
//    }
    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
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
