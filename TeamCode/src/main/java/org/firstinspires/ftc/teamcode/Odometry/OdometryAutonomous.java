package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Wobble;


/**
 * Created by Sarthak on 10/4/2019.
 */
@Autonomous(name = "OdometryAuto", group= "Autonomous")
public class OdometryAutonomous extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException {
        //DriveTrain.initDriveTrain(hardwareMap);
        Shooter.initShooter(hardwareMap);
        Intake.initIntake(hardwareMap);
        Wobble.initWobble(hardwareMap);
        DriveTrain.initDriveTrain(hardwareMap);

        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        left_front = DriveTrain.leftFrontMotor;
        left_back = DriveTrain.leftBackMotor;
        right_front = DriveTrain.rightFrontMotor;
        right_back = DriveTrain.rightBackMotor;

        verticalLeft = DriveTrain.verticalLeft;
        verticalRight = DriveTrain.verticalRight;
        horizontal = DriveTrain.horizontal;

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        waitForStart();

        goToPosition(0 * COUNTS_PER_INCH, 25 * COUNTS_PER_INCH, .6, 0, 1000);
//        left_front.setPower(0);
//        left_back.setPower(0);
//        right_front.setPower(0);
//        right_back.setPower(0);
//        sleep(1000);
        goToPosition(25 * COUNTS_PER_INCH, 25 * COUNTS_PER_INCH, .7, 0, 1000);
//        left_front.setPower(0);
//        left_back.setPower(0);
//        right_front.setPower(0);
//        right_back.setPower(0);
//        sleep(1000);
        goToPosition(25 * COUNTS_PER_INCH, 0 * COUNTS_PER_INCH, .6, 0, 1000);
//        left_front.setPower(0);
//        left_back.setPower(0);
//        right_front.setPower(0);
//        right_back.setPower(0);
//        sleep(1000);
        goToPosition(0 * COUNTS_PER_INCH, 0 * COUNTS_PER_INCH, .7, 0, 1000);
//        left_front.setPower(0);
//        left_back.setPower(0);
//        right_front.setPower(0);
//        right_back.setPower(0);
//        sleep(1000);
        goToPosition(25 * COUNTS_PER_INCH, 25 * COUNTS_PER_INCH, .7, 0, 1000);

        Shooter.shoot(.5);
        sleep(2000);
    }

    public void goToPosition(double targetXPos, double targetYPos, double power, double desiredOrientation, double allowedError){
        double distanceToXTarget = targetXPos - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPos - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        telemetry.addData("Distance To X", distanceToXTarget);
        telemetry.addData("Distance To Y", distanceToYTarget);
        telemetry.addData("Hypotenuse Distance", distance);
        telemetry.update();

//        sleep(2000);

        double initialDistance = Math.hypot(distanceToXTarget, distanceToYTarget);

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

            if(Math.abs(pivot) > (Math.PI / 12)){
                pivotPower = 0.40;
            }
            else{
                if(Math.abs(pivot) > (Math.PI / 180)){
                    pivotPower = Math.abs(pivot / 0.6) + 0.13;
                }
                else{
                    pivotPower = 0;
                }
            }

            if(pivot < 0){
                pivotPower = pivotPower * -1;
            }

            double feedForward = 0.1;
            double xMultiplier = 1.7;


            if((targetXPos == previousTargetX && distanceToYTarget == getPreviousTargetY) ||  pivot > (Math.PI)/6){
                left_front.setPower(pivotPower);
                right_front.setPower(-pivotPower);
                left_back.setPower(pivotPower);
                right_back.setPower(-pivotPower);
            }
//            else if(distanceToXTarget < 0 || distanceToYTarget < 0 || pivot > (Math.PI)/180){
//                left_front.setPower((robotMovementYComponent * (distance/targetDistance) + (xMultiplier * robotMovementXComponent)) + pivotPower);
//                right_front.setPower((robotMovementYComponent * (distance/targetDistance) - (xMultiplier * robotMovementXComponent)) - pivotPower);
//                left_back.setPower((robotMovementYComponent * (distance/targetDistance) - (xMultiplier * robotMovementXComponent))  + pivotPower);
//                right_back.setPower((robotMovementYComponent * (distance/targetDistance) + (xMultiplier * robotMovementXComponent)) - pivotPower);
//            }
            else{
                left_front.setPower((robotMovementYComponent * (distance/initialDistance)+ (xMultiplier * robotMovementXComponent)) + pivotPower); //+ feedForward
                right_front.setPower((robotMovementYComponent * (distance/initialDistance) - (xMultiplier * robotMovementXComponent)) - pivotPower);// + feedForward
                left_back.setPower((robotMovementYComponent  * (distance/initialDistance)- (xMultiplier * robotMovementXComponent)) + pivotPower); //+ feedForward
                right_back.setPower((robotMovementYComponent * (distance/initialDistance)+ (xMultiplier * robotMovementXComponent)) - pivotPower); //+ feedForward
            }
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

            telemetry.addData("Left Front Commanded Power:", ((robotMovementYComponent * (distance/initialDistance) + (xMultiplier * robotMovementXComponent)) + pivotPower));
            telemetry.addData("Left Back Commanded Power:", ((robotMovementYComponent * (distance/initialDistance) - (xMultiplier * robotMovementXComponent)) + pivotPower));
            telemetry.addData("Right Front Commanded Power:", ((robotMovementYComponent * (distance/initialDistance) - (xMultiplier * robotMovementXComponent)) + pivotPower));
            telemetry.addData("Right Back Commanded Power:", ((robotMovementYComponent * (distance/initialDistance) + (xMultiplier * robotMovementXComponent)) + pivotPower));

            telemetry.update();


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

//    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
//        right_front = hardwareMap.dcMotor.get(rfName);
//        right_back = hardwareMap.dcMotor.get(rbName);
//        left_front = hardwareMap.dcMotor.get(lfName);
//        left_back = hardwareMap.dcMotor.get(lbName);
//
//        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
//        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
//        horizontal = hardwareMap.dcMotor.get(hEncoderName);
//
//        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
//        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        telemetry.addData("Status", "Hardware Map Init Complete");
//        telemetry.update();
//    }

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
