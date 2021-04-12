package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.UtilOG.Constants;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

public class DriveTrain {

    //Motors
    public static DcMotor leftFrontMotor; //Expansion hub, port 0
    public static DcMotor leftBackMotor; //Expansion hub, port 1
    public static DcMotor rightFrontMotor; //Expansion hub, port 3
    public static DcMotor rightBackMotor; //Expansion hub, port 2

    public static DcMotor verticalLeft;
    public static DcMotor verticalRight;
    public static DcMotor horizontal;

    public static ColorSensor floorColorSensor; //Expansion hub, I2C Bus 3

    public static BNO055IMU imu; //Control hub, I2C Bus 0
    public static Orientation angles;
    public static Acceleration gravity;
    private static float gyroVariation = 0;

    public static double driveTrainError = 0;
    public static double driveTrainPower = 0;

    //2m distance sensors
    public static DistanceSensor frontDistanceSensor; //Expansion hub, I2C Bus 2;
    public static DistanceSensor backDistanceSensor; //Control hub, I2C Bus 2
    public static DistanceSensor leftDistanceSensor; //Control hub, I2C Bus 3;
    public static DistanceSensor rightDistanceSensor;//Control hub, I2C Bus 1

    //Constructor
    public DriveTrain(){}

    public static void initDriveTrain(HardwareMap hwm) {

        Constants.HwMap = hwm;
        leftFrontMotor = Constants.HwMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = Constants.HwMap.dcMotor.get("leftBackMotor");
        rightFrontMotor = Constants.HwMap.dcMotor.get("rightFrontMotor");
        rightBackMotor = Constants.HwMap.dcMotor.get("rightBackMotor");

        floorColorSensor = Constants.HwMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "floorColorSensor");

        frontDistanceSensor = Constants.HwMap.get(DistanceSensor.class, "frontDistanceSensor");
        backDistanceSensor = Constants.HwMap.get(DistanceSensor.class, "backDistanceSensor");
        leftDistanceSensor = Constants.HwMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = Constants.HwMap.get(DistanceSensor.class, "rightDistanceSensor");

        imu = Constants.HwMap.get(BNO055IMU.class, "imu");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters1);

        angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }

    public static void cartesianDrive(double x, double y, double z){
        double speed = Math.sqrt(2) * Math.hypot(x, y);
        double command = Math.atan2(y, -x) + Math.PI/2;
        double rotation = z;

        if(z > 0) {
            if (z < 0.01) {
                rotation = 0;
            } else if (z > 0.9) {
                rotation = z * z;
            } else {
                rotation = (z * z) + 0.013;
            }
        }
        if(z < 0) {
            if (z > -0.01) {
                rotation = 0;
            } else if (z < -0.9) {
                rotation = -(z * z);
            }
            else{
                rotation = (-(z*z)) - 0.013;
            }
        }

        angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double adjustedXHeading = Math.cos(command + angles.firstAngle + Math.PI/4);
        double adjustedYHeading = Math.sin(command + angles.firstAngle + Math.PI/4);

        DriveTrain.leftFrontMotor.setPower((speed * adjustedYHeading + rotation) * Constants.TELEOP_LIMITER);
        DriveTrain.rightFrontMotor.setPower((speed * adjustedXHeading - rotation) * Constants.TELEOP_LIMITER);
        DriveTrain.leftBackMotor.setPower((speed * adjustedXHeading + rotation) * Constants.TELEOP_LIMITER);
        DriveTrain.rightBackMotor.setPower((speed * adjustedYHeading - rotation) * Constants.TELEOP_LIMITER);
    }

    public static void cartesianDriveDropWheelEncoders(int leftEncoderCount, int rightEncoderCount, int horizontalEncoderCount){
    }

    public static void cartesianDriveTimer(double x, double y, int timerLength) throws InterruptedException {
        double speed = Math.sqrt(2) * Math.hypot(x, y);
        double command = Math.atan2(y, -x) + Math.PI/2;
        double rotation = 0;
        double startingHeading = angles.firstAngle;
        double currentError = 0;
        double adjustedXHeading = 0;
        double adjustedYHeading = 0;


        while(timerLength > 0) {
            angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            adjustedXHeading = Math.cos(command + angles.firstAngle + Math.PI / 4);
            adjustedYHeading = Math.sin(command + angles.firstAngle + Math.PI / 4);

            currentError = angles.firstAngle - startingHeading;

            if(Math.abs(currentError) > (Math.PI / 12)){
                rotation = 0.40;
            }
            else{
                if(Math.abs(currentError) > (Math.PI / 180)){
                    rotation = Math.abs(currentError / 0.6);
                }
                else{
                    rotation = 0;
                }
            }

            if(currentError < 0){
                rotation = rotation * -1;
            }

            leftFrontMotor.setPower((speed * adjustedYHeading + rotation) * Constants.TELEOP_LIMITER);
            rightFrontMotor.setPower((speed * adjustedXHeading - rotation) * Constants.TELEOP_LIMITER);
            leftBackMotor.setPower((speed * adjustedXHeading + rotation) * Constants.TELEOP_LIMITER);
            rightBackMotor.setPower((speed * adjustedYHeading - rotation) * Constants.TELEOP_LIMITER);
            Thread.sleep(20);
            timerLength--;
        }

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
    public static void cartesianDriveRing(double x, double y) throws InterruptedException {
        double speed = Math.sqrt(2) * Math.hypot(x, y);
        double command = Math.atan2(y, -x) + Math.PI/2;
        double rotation = 0;
        double startingHeading = 0;
        double currentError = 0;
        double adjustedXHeading = 0;
        double adjustedYHeading = 0;

        double ringDistance = Double.MAX_VALUE;


        while(ringDistance > 10) {

            ringDistance = frontDistanceSensor.getDistance(DistanceUnit.CM);

            angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            adjustedXHeading = Math.cos(command + angles.firstAngle + Math.PI / 4);
            adjustedYHeading = Math.sin(command + angles.firstAngle + Math.PI / 4);

            currentError = angles.firstAngle - startingHeading;

            if(Math.abs(currentError) > (Math.PI / 12)){
                rotation = 0.40;
            }
            else{
                if(Math.abs(currentError) > (Math.PI / 60)){
                    rotation = Math.abs(currentError / 0.6);
                }
                else{
                    rotation = 0;
                }
            }

            if(currentError < 0){
                rotation = rotation * -1;
            }

            leftFrontMotor.setPower((speed * adjustedYHeading + rotation) * Constants.TELEOP_LIMITER);
            rightFrontMotor.setPower((speed * adjustedXHeading - rotation) * Constants.TELEOP_LIMITER);
            leftBackMotor.setPower((speed * adjustedXHeading + rotation) * Constants.TELEOP_LIMITER);
            rightBackMotor.setPower((speed * adjustedYHeading - rotation) * Constants.TELEOP_LIMITER);

            ringDistance = frontDistanceSensor.getDistance(DistanceUnit.CM);

        }

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    public static void cartesianDriveDistance(double x, double y, String side, Telemetry telemetry, String greatOrLess) throws InterruptedException {
        double speed = Math.sqrt(2) * Math.hypot(x, y);
        double command = Math.atan2(y, -x) + Math.PI/2;
        double rotation = 0;
        double startingHeading = angles.firstAngle;
        double currentError = 0;
        double adjustedXHeading = 0;
        double adjustedYHeading = 0;

        double currentDistance;
        double exitValue;
        if(greatOrLess.equals("GREATER")) {
            currentDistance = Double.MAX_VALUE;
            exitValue = Double.MIN_VALUE;
            while (currentDistance > exitValue) {

                if (side.equals("LEFT")) {
                    currentDistance = leftDistanceSensor.getDistance(DistanceUnit.CM);//leftDistanceSensor
                    telemetry.addData("Left Distance Sensor", leftDistanceSensor.getDistance(DistanceUnit.CM));
                    exitValue = 73;
                } else if (side.equals("RIGHT")) {
                    currentDistance = rightDistanceSensor.getDistance(DistanceUnit.CM);// rightDistanceSensor
                    telemetry.addData("Right Distance Sensor", rightDistanceSensor.getDistance(DistanceUnit.CM));
                    exitValue = 15;
                } else if (side.equals("FRONT")) {
                    currentDistance = frontDistanceSensor.getDistance(DistanceUnit.CM);
                    telemetry.addData("Front Distance Sensor", frontDistanceSensor.getDistance(DistanceUnit.CM));
                    exitValue = 35;
                } else if (side.equals("BACK")) {
                    currentDistance = backDistanceSensor.getDistance(DistanceUnit.CM);
                    telemetry.addData("Back Distance Sensor", backDistanceSensor.getDistance(DistanceUnit.CM));
                    exitValue = 30;//28
                } else if (side.equals("BACK_SECOND")) {
                    currentDistance = backDistanceSensor.getDistance(DistanceUnit.CM);
                    telemetry.addData("Back Distance Sensor", backDistanceSensor.getDistance(DistanceUnit.CM));
                    exitValue = 28;
                } else if (side.equals("RIGHT_ZERO")) {
                    currentDistance = rightDistanceSensor.getDistance(DistanceUnit.CM);// rightDistanceSensor
                    telemetry.addData("Right Distance Sensor", rightDistanceSensor.getDistance(DistanceUnit.CM));
                    exitValue = 58;
                }

                angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

                adjustedXHeading = Math.cos(command + angles.firstAngle + Math.PI / 4);
                adjustedYHeading = Math.sin(command + angles.firstAngle + Math.PI / 4);

                currentError = angles.firstAngle - startingHeading;

                if (Math.abs(currentError) > (Math.PI / 12)) {
                    rotation = 0.40;
                } else {
                    if (Math.abs(currentError) > (Math.PI / 180)) {
                        rotation = Math.abs(currentError / 0.6);
                    } else {
                        rotation = 0;
                    }
                }

                if (currentError < 0) {
                    rotation = rotation * -1;
                }

                leftFrontMotor.setPower((speed * adjustedYHeading + rotation) * Constants.TELEOP_LIMITER);
                rightFrontMotor.setPower((speed * adjustedXHeading - rotation) * Constants.TELEOP_LIMITER);
                leftBackMotor.setPower((speed * adjustedXHeading + rotation) * Constants.TELEOP_LIMITER);
                rightBackMotor.setPower((speed * adjustedYHeading - rotation) * Constants.TELEOP_LIMITER);

                if (side.equals("LEFT")) {
                    currentDistance = leftDistanceSensor.getDistance(DistanceUnit.CM);//leftDistanceSensor
                } else if (side.equals("RIGHT")) {
                    currentDistance = rightDistanceSensor.getDistance(DistanceUnit.CM);//rightDistanceSensor
                } else if (side.equals("FRONT")) {
                    currentDistance = frontDistanceSensor.getDistance(DistanceUnit.CM);
                } else if (side.equals("BACK") || side.equals("FOUR_SECOND")) {
                    currentDistance = backDistanceSensor.getDistance(DistanceUnit.CM);
                } else if (side.equals("RIGHT_ZERO")) {
                    currentDistance = rightDistanceSensor.getDistance(DistanceUnit.CM);
                }
                telemetry.addData("Back Distance: ", backDistanceSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        }
        else if(greatOrLess.equals("LESS")){
            currentDistance = Double.MIN_VALUE;
            exitValue = Double.MAX_VALUE;
            while (currentDistance < exitValue) {

                if (side.equals("LEFT")) {
                    currentDistance = leftDistanceSensor.getDistance(DistanceUnit.CM);//leftDistanceSensor
                    telemetry.addData("Left Distance Sensor", leftDistanceSensor.getDistance(DistanceUnit.CM));
                    exitValue = 30;
                } else if (side.equals("RIGHT")) {
                    currentDistance = rightDistanceSensor.getDistance(DistanceUnit.CM);// rightDistanceSensor
                    telemetry.addData("Right Distance Sensor", rightDistanceSensor.getDistance(DistanceUnit.CM));
                    exitValue = 67;
                } else if (side.equals("FRONT")) {
                    currentDistance = frontDistanceSensor.getDistance(DistanceUnit.CM);
                    telemetry.addData("Front Distance Sensor", frontDistanceSensor.getDistance(DistanceUnit.CM));
                    exitValue = 30;
                } else if (side.equals("BACK")) {
                    currentDistance = backDistanceSensor.getDistance(DistanceUnit.CM);
                    telemetry.addData("Back Distance Sensor", backDistanceSensor.getDistance(DistanceUnit.CM));
                    exitValue = 27;
                } else if (side.equals("FOUR_SECOND")) {
                    currentDistance = backDistanceSensor.getDistance(DistanceUnit.CM);
                    telemetry.addData("Back Distance Sensor", backDistanceSensor.getDistance(DistanceUnit.CM));
                    exitValue = 25;
                }else if (side.equals("RIGHT_ZERO")) {
                    currentDistance = rightDistanceSensor.getDistance(DistanceUnit.CM);// rightDistanceSensor
                    telemetry.addData("Right Distance Sensor", rightDistanceSensor.getDistance(DistanceUnit.CM));
                    exitValue = 46;
                }

                angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

                adjustedXHeading = Math.cos(command + angles.firstAngle + Math.PI / 4);
                adjustedYHeading = Math.sin(command + angles.firstAngle + Math.PI / 4);

                currentError = angles.firstAngle - startingHeading;

                if (Math.abs(currentError) > (Math.PI / 12)) {
                    rotation = 0.40;
                } else {
                    if (Math.abs(currentError) > (Math.PI / 180)) {
                        rotation = Math.abs(currentError / 0.6);
                    } else {
                        rotation = 0;
                    }
                }

                if (currentError < 0) {
                    rotation = rotation * -1;
                }

                leftFrontMotor.setPower((speed * adjustedYHeading + rotation) * Constants.TELEOP_LIMITER);
                rightFrontMotor.setPower((speed * adjustedXHeading - rotation) * Constants.TELEOP_LIMITER);
                leftBackMotor.setPower((speed * adjustedXHeading + rotation) * Constants.TELEOP_LIMITER);
                rightBackMotor.setPower((speed * adjustedYHeading - rotation) * Constants.TELEOP_LIMITER);

                if (side.equals("LEFT")) {
                    currentDistance = leftDistanceSensor.getDistance(DistanceUnit.CM);//leftDistanceSensor
                } else if (side.equals("RIGHT")) {
                    currentDistance = rightDistanceSensor.getDistance(DistanceUnit.CM);//rightDistanceSensor
                } else if (side.equals("FRONT")) {
                    currentDistance = frontDistanceSensor.getDistance(DistanceUnit.CM);
                } else if (side.equals("BACK") || side.equals("FOUR_SECOND")) {
                    currentDistance = backDistanceSensor.getDistance(DistanceUnit.CM);
                } else if (side.equals("RIGHT_ZERO")) {
                    currentDistance = rightDistanceSensor.getDistance(DistanceUnit.CM);
                }
                telemetry.update();
            }
        }

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

//    public static void DriveAndTwist(double x, double y, double z, int timer, Telemetry telemetry){
//        double speed = Math.sqrt(2) * Math.hypot(x, y);
//        double command = Math.atan2(y, -x) + Math.PI/2;
//        double rotation = z;
//
//        angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//        double adjustedXHeading = Math.cos(command + angles.firstAngle + Math.PI/4);
//        double adjustedYHeading = Math.sin(command + angles.firstAngle + Math.PI/4);
//
//        driveTrainError = angles.firstAngle - Math.PI;
//
//        while(timer > 0){
//            driveTrainError = angles.firstAngle - Math.PI;
//            if(Math.abs(driveTrainError) > (Math.PI / 30)){
//                rotation = 0;
//            }
//            if(driveTrainError < 0){
//                rotation = -z;
//            }
//
//            DriveTrain.leftFrontMotor.setPower((speed * adjustedYHeading + rotation) * Constants.TELEOP_LIMITER);
//            DriveTrain.rightFrontMotor.setPower((speed * adjustedXHeading - rotation) * Constants.TELEOP_LIMITER);
//            DriveTrain.leftBackMotor.setPower((speed * adjustedXHeading + rotation) * Constants.TELEOP_LIMITER);
//            DriveTrain.rightBackMotor.setPower((speed * adjustedYHeading - rotation) * Constants.TELEOP_LIMITER);
//            timer--;
//        }
//
//        DriveTrain.leftFrontMotor.setPower(0);
//        DriveTrain.rightFrontMotor.setPower(0);
//        DriveTrain.leftBackMotor.setPower(0);
//        DriveTrain.rightBackMotor.setPower(0);
//    }

    public static void autoAlign(){
        driveTrainError = angles.firstAngle - 0;
        if(Math.abs(driveTrainError) > (Math.PI / 6)){
            driveTrainPower = .65;
        }
        else{
            if(Math.abs(driveTrainError) < (Math.PI / 90)){
                driveTrainPower = 0;
            }
            else if(Math.abs(driveTrainError) > (Math.PI / 90)) {
                driveTrainPower = Math.abs(driveTrainError / (Math.PI / 3.0)) + 0.13; //  (Math.PI / 5.2)) + 0.1
            }
        }
        if(driveTrainError > 0){
            cartesianDrive(0, 0, driveTrainPower);
        }
        else if(driveTrainError < 0){
            cartesianDrive(0, 0, -driveTrainPower);
        }
    }

    public static void autoAlignAuto(double finalAngle){
        driveTrainError = angles.firstAngle - finalAngle;
        if(Math.abs(driveTrainError) > (Math.PI / 6)){
            driveTrainPower = 0.5;
        }
        else{
            if(Math.abs(driveTrainError) < (Math.PI / 90)){//60
                driveTrainPower = 0;
            }
            else if(Math.abs(driveTrainError) > (Math.PI / 90)) {//60
                driveTrainPower = Math.abs(driveTrainError / (Math.PI / 2)) + 0.1;
            }
        }
        driveTrainError = angles.firstAngle - finalAngle;
        if(driveTrainError > 0){
            cartesianDrive(0, 0, driveTrainPower);
        }
        else if(driveTrainError < 0){
            cartesianDrive(0, 0, -driveTrainPower);
        }
    }

    public static void driveToLine(double power, String color, Telemetry telemetry) throws InterruptedException {
        double minBlue = Double.MAX_VALUE;
        double maxBlue = Double.MIN_VALUE;

        double minWhite = Double.MAX_VALUE;
        double maxWhite = Double.MIN_VALUE;
        if(color.equals("RED")){
            while(floorColorSensor.red() < 1350){ //1600
                leftFrontMotor.setPower(power);
                rightFrontMotor.setPower(power);
                leftBackMotor.setPower(power);
                rightBackMotor.setPower(power);
            }
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
        }
        else if(color.equals("BLUE")){
            while(floorColorSensor.blue() < 1600){//2100
                if(DriveTrain.floorColorSensor.blue() > maxBlue){
                    maxBlue = DriveTrain.floorColorSensor.blue();
                }

                if (DriveTrain.floorColorSensor.blue() < minBlue){
                    minBlue = DriveTrain.floorColorSensor.blue();
                }
                telemetry.addData("Max Blue: ", maxBlue);
                telemetry.addData("Min Blue: ", minBlue);
                telemetry.update();
                leftFrontMotor.setPower(power);
                rightFrontMotor.setPower(power);
                leftBackMotor.setPower(power);
                rightBackMotor.setPower(power);
            }
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
        }
        else if(color.equals("WHITE")) {
            while(floorColorSensor.alpha() < 3500){//480, 680
                if(DriveTrain.floorColorSensor.alpha() > maxWhite){
                    maxWhite = DriveTrain.floorColorSensor.alpha();
                }

                if (DriveTrain.floorColorSensor.alpha() < minWhite){
                    minWhite = DriveTrain.floorColorSensor.alpha();
                }
                leftFrontMotor.setPower(power);
                rightFrontMotor.setPower(power);
                leftBackMotor.setPower(power);
                rightBackMotor.setPower(power);
//                Intake.releaseAll();
                telemetry.addData("Max White: ", maxWhite);
                telemetry.addData("Min White: ", minWhite);
                telemetry.update();
            }
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
        }
    }

    public static void setRunMode(String input) {
        if (input.equals("STOP_AND_RESET_ENCODER")) {
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (input.equals("RUN_WITHOUT_ENCODER")) {
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (input.equals("RUN_USING_ENCODER")) {
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (input.equals("RUN_TO_POSITION")) {
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public static void gyroTurn(double finalAngle, int timer){
        int turnTimer = timer;
        while (Math.abs(DriveTrain.angles.firstAngle - (finalAngle)) > (Math.PI / 60) && turnTimer > 0){
            driveTrainError = angles.firstAngle - finalAngle;
            if(Math.abs(driveTrainError) > (Math.PI / 6)){
                driveTrainPower = 0.5;
            }
            else{
                if(Math.abs(driveTrainError) < (Math.PI / 60)){//60
                    driveTrainPower = 0;
                }
                else if(Math.abs(driveTrainError) > (Math.PI / 60)) {//60
                    driveTrainPower = Math.abs(driveTrainError / (Math.PI / 2)) + 0.1;
                }
            }
            driveTrainError = angles.firstAngle - finalAngle;
            if(driveTrainError > 0){
                cartesianDrive(0, 0, driveTrainPower);
            }
            else if(driveTrainError < 0){
                cartesianDrive(0, 0, -driveTrainPower);
            }
            turnTimer--;
        }
        DriveTrain.rightFrontMotor.setPower(0);
        DriveTrain.rightBackMotor.setPower(0);
        DriveTrain.leftFrontMotor.setPower(0);
        DriveTrain.leftBackMotor.setPower(0);
    }

    public static void driveTelemetry(Telemetry telemetry){
        telemetry.addData("left front encoder", leftFrontMotor.getCurrentPosition());
        telemetry.addData("left back encoder", leftBackMotor.getCurrentPosition());
        telemetry.addData("right front encoder", rightFrontMotor.getCurrentPosition());
        telemetry.addData("right back encoder", rightBackMotor.getCurrentPosition());
//        telemetry.addLine();
//        telemetry.addData("left front power", leftFrontMotor.getPower());
//        telemetry.addData("left back power", leftBackMotor.getPower());
//        telemetry.addData("right front power", rightFrontMotor.getPower());
//        telemetry.addData("right back power", rightBackMotor.getPower());
//        telemetry.addData("DriveTrainError", driveTrainError);
//        telemetry.addData("DriveTrainPower", driveTrainPower);
//        telemetry.addLine();
        telemetry.addLine();
                    telemetry.addLine()
                    .addData("Red", "%.3f", (double) floorColorSensor.red())
                    .addData("Blue", "%.3f", (double) floorColorSensor.blue())
                    .addData("Alpha", "%.3f", (double) floorColorSensor.alpha());
//        telemetry.addLine();
//        telemetry.addData("range1", String.format("%.3f cm", Constants.Distance1.getAverage() + Constants.cal1));
//        telemetry.addData("range2", String.format("%.3f cm",Constants.Distance2.getAverage() + Constants.cal2));
//        telemetry.addLine();
        telemetry.addData("Front", String.format("%.3f cm", frontDistanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("rightWallDistance", rightDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("BaccDistanceSensor", backDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addLine();
    }

    public static void SUMO_MODE(){

            DriveTrain.leftFrontMotor.setTargetPosition(0);
            DriveTrain.leftBackMotor.setTargetPosition(0);
            DriveTrain.rightFrontMotor.setTargetPosition(0);
            DriveTrain.rightBackMotor.setTargetPosition(0);
            DriveTrain.setRunMode("RUN_TO_POSITION");
            DriveTrain.leftFrontMotor.setPower(0.8);
            DriveTrain.leftBackMotor.setPower(0.8);
            DriveTrain.rightFrontMotor.setPower(0.8);
            DriveTrain.rightBackMotor.setPower(0.8);

    }

    public static void composeTelemetry (Telemetry telemetry) {

        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                gravity = imu.getGravity();
            }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.RADIANS.fromUnit(angleUnit, angle));
    }
    static String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.RADIANS.normalize(degrees));
    }
    public static void resetGyro(){
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters1);

        angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }
}