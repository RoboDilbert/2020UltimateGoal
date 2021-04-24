package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Const;
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
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

public class DriveTrain {

    //Declare motors, servos, sensors, and imu
    public static DcMotor leftFrontMotor; //Expansion hub, port 0
    public static DcMotor leftBackMotor; //Expansion hub, port 1
    public static DcMotor rightFrontMotor; //Expansion hub, port 3
    public static DcMotor rightBackMotor; //Expansion hub, port 2

    //These are the dead wheel encoders
    public static DcMotor verticalLeft;
    public static DcMotor verticalRight;
    public static DcMotor horizontal;

    public static RevBlinkinLedDriver blinkinLedDriver;

    public static ColorSensor floorColorSensor; //Expansion hub, I2C Bus 3

    public static BNO055IMU imu; //Control hub, I2C Bus 0
    public static Orientation angles;
    public static Acceleration gravity;
    private static float gyroVariation = 0;

    public static double driveTrainError = 0;
    public static double driveTrainPower = 0;

//    private static Servo blinkin;

    public static DistanceSensor frontDistanceSensor; //Expansion hub, I2C Bus 2;
    public static DistanceSensor backDistanceSensor; //Control hub, I2C Bus 2
    public static DistanceSensor leftDistanceSensor; //Control hub, I2C Bus 3;
    public static DistanceSensor rightDistanceSensor;//Control hub, I2C Bus 1

    //Constructor
    public DriveTrain(){}

    public static void initDriveTrain(HardwareMap hwm) {
        //Create hwm
        Constants.HwMap = hwm;

        //Declare motors and stuff on the hardware map
        leftFrontMotor = Constants.HwMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = Constants.HwMap.dcMotor.get("leftBackMotor");
        rightFrontMotor = Constants.HwMap.dcMotor.get("rightFrontMotor");
        rightBackMotor = Constants.HwMap.dcMotor.get("rightBackMotor");

//        blinkin = Constants.HwMap.servo.get("blinkin");
        blinkinLedDriver = Constants.HwMap.get(RevBlinkinLedDriver.class, "blinkin");

        floorColorSensor = Constants.HwMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "floorColorSensor");

        frontDistanceSensor = Constants.HwMap.get(DistanceSensor.class, "frontDistanceSensor");
        backDistanceSensor = Constants.HwMap.get(DistanceSensor.class, "backDistanceSensor");
        leftDistanceSensor = Constants.HwMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = Constants.HwMap.get(DistanceSensor.class, "rightDistanceSensor");

        //The dead wheels are declared with the motor names of the ports they are plugged into
        verticalLeft = Constants.HwMap.dcMotor.get("rightBackMotor");
        verticalRight = Constants.HwMap.dcMotor.get("leftFrontMotor");
        horizontal = Constants.HwMap.dcMotor.get("rightFrontMotor");

        imu = Constants.HwMap.get(BNO055IMU.class, "imu");

        //Set directions for motors and run modes
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);//AQUA

        //Intialize imu with parameters
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

    //This is our main cartesian drive method that is used in teleop
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

    //This is our autonomous cartesian drive method. It utilizes the same math as the normal cartesian drive method
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

    //This is similar to our normal method but it drives forward until the front distance sensor reads less than 10 cm
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

    //The method uses the distance sensors to drive until a certain distance sensors reads greater or less than a certain value
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

    //This method uses the gyro to automatically. set our orientation to 0 degrees. This is used to align ourselves in teleop.
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

    //This does the same as the last method but in auto. It also allows you to change the desired orientation.
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

    //This method drives until the color sensor on the bottom reads a certain value. We can use this to drive to the red, white, and blue lines on the field.
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

    //Set run mode for drive motors
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

    //Turn using the gyro in auto
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

    //Telemetry for drive motors (this changes very often)
    public static void driveTelemetry(Telemetry telemetry){
        telemetry.addData("left front encoder", leftFrontMotor.getCurrentPosition());
        telemetry.addData("left back encoder", leftBackMotor.getCurrentPosition());
        telemetry.addData("right front encoder", rightFrontMotor.getCurrentPosition());
        telemetry.addData("right back encoder", rightBackMotor.getCurrentPosition());
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

    //This is our favorite method by far. It sets the encoder positions to zero and stays there. This way, we can move at the end of auto.
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

//    public static void ledOn(){
//        blinkin.setPosition(.5);
//    }
    //Telemetry for imu stuff (very boring)
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

    //More imu stuff (also very boring)
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