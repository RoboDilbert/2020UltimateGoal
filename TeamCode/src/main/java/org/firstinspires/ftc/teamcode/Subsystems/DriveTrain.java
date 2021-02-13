package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Util.Constants;

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

    public static ColorSensor floorColorSensor; //Control hub, I2C Bus 1

    public static BNO055IMU imu; //Control hub, I2C Bus 0
    public static Orientation angles;
    public static Acceleration gravity;

    //2m distance sensors
    public static DistanceSensor driveDistanceSensor; //Control hub, I2C Bus 2;

    public static void initDriveTrain(HardwareMap hwm) {

        Constants.HwMap = hwm;
        leftFrontMotor = Constants.HwMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = Constants.HwMap.dcMotor.get("leftBackMotor");
        rightFrontMotor = Constants.HwMap.dcMotor.get("rightFrontMotor");
        rightBackMotor = Constants.HwMap.dcMotor.get("rightBackMotor");

        floorColorSensor = Constants.HwMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "floorColorSensor");

        driveDistanceSensor = Constants.HwMap.get(DistanceSensor.class, "driveDistanceSensor");

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
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters1);
    }

    //Constructor
    public DriveTrain(){}

    public static void cartesianDrive(double x, double y, double z){
        double speed = Math.sqrt(2) * Math.hypot(x, y);
        double command = Math.atan2(y, -x) + Math.PI/2;
        double rotation = z;

        angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double adjustedXHeading = Math.cos(command + angles.firstAngle + Math.PI/4);
        double adjustedYHeading = Math.sin(command + angles.firstAngle + Math.PI/4);

        DriveTrain.leftFrontMotor.setPower((speed * adjustedYHeading + rotation) * Constants.TELEOP_LIMITER);
        DriveTrain.rightFrontMotor.setPower((speed * adjustedXHeading - rotation) * Constants.TELEOP_LIMITER);
        DriveTrain.leftBackMotor.setPower((speed * adjustedXHeading + rotation) * Constants.TELEOP_LIMITER);
        DriveTrain.rightBackMotor.setPower((speed * adjustedYHeading - rotation) * Constants.TELEOP_LIMITER);
    }

    public static void cartesianDriveAuto(double x, double y, double z, int timerLength, Telemetry telemetry) throws InterruptedException {
        double speed = Math.sqrt(2) * Math.hypot(x, y);
        double command = Math.atan2(y, -x) + Math.PI/2;
        double rotation = z;
        double adjustedXHeading = 0;
        double adjustedYHeading = 0;


        while(timerLength > 0) {
            angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            adjustedXHeading = Math.cos(command + angles.firstAngle + Math.PI / 4);
            adjustedYHeading = Math.sin(command + angles.firstAngle + Math.PI / 4);

            leftFrontMotor.setPower((speed * adjustedYHeading + rotation) * Constants.TELEOP_LIMITER);
            rightFrontMotor.setPower((speed * adjustedXHeading - rotation) * Constants.TELEOP_LIMITER);
            leftBackMotor.setPower((speed * adjustedXHeading + rotation) * Constants.TELEOP_LIMITER);
            rightBackMotor.setPower((speed * adjustedYHeading - rotation) * Constants.TELEOP_LIMITER);
            DriveTelemetry(telemetry);
            Thread.sleep(20);
            telemetry.addData("Counter", timerLength);
            telemetry.update();
            timerLength--;
        }

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }


//    public void Drive(String input, int encoderTicks, double power){
//        if(input.equals("STRAFE_RIGHT")){
//            leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + encoderTicks);
//            leftBackMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - encoderTicks);
//            rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - encoderTicks);
//            rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + encoderTicks);
//            setRunMode("RUN_TO_POSITION");
//
//            while(anyDriveMotorsBusy()) {
//                leftFrontMotor.setPower(power);
//                leftBackMotor.setPower(-power);
//                rightFrontMotor.setPower(-power);
//                rightBackMotor.setPower(power);
//            }
//        }
//        if(input.equals("STRAFE_LEFT")){
//            leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - encoderTicks);
//            leftBackMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + encoderTicks);
//            rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + encoderTicks);
//            rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - encoderTicks);
//            setRunMode("RUN_TO_POSITION");
//
//            while(anyDriveMotorsBusy()){
//                leftFrontMotor.setPower(-power);
//                leftBackMotor.setPower(power);
//                rightFrontMotor.setPower(power);
//                rightBackMotor.setPower(-power);
//            }
//        }
//        if(input.equals("FORWARD")){
//            leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + encoderTicks);
//            leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + encoderTicks);
//            rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + encoderTicks);
//            rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + encoderTicks);
//            setRunMode("RUN_TO_POSITION");
//
//            while(anyDriveMotorsBusy()){
//                leftFrontMotor.setPower(power);
//                leftBackMotor.setPower(power);
//                rightFrontMotor.setPower(power);
//                rightBackMotor.setPower(power);
//            }
//        }
//        if(input.equals("REVERSE")) {
//            leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - encoderTicks);
//            leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - encoderTicks);
//            rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - encoderTicks);
//            rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - encoderTicks);
//            setRunMode("RUN_TO_POSITION");
//
//            while (anyDriveMotorsBusy()) {
//                leftFrontMotor.setPower(-power);
//                leftBackMotor.setPower(-power);
//                rightFrontMotor.setPower(-power);
//                rightBackMotor.setPower(-power);
//            }
//        }
//        if(input.equals("FORWARD_LEFT")){
//            leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - encoderTicks);
//            leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + (6 * encoderTicks));
//            rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + (6 * encoderTicks));
//            rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - encoderTicks);
//            setRunMode("RUN_TO_POSITION");
//
//            while(anyDriveMotorsBusy()){
//                leftFrontMotor.setPower(power * -.5);
//                leftBackMotor.setPower(power * 3);
//                rightFrontMotor.setPower(power * 3);
//                rightBackMotor.setPower(power * -.5);
//            }
//        }
//        if(input.equals("SLIGHTLY_LEFT")){
//            leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - encoderTicks);
//            leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - encoderTicks);
//            rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - (2 * encoderTicks));
//            rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - (2 * encoderTicks));
//            setRunMode("RUN_TO_POSITION");
//
//            while(anyDriveMotorsBusy()){
//                leftFrontMotor.setPower(power * -0.8);
//                leftBackMotor.setPower(power * -0.8);
//                rightFrontMotor.setPower(power * -1.6);
//                rightBackMotor.setPower(power * -1.6);
//            }
//        }
//        if(input.equals("FORWARD_LEFT")){
//            leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - encoderTicks);
//            leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + (6 * encoderTicks));
//            rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + (6 * encoderTicks));
//            rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - encoderTicks);
//            setRunMode("RUN_TO_POSITION");
//
//            while(anyDriveMotorsBusy()){
//                leftFrontMotor.setPower(power * -.5);
//                leftBackMotor.setPower(power * 3);
//                rightFrontMotor.setPower(power * 3);
//                rightBackMotor.setPower(power * -.5);
//            }
//        }
//
//        leftFrontMotor.setPower(0);
//        leftBackMotor.setPower(0);
//        rightFrontMotor.setPower(0);
//        rightBackMotor.setPower(0);
//    }
    public void driveToRing(double power){
        while(driveDistanceSensor.getDistance(DistanceUnit.CM) > 10) {
                leftFrontMotor.setPower(power);
                leftBackMotor.setPower(power);
                rightFrontMotor.setPower(power);
                rightBackMotor.setPower(power);
        }

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

    }

    public void Turn(String input, double power, float degrees){
        if(input.equals("TURN_LEFT")){
            float targetLocation = angles.firstAngle - degrees;
            if(targetLocation < -180){
                targetLocation = Math.abs(targetLocation + 180);
            }
            while (angles.firstAngle < targetLocation - 2 && angles.firstAngle > targetLocation + 2) {
                leftFrontMotor.setPower(-power);
                leftBackMotor.setPower(-power);
                rightFrontMotor.setPower(power);
                rightBackMotor.setPower(power);
            }
        }
        if(input.equals("TURN_RIGHT")){
            float targetLocation = angles.firstAngle + degrees;
            if(targetLocation > 180){
                targetLocation = -(targetLocation - 180);
            }
            while (angles.firstAngle < targetLocation - 2 && angles.firstAngle > targetLocation + 2) {
                leftFrontMotor.setPower(power);
                leftBackMotor.setPower(power);
                rightFrontMotor.setPower(-power);
                rightBackMotor.setPower(-power);
            }
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


    //Returns TRUE if any drive motors are busy and FALSE if not.
    public boolean anyDriveMotorsBusy() {
        if (leftFrontMotor.isBusy() /*|| leftBackMotor.isBusy() || rightFrontMotor.isBusy() || rightBackMotor.isBusy()*/) {
            return (true);
        } else {
            return (false);
        }
    }

    public static void DriveTelemetry(Telemetry telemetry){
//        telemetry.addData("left front encoder", leftFrontMotor.getCurrentPosition());
//        telemetry.addData("left back encoder", leftBackMotor.getCurrentPosition());
//        telemetry.addData("right front encoder", rightFrontMotor.getCurrentPosition());
//        telemetry.addData("right back encoder", rightBackMotor.getCurrentPosition());
//        telemetry.addLine();
        telemetry.addData("left front power", leftFrontMotor.getPower());
        telemetry.addData("left back power", leftBackMotor.getPower());
        telemetry.addData("right front power", rightFrontMotor.getPower());
        telemetry.addData("right back power", rightBackMotor.getPower());
//        telemetry.addLine();
//        telemetry.addLine();
//                    telemetry.addLine()
//                    .addData("Red", "%.3f", (double) floorColorSensor.red())
//                    .addData("Blue", "%.3f", (double) floorColorSensor.blue())
//                    .addData("Alpha", "%.3f", (double) floorColorSensor.alpha());
//        telemetry.addLine();
//        telemetry.addData("range1", String.format("%.3f cm", Constants.Distance1.getAverage() + Constants.cal1));
//        telemetry.addData("range2", String.format("%.3f cm",Constants.Distance2.getAverage() + Constants.cal2));
//        telemetry.addData("laserboi", String.format("%.3f cm", driveDistanceSensor.getDistance(DistanceUnit.CM)));
//        telemetry.addLine();
        telemetry.addData("current angle: ", angles.firstAngle);
        telemetry.addLine();
    }

    public void DriveToLine(String color){
        if(color.equals("RED")){
            while(floorColorSensor.red() < 175){//240, 82
                leftFrontMotor.setPower(0.3);
                rightFrontMotor.setPower(0.3);
                leftBackMotor.setPower(0.3);
                rightBackMotor.setPower(0.3);
            }
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
        }
        else if(color.equals("WHITE")) {
            while(floorColorSensor.red() < 190){//480, 680
                leftFrontMotor.setPower(0.4);
                rightFrontMotor.setPower(0.4);
                leftBackMotor.setPower(0.4);
                rightBackMotor.setPower(0.4);
            }
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
        }
    }

    public static void composeTelemetry (Telemetry telemetry) {

        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}

