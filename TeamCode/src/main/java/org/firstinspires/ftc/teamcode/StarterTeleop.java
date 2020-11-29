package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Util.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@TeleOp(name= "TeleOop", group= "TeleOp")
//@Disabled
public class StarterTeleop extends LinearOpMode {

    HardwarePresets robot = new HardwarePresets();
    //Constants constant = new Constants();
    public double drive = 0;
    public double strafe = 0;
    public double twist = 0;
    public final double TELEOP_LIMITER = 0.5;
    public float gyroVariation = 0;

    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;

    public Rolling Distance1 = new Rolling(20);
    public Rolling Distance2 = new Rolling (20);

    @Override
    public void runOpMode(){

        robot.init(hardwareMap);

        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);

//        telemetry.addLine()
//                .addData("heading", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return formatAngle(angles.angleUnit, angles.firstAngle);
//                    }
//                })
//                .addData("roll", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return formatAngle(angles.angleUnit, angles.secondAngle);
//                    }
//                })
//                .addData("pitch", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return formatAngle(angles.angleUnit, angles.thirdAngle);
//                    }
//                });

        waitForStart();

        while(opModeIsActive()){

            // 2m Distance sensor stuff
            DistanceSensor sensorRange;
            sensorRange = hardwareMap.get(DistanceSensor.class, "laserboi");
            Distance1.add(sensorRange.getDistance(DistanceUnit.METER));
            DistanceSensor sensorRange2;
            sensorRange2 = hardwareMap.get(DistanceSensor.class, "pewpewboi");
            Distance2.add(sensorRange2.getDistance(DistanceUnit.METER));

            //colorsensor stuff
            NormalizedColorSensor colorSensor;
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "cranberi");
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            robot.cranberi.enableLed(true);
            //Mecanum Drive
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double heading;

            if(x == 0 && y == 0)
                heading = 0;
            else if(x >= 0)
                heading = Math.PI - Math.atan(y / x);
            else
                heading = -Math.atan(y / x);

            double pow = Math.sqrt(Math.pow(x, 6) + Math.pow(y, 6));
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            heading -= (-angles.firstAngle + Math.PI / 4.0);

            double pow1 = Math.sqrt(2) * pow * Math.cos(heading + gyroVariation);//negative 1
            double pow2 = Math.sqrt(2) * pow * Math.sin(heading + gyroVariation);//positive 1

            robot.leftFrontMotor.setPower((turn + pow2) * TELEOP_LIMITER);//n
            robot.leftBackMotor.setPower((turn + pow1) * TELEOP_LIMITER);//p
            robot.rightFrontMotor.setPower((pow1 - turn) * TELEOP_LIMITER);//n
            robot.rightBackMotor.setPower((pow2 - turn) * TELEOP_LIMITER);//p

            if(gamepad1.x){
                gyroVariation = angles.firstAngle;
            }
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red * 255)
                    .addData("Blue", "%.3f", colors.blue * 255)
                    .addData("Alpha", "%.3f", colors.alpha * 255);
            telemetry.addData("range1", String.format("%.3f m",Distance1.getAverage() ));
            telemetry.addData("range2", String.format("%.3f m",Distance2.getAverage() ));
            telemetry.update();
//            drive  = gamepad1.left_stick_y * TELEOP_LIMITER;
//            strafe = gamepad1.left_stick_x * TELEOP_LIMITER;
//            twist  = gamepad1.right_stick_x * TELEOP_LIMITER;
//
//            robot.leftFrontMotor.setPower(drive + strafe + twist);
//            robot.leftBackMotor.setPower(drive - strafe + twist);
//            robot.rightFrontMotor.setPower(drive - strafe - twist);
//            robot.rightBackMotor.setPower(drive + strafe - twist);
        }
    }

    public void composeTelemetry () {


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