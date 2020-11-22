package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

        waitForStart();

        while(opModeIsActive()){

            //Mecanum Drive
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
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
            heading -= (angles.firstAngle + Math.PI / 4.0);

            double pow1 = Math.sqrt(2) * pow * Math.cos(heading + gyroVariation);
            double pow2 = Math.sqrt(2) * pow * Math.sin(heading + gyroVariation);

            robot.leftFrontMotor.setPower((turn + pow1) * TELEOP_LIMITER);
            robot.leftBackMotor.setPower(-(turn + pow2) * TELEOP_LIMITER);
            robot.rightFrontMotor.setPower((pow1 - turn) * TELEOP_LIMITER);
            robot.rightBackMotor.setPower(-(pow2 - turn) * TELEOP_LIMITER);

            if(gamepad1.x){
                gyroVariation = angles.firstAngle;
            }


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