package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import java.util.Locale;

public class HardwarePresets extends LinearOpMode {

    public void runOpMode() throws InterruptedException {}
    public static HardwareMap HwMap;

    public DcMotorEx shooter; //Control hub, port 0
    public Shooter mainShooter;
    public static double NEW_P = 50.0;//18.6
    public static double NEW_I = 2.0;
    public static double NEW_D = 0.4;
    public static double NEW_F = 0;

    public DcMotor wobbleMotor;

    //Servo
    public Servo vibrator; //Control hub, port  0
    public Servo grabber; //Control hub, port 1
    public Servo angleAdjust; //Control hub, port

    //Wobble
    public Servo wobble1;
    public Servo wobble2;

    //Other
    public BNO055IMU imu; //Control hub, I2C Bus 0
    public Orientation angles;
    public Acceleration gravity;

    //Contsructor
    public HardwarePresets(){}

    public void init(HardwareMap hwm){
        //Set up Drive Train Motors
        HwMap = hwm;

        frontIntakeMotor = HwMap.dcMotor.get("frontIntakeMotor");
        rearIntakeMotor = HwMap.dcMotor.get("rearIntakeMotor");

        shooter = HwMap.get(DcMotorEx.class, "shooter");
        mainShooter = new Shooter(NEW_P, NEW_I, NEW_D, NEW_F);
        wobbleMotor = HwMap.dcMotor.get("wobbleMotor");

        vibrator = HwMap.servo.get("vibrator");
        angleAdjust = HwMap.servo.get("angleAdjust");
        grabber = HwMap.servo.get("grabber");
        wobble1 = HwMap.servo.get("wobble1");
        wobble2 = HwMap.servo.get("wobble2");


        indexSensor = HwMap.get(DistanceSensor.class, "indexSensor");
        //indexColorSensor = HwMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "indexColorSensor");


        wobbleMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
