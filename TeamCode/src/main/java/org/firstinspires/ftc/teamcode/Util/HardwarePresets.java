package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class HardwarePresets extends LinearOpMode {

    public void runOpMode() throws InterruptedException {}
    public static HardwareMap HwMap;

    //2m distance sensors
    public DistanceSensor laserboi; //Control hub, I2C Bus 2
    public DistanceSensor pewpewboi; //Control hub, I2C Bus 3

    //Color Sensors
    public ColorSensor cranberi; //Control hub, I2C Bus 1
    public ColorSensor orngi; //Expansion hub, I2C Bus 1
    //public ColorSensor tanjereen;

    //Motors
    public DcMotor leftFrontMotor; //Expansion hub, port 0
    public DcMotor leftBackMotor; //Expansion hub, port 1
    public DcMotor rightFrontMotor; //Expansion hub, port 3
    public DcMotor rightBackMotor; //Expansion hub, port 2

    public DcMotor frontIntakeMotor; //Control hub, port 1
    public DcMotor rearIntakeMotor; //Control hub, port 2

    public DcMotorEx grapfroot; //Control hub, port 0

    //Servo
    public Servo vibrator; //Control hub, port  0

    //Other
    public BNO055IMU imu; //Control hub, I2C Bus 0
    public Orientation angles;
    public Acceleration gravity;

    //Contructor
    public HardwarePresets(){}

    public void init(HardwareMap hwm){
        //Set up Drive Train Motors
        HwMap = hwm;

        frontIntakeMotor = HwMap.dcMotor.get("frontIntakeMotor");
        rearIntakeMotor = HwMap.dcMotor.get("rearIntakeMotor");

        leftFrontMotor = HwMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = HwMap.dcMotor.get("leftBackMotor");
        rightFrontMotor = HwMap.dcMotor.get("rightFrontMotor");
        rightBackMotor = HwMap.dcMotor.get("rightBackMotor");

        grapfroot = (DcMotorEx)HwMap.get(DcMotor.class, "grapfroot");
        vibrator = HwMap.servo.get("vibrator");
//
        laserboi = HwMap.get(DistanceSensor.class, "laserboi");
        pewpewboi = HwMap.get(DistanceSensor.class, "pewpewboi");
        cranberi = HwMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "cranberi");
        orngi = HwMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "orngi");
//        tanjereen = HwMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "tanjereen");

//        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
