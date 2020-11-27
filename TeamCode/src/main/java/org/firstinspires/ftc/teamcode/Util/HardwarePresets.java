package org.firstinspires.ftc.teamcode.Util;

import android.graphics.Color;
import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class HardwarePresets{

    public HardwareMap HwMap;

    //Drive Motors
    public DcMotor leftFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotor rightFrontMotor;
    public DcMotor rightBackMotor;


    //2m distance sensors
    public DistanceSensor laserboi;
    public DistanceSensor pewpewboi;
    //Color Sensors
    public ColorSensor cranberi;

    //IMU variables
    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;

    //Contructor
    public HardwarePresets(){}

    public void init(HardwareMap hwm) {
        //Set up Drive Train Motors
        HwMap = hwm;
        leftFrontMotor = HwMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = HwMap.dcMotor.get("leftBackMotor");
        rightFrontMotor = HwMap.dcMotor.get("rightFrontMotor");
        rightBackMotor = HwMap.dcMotor.get("rightBackMotor");

        laserboi = HwMap.get(DistanceSensor.class, "laserboi");
        pewpewboi = HwMap.get(DistanceSensor.class, "pewpewboi");
        cranberi = HwMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "cranberi");

        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = HwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}