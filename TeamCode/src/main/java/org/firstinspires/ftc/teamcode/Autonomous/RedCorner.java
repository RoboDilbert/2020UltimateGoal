package org.firstinspires.ftc.teamcode.Autonomous;

import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Util.HardwarePresets;
import org.firstinspires.ftc.teamcode.Util.SensorColor;

import java.util.Locale;

@Autonomous(name= "RedCorner", group= "Autonomous")

public class RedCorner extends LinearOpMode {

    HardwarePresets robot = new HardwarePresets();
    SensorColor color = new SensorColor();
    DriveTrain drive = new DriveTrain();

    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;

    public void runOpMode() throws InterruptedException {
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

        //Shoot shot
        //View stack and pick zone
        //Drive forward until we see red line
        drive.setRunMode("RUN_USING_ENCODER");
        Thread.sleep(1000);
        color.DriveToLine("RED");


        //If A, drop thingo

        //If B, go white, red, a little farther, turn, drop boyo, straighten back out
//        color.DriveToLine("WHITE");
//        Thread.sleep(1000);
//        color.DriveToLine("RED");
//        Thread.sleep(1000);
//        drive.Turn("TURN_LEFT", 0.2, 90);
//        Thread.sleep(1000);
//        drive.Turn("TURN_RIGHT", 0.2, 90);
//
//        //If C, go white red, red, drop boyo
//        color.DriveToLine("RED");
//        Thread.sleep(1000);
//        color.DriveToLine("RED");
//        Thread.sleep(1000);
//        drive.setRunMode("RUN_TO_POSITION");
//        Thread.sleep(50);
//        drive.Drive("FORWARD", 200, 0.2);
//        Thread.sleep(1000);
//
//        //Drive backwards to white
//        color.DriveToLine("WHITE");

    }

    static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
