package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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
    public double cal1 = 0;
    public double cal2 = 0;

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

        waitForStart();

        while(opModeIsActive()){

            // 2m Distance sensor stuff
            Distance1.add(robot.laserboi.getDistance(DistanceUnit.CM));
            Distance2.add(robot.pewpewboi.getDistance(DistanceUnit.CM));

            //colorsensor stuff
            NormalizedColorSensor colorSensor;
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "cranberi");
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
                    .addData("Red", "%.3f", (double) robot.cranberi.red())
                    .addData("Blue", "%.3f", (double) robot.cranberi.blue())
                    .addData("Alpha", "%.3f", (double) robot.cranberi.alpha());

            telemetry.addData("range1", String.format("%.3f cm",Distance1.getAverage() + cal1));
            telemetry.addData("range2", String.format("%.3f cm",Distance2.getAverage() + cal2));
            telemetry.addData("laserboi", String.format("%.3f cm", robot.laserboi.getDistance(DistanceUnit.CM)));
            telemetry.addData("pewpewboi", String.format("%.3f m", robot.pewpewboi.getDistance(DistanceUnit.CM)));
            telemetry.addData("skewAngle", String.format("%.3f °",180*(Math.atan((Distance2.getAverage()-Distance1.getAverage())/0.15))/Math.PI));

            telemetry.update();
        }
    }
    static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}