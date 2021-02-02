package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
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

    public static double NEW_P = 30.0;//18.6
    public static double NEW_I = 2.0;
    public static double NEW_D = 0.4;
    public static double NEW_F = 0;

    public Rolling Distance1 = new Rolling(20);
    public Rolling Distance2 = new Rolling (20);
    public double cal1 = 0;
    public double cal2 = 0;

    public Intake mainIntake;
    public boolean gamepadA = false;

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

        mainIntake = new Intake();

        robot.vibrator.setPosition(0.53);

        robot.frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.grapfroot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.grapfroot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.grapfroot.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive()){

            // get the PID coefficients for the RUN_USING_ENCODER  modes.
            PIDFCoefficients pidOrig = robot.grapfroot.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            // change coefficients using methods included with DcMotorEx class.
            PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
            robot.grapfroot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

            // re-read coefficients and verify change.
            PIDFCoefficients pidModified = robot.grapfroot.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            // 2m Distance sensor stuff
            Distance1.add(robot.laserboi.getDistance(DistanceUnit.CM));
            Distance2.add(robot.pewpewboi.getDistance(DistanceUnit.CM));

            //colorsensor stuff
            NormalizedColorSensor colorSensor;
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "cranberi");
            robot.cranberi.enableLed(true);

            NormalizedColorSensor colorSensor2;
            colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "orngi");
            robot.orngi.enableLed(true);


            //Mecanum Drive
//            double x = gamepad1.left_stick_x;
//            double y = -gamepad1.left_stick_y;
//            double turn = gamepad1.right_stick_x;
//            double heading;
//
//            if(x == 0 && y == 0)
//                heading = 0;
//            else if(x >= 0)
//                heading = Math.PI - Math.atan(y / x);
//            else
//                heading = -Math.atan(y / x);
//
//            double pow = Math.sqrt(Math.pow(x, 6) + Math.pow(y, 6));
//            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//            heading -= (-angles.firstAngle + Math.PI / 4.0);
//
//            double pow1 = Math.sqrt(2) * pow * Math.cos(heading + gyroVariation);//negative 1
//            double pow2 = Math.sqrt(2) * pow * Math.sin(heading + gyroVariation);//positive 1
//
//            robot.leftFrontMotor.setPower((turn + pow2) * TELEOP_LIMITER);//n
//            robot.leftBackMotor.setPower((turn + pow1) * TELEOP_LIMITER);//p
//            robot.rightFrontMotor.setPower((pow1 - turn) * TELEOP_LIMITER);//n
//            robot.rightBackMotor.setPower((pow2 - turn) * TELEOP_LIMITER);//p
//
            //Drive
            double speed = Math.sqrt(2) * Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double command = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.PI/2;
            double rotation = -gamepad1.right_stick_x;

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double adjustedXHeading = Math.cos(command + angles.firstAngle + Math.PI/4);
            double adjustedYHeading = Math.sin(command + angles.firstAngle + Math.PI/4);

            robot.leftFrontMotor.setPower((speed * adjustedYHeading + rotation) * TELEOP_LIMITER);
            robot.rightFrontMotor.setPower((speed * adjustedXHeading - rotation) * TELEOP_LIMITER);
            robot.leftBackMotor.setPower((speed * adjustedXHeading + rotation) * TELEOP_LIMITER);
            robot.rightBackMotor.setPower((speed * adjustedYHeading - rotation) * TELEOP_LIMITER);

//            final double v1 = -gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
//            final double v2 = -gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;
//            final double v3 = -gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
//            final double v4 = -gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;
//
//            robot.leftFrontMotor.setPower(v1);
//            robot.leftBackMotor.setPower(v2);
//            robot.rightFrontMotor.setPower(v3);
//            robot.rightBackMotor.setPower(v4);
//
//            robot.leftFrontMotor.setPower(.5);
//            robot.leftBackMotor.setPower(.5);
//            robot.rightFrontMotor.setPower(.5);
//            robot.rightBackMotor.setPower(.5);

//            robot.leftFrontMotor.setPower(gamepad1.left_stick_y);
//            robot.leftBackMotor.setPower(gamepad1.left_stick_y);
//            robot.rightFrontMotor.setPower(gamepad1.left_stick_y);
//            robot.rightBackMotor.setPower(gamepad1.left_stick_y);
//            if(gamepad1.x){
//                gyroVariation = angles.firstAngle;
//            }
//            if (gamepad1.a){
//               gamepadA = !gamepadA;
//            }
////            if(gamepadA){
////                mainIntake.intake();
////            }
            if(gamepad1.x){
//                mainIntake.intakeTwo();
                robot.frontIntakeMotor.setPower(.85);
                robot.rearIntakeMotor.setPower(.85);
            }
//
            if(gamepad1.b){
                robot.frontIntakeMotor.setPower(-0.85);
                robot.rearIntakeMotor.setPower(-0.85);
            }

            if(gamepad1.a) {
                robot.grapfroot.setPower(0.55);
            }

            if(gamepad1.y)
                robot.grapfroot.setPower(0);

            if(gamepad1.dpad_down){
                robot.frontIntakeMotor.setPower(0);
                robot.rearIntakeMotor.setPower(0);
            }
//            if(gamepad1.dpad_up){
//                robot.leftFrontMotor.setPower(.5);
//                robot.leftBackMotor.setPower(.5);
//                robot.rightFrontMotor.setPower(.5);
//                robot.rightBackMotor.setPower(.5);
//            }
            if (gamepad1.dpad_right) {
                robot.vibrator.setPosition(0.65);
                sleep(150);
                robot.vibrator.setPosition(0.45);
                sleep(75);
            }
            else
                robot.vibrator.setPosition(.60);


//            telemetry.addLine()
//                    .addData("Red", "%.3f", (double) robot.cranberi.red())
//                    .addData("Blue", "%.3f", (double) robot.cranberi.blue())
//                    .addData("Alpha", "%.3f", (double) robot.cranberi.alpha());

            //telemetry.addLine()
//                    .addData("Red", "%.3f", (double) robot.orngi.red())
//                    .addData("Blue", "%.3f", (double) robot.orngi.blue())
//                    .addData("Alpha", "%.3f", (double) robot.orngi.alpha());

//            telemetry.addData("range1", String.format("%.3f cm",Distance1.getAverage() + cal1));
//            telemetry.addData("range2", String.format("%.3f cm",Distance2.getAverage() + cal2));
//            telemetry.addData("laserboi", String.format("%.3f cm", robot.laserboi.getDistance(DistanceUnit.CM)));
//            telemetry.addData("pewpewboi", String.format("%.3f m", robot.pewpewboi.getDistance(DistanceUnit.CM)));
//            telemetry.addData("skewAngle", String.format("%.3f °",180*(Math.atan((Distance2.getAverage()-Distance1.getAverage())/0.15))/Math.PI));
            telemetry.addData("left front encoder", robot.leftFrontMotor.getCurrentPosition());
            telemetry.addData("left back encoder", robot.leftBackMotor.getCurrentPosition());
            telemetry.addData("right front encoder", robot.rightFrontMotor.getCurrentPosition());
            telemetry.addData("right back encoder", robot.rightBackMotor.getCurrentPosition());
            telemetry.addLine();

//            telemetry.addData("Pow", pow);
//            telemetry.addData("Pow1", pow1);
//            telemetry.addData("Pow2", pow2);

            telemetry.addData("left front power", robot.leftFrontMotor.getPower());
            telemetry.addData("left back power", robot.leftBackMotor.getPower());
            telemetry.addData("right front power", robot.rightFrontMotor.getPower());
            telemetry.addData("right back power", robot.rightBackMotor.getPower());
            telemetry.addData("leftstick y value: ", gamepad1.left_stick_y);
            telemetry.addData("leftstick x value: ", gamepad1.left_stick_x);
            telemetry.addData("current angle: ", angles.firstAngle);
            telemetry.addData("command heading: ", command);
            telemetry.addLine();

            telemetry.addData("Vibrator:", robot.vibrator.getPosition());
            //telemetry.addData("Inake Array Size:", mainIntake.rings.lastIndexOf(true)) ;
            telemetry.addData("grapfroot encoder", robot.grapfroot.getCurrentPosition());
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                    pidOrig.p, pidOrig.i, pidOrig.d);
            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                    pidModified.p, pidModified.i, pidModified.d);
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