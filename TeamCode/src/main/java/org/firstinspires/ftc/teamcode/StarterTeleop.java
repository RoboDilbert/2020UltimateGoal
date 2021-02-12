package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Wobble;
import org.firstinspires.ftc.teamcode.Util.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@TeleOp(name= "TeleOop", group= "TeleOp")
//@Disabled
public class StarterTeleop extends LinearOpMode{

    public final double TELEOP_LIMITER = 0.8;
    public float gyroVariation = 0;

    public boolean intake = false;

    @Override
    public void runOpMode() throws InterruptedException {

        DriveTrain.initDriveTrain(hardwareMap);
        Wobble.initWobble(hardwareMap);
        Shooter.initShooter(hardwareMap);
        Intake.initIntake(hardwareMap);

        Intake.vibrator.setPosition(0.53);
        Shooter.angleAdjust.setPosition(0.5);

        Intake.frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.rearIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Constants.drive.setRunMode("STOP_AND_RESET_ENCODER");
        Constants.drive.setRunMode("RUN_USING_ENCODER");

        Shooter.shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Wobble.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while(opModeIsActive()){

            // 2m Distance sensor stuff
            Constants.Distance1.add(DriveTrain.driveDistanceSensor.getDistance(DistanceUnit.CM));

            DriveTrain.floorColorSensor.enableLed(true);

            //Mecanum Drive
            double speed = Math.sqrt(2) * Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double command = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.PI/2;
            double rotation = gamepad1.right_stick_x;

            Orientation angles = DriveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double adjustedXHeading = Math.cos(command + angles.firstAngle + Math.PI/4);
            double adjustedYHeading = Math.sin(command + angles.firstAngle + Math.PI/4);

            DriveTrain.leftFrontMotor.setPower((speed * adjustedYHeading + rotation) * TELEOP_LIMITER);
            DriveTrain.rightFrontMotor.setPower((speed * adjustedXHeading - rotation) * TELEOP_LIMITER);
            DriveTrain.leftBackMotor.setPower((speed * adjustedXHeading + rotation) * TELEOP_LIMITER);
            DriveTrain.rightBackMotor.setPower((speed * adjustedYHeading - rotation) * TELEOP_LIMITER);

//            if(gamepad1.x){
//                gyroVariation = angles.firstAngle;
//            }

            if (gamepad1.x){
               intake = !intake;
            }

            if(intake){
                Constants.mainIntake.intake();
            }

            if(!intake){
                Intake.frontIntakeMotor.setPower(0);
                Intake.rearIntakeMotor.setPower(0);
            }

            if(gamepad1.b){
                Intake.frontIntakeMotor.setPower(-1);
                Intake.rearIntakeMotor.setPower(-1);
            }

            if(gamepad1.a) {
                Shooter.angleAdjust.setPosition(0.43);
                Shooter.mainShooter.shoot(0.58);
            }
            if(Shooter.shooter.getPower() == 0) {
                Shooter.angleAdjust.setPosition(0.7);
            }

            //White Line
            if(gamepad2.dpad_up){
                Shooter.angleAdjust.setPosition(0.51);
            }
            //Back Wall
            if(gamepad2.dpad_down){
                Shooter.angleAdjust.setPosition(0.58);
            }
            //In front of rings
            if (gamepad2.dpad_left) {
                Shooter.angleAdjust.setPosition(.5);
            }
//            if(gamepad2.a){
////                robot.grabber.setPosition(.5);
//            }
//
//            if(gamepad2.b) {
////                robot.grabber.setPosition(.25);
//            }

            if(gamepad2.a){
                Wobble.wobble1.setPosition(.12);
                //wobble bottom position
            }
            if(gamepad2.y){
                Wobble.wobble1.setPosition(.08);
                //wobble top position
            }
            if(gamepad2.x){
                Wobble.wobble2.setPosition(.5);
                //wobble grabber close position
            }
            if(gamepad2.b){
                Wobble.wobble2.setPosition(.1);
                //wobble grabber open position
            }
            if(gamepad1.y)
                Shooter.shooter.setPower(0);
               // turn shooter off

            //Lifter
            int lifterTP = -500;
            if(gamepad2.right_bumper){
                Wobble.wobbleMotor.setTargetPosition(lifterTP);
                Wobble.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wobble.wobbleMotor.setPower(-.3);
                if(Math.abs(lifterTP - Wobble.wobbleMotor.getCurrentPosition()) < (-lifterTP * .1)){
                    Wobble.wobbleMotor.setPower(-0.4);
                }
            }
            else if(Math.abs(lifterTP - Wobble.wobbleMotor.getCurrentPosition()) < (-lifterTP * .1) && !gamepad2.right_bumper){
                Wobble.wobbleMotor.setTargetPosition(-350);
                Wobble.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wobble.wobbleMotor.setPower(.3);
                if(Math.abs(Wobble.wobbleMotor.getCurrentPosition()) < (-lifterTP * .1)){
                    Wobble.wobbleMotor.setPower(0.4);
                }
            }

            if (gamepad1.dpad_right) {
                Constants.mainIntake.releaseAll();
            }
            else
                Intake.vibrator.setPosition(.60);

            if(gamepad1.dpad_left){
                Intake.vibrator.setPosition(0.65);
                sleep(150);
                Intake.vibrator.setPosition(0.45);
                sleep(75);
            }
            else
                Intake.vibrator.setPosition(.60);


//            DriveTrain.DriveTelemetry(telemetry);
//            Intake.intakeTelemetry(telemetry);
//            Shooter.shooterTelemetry(telemetry);
//            Wobble.wobbleTelemetry(telemetry);
//            telemetry.addData("leftstick y value: ", gamepad1.left_stick_y);
//            telemetry.addData("leftstick x value: ", gamepad1.left_stick_x);
//            telemetry.addLine();
//            telemetry.addData("command heading: ", command);
//            telemetry.addLine();


            //telemetry.addData("grapfroot encoder", robot.grapfroot.getCurrentPosition());
//            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.update();
        }
    }
}