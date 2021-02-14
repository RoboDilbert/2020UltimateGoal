package org.firstinspires.ftc.teamcode;

import android.view.View;

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

import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@TeleOp(name= "TeleOop", group= "TeleOp")
//@Disabled
public class StarterTeleop extends LinearOpMode {

    public float gyroVariation = 0;

    public boolean intake = false;

    private static final long MIN_DELAY_MS = 500;

    private long mLastClickTime;

    @Override
    public void runOpMode() throws InterruptedException {

        DriveTrain.initDriveTrain(hardwareMap);
        Wobble.initWobble(hardwareMap);
        Shooter.initShooter(hardwareMap);
        Intake.initIntake(hardwareMap);

        Intake.intakeRunMode("RUN_WITHOUT_ENCODER");

        DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");
        DriveTrain.setRunMode("RUN_USING_ENCODER");

        Shooter.shooterRunMode("STOP_AND_RESET_ENCODER");
        Shooter.shooterRunMode("RUN_USING_ENCODER");

        DriveTrain.composeTelemetry(telemetry);
        waitForStart();

        while (opModeIsActive()) {

            // 2m Distance sensor stuff
            Constants.Distance1.add(DriveTrain.driveDistanceSensor.getDistance(DistanceUnit.CM));

            DriveTrain.floorColorSensor.enableLed(true);

            //Mecanum Drive
            if(gamepad1.right_trigger > 0.4){
                DriveTrain.setRunMode("RUN_WITHOUT_ENCODER");
                DriveTrain.autoAlign();
            }
            else {
                DriveTrain.setRunMode("RUN_USING_ENCODER");
                DriveTrain.cartesianDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

//            if(gamepad1.x){
//                gyroVariation = angles.firstAngle;
//            }
            if (gamepad1.x) {
                intake = !intake;
//                onClick(intake);
            }

            if (intake) {
                Intake.intake();
            } else {
                Intake.stop();
            }

            if (gamepad1.b) {
                Intake.setBackwards();
            }

            if (gamepad1.a) {
                Shooter.setPosition("WHITE_LINE");
                Shooter.shoot(0.58);
            }
            if (Shooter.shooter.getPower() == 0) {
                Shooter.setPosition("INDEX");
            }

            //White Line
            if (gamepad2.dpad_up) {
                Shooter.setPosition("WHITE_LINE");
            }
            //In front of rings
//            if (gamepad2.dpad_left) {
//                Shooter.mainShooter.setPosition("RINGS");
//            }
            if(gamepad2.dpad_left){
                Wobble.grab();
            }
            if(gamepad2.dpad_right){
                Wobble.drop();
            }
//            if(gamepad2.a){
////                robot.grabber.setPosition(.5);
//            }
//
//            if(gamepad2.b) {
////                robot.grabber.setPosition(.25);
//            }

//            if (gamepad2.a) {
//                Wobble.wobble1.setPosition(.12);
//                //wobble bottom position
//            }
//            if (gamepad2.y) {
//                Wobble.wobble1.setPosition(.08);
//                //wobble top position

            if (gamepad2.x) {
                Wobble.close();
                //wobble grabber close position
            }
            if (gamepad2.b) {
                Wobble.open();
                //wobble grabber open position
            }
            if (gamepad1.y) {
                Shooter.shoot(0);
            }

            //Lifter
            if (gamepad1.right_bumper) {
                Wobble.lift(420, gamepad1.right_bumper);
            }
//            else if(!gamepad1.right_bumper){
//                Wobble.unlift(-100, gamepad1.right_bumper);
//            }

            if(gamepad2.right_bumper){
                Wobble.wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Wobble.wobbleMotor.setPower(0);
            }

            if (gamepad1.dpad_right) {
                Intake.releaseAll();
            } else {
                Intake.defaultPos();
            }

            if (gamepad1.dpad_left) {
                Intake.shootOneNoClear();
            } else {
                Intake.defaultPos();
            }

            DriveTrain.DriveTelemetry(telemetry);
//            Intake.intakeTelemetry(telemetry);
//            Shooter.shooterTelemetry(telemetry);
//            Wobble.wobbleTelemetry(telemetry);
//            Intake.intakeTelemetry(telemetry);
//            telemetry.addData("wobble:", Wobble.wobbleMotor.getCurrentPosition());
//            telemetry.addData("wobble mode:", Wobble.wobbleMotor.getMode());
//            telemetry.addData("leftstick y value: ", gamepad1.left_stick_y);
//            telemetry.addData("leftstick x value: ", gamepad1.left_stick_x);
//            telemetry.addLine();
//            telemetry.addData("command heading: ", command);
//            telemetry.addLine();

            telemetry.addData("Trigger Position", gamepad1.right_trigger);
            //telemetry.addData("grapfroot encoder", robot.grapfroot.getCurrentPosition());
//            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.update();
        }
    }
    public final void onClick(boolean v) {
        long lastClickTime = mLastClickTime;
        long now = System.currentTimeMillis();
        mLastClickTime = now;
        if (now - lastClickTime < MIN_DELAY_MS) {

        }
        else{
            v = !v;
        }
    }
}
