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

import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@TeleOp(name= "TeleOop", group= "TeleOp")
//@Disabled
public class StarterTeleop extends LinearOpMode {

    public float gyroVariation = 0;

    public boolean intake = false;

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
            DriveTrain.cartesianDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

//            if(gamepad1.x){
//                gyroVariation = angles.firstAngle;
//            }

            if (gamepad1.x) {
                intake = !intake;
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
                Shooter.mainShooter.setPosition("WHITE_LINE");
                Shooter.mainShooter.shoot(0.58);
            }
            if (Shooter.shooter.getPower() == 0) {
                Shooter.mainShooter.setPosition("INDEX");
            }

            //White Line
            if (gamepad2.dpad_up) {
                Shooter.mainShooter.setPosition("WHITE_LINE");
            }
            //In front of rings
            if (gamepad2.dpad_left) {
                Shooter.mainShooter.setPosition("RINGS");
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
            if (gamepad2.right_bumper) {
                Wobble.lift(-500, gamepad1.right_bumper);
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
            Intake.intakeTelemetry(telemetry);
            Shooter.shooterTelemetry(telemetry);
            Wobble.wobbleTelemetry(telemetry);
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