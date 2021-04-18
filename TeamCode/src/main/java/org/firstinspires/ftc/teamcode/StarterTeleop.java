package org.firstinspires.ftc.teamcode;

import android.drm.DrmInfoEvent;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Wobble;
import org.firstinspires.ftc.teamcode.UtilOG.*;

@TeleOp(name= "TeleOop", group= "TeleOp")
//@Disabled
public class StarterTeleop extends LinearOpMode {

    public boolean intake = false;

    private boolean wobbleFlag = false;
    private boolean resetFlag = false;

    private boolean intakeFlagFoward = false;
    private boolean intakeFlagReverse = false;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {

        //Init motors and servos
        DriveTrain.initDriveTrain(hardwareMap);
        Wobble.initWobble(hardwareMap);
        Shooter.initShooter(hardwareMap);
        Intake.initIntake(hardwareMap);

        Intake.intakeRunMode("RUN_WITHOUT_ENCODER");

        //Set drive motors to run without encoder
        DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");
        DriveTrain.setRunMode("RUN_WITHOUT_ENCODER");

        //Drivetrain telemetr
        DriveTrain.composeTelemetry(telemetry);

        waitForStart();

        //Reset wobble claw so there isnt a wobble stuck in it
        Wobble.drop();
        sleep(200);
        Wobble.grab();

        //Shooter PID constants
        double P = 40;
        double I = 2;
        double D = 2;

        while (opModeIsActive()) {

            //2m Distance sensor stuff
            Constants.Distance1.add(DriveTrain.frontDistanceSensor.getDistance(DistanceUnit.CM));

            DriveTrain.floorColorSensor.enableLed(true);

            //Auto align
            if(gamepad1.right_trigger > 0.4){
                DriveTrain.setRunMode("RUN_WITHOUT_ENCODER");
                DriveTrain.autoAlign();
            }
            //Slow turning
            else if(gamepad1.left_trigger > .2){
                DriveTrain.setRunMode("RUN_WITHOUT_ENCODER");
                DriveTrain.cartesianDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, (gamepad1.right_stick_x / 3));
            }
            //Field centric drive
            else {
                DriveTrain.setRunMode("RUN_WITHOUT_ENCODER");
                DriveTrain.cartesianDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            //Reset gyro for field centric
            if(gamepad1.dpad_up){
               DriveTrain.resetGyro();
            }

            //Intake forward
            if (gamepad1.x && !intakeFlagFoward) {
                intakeFlagFoward = true;
                Intake.intakeChangeState("FORWARD");
            }
            //Intake backwards
            else if (gamepad1.b && !intakeFlagReverse){
                intakeFlagReverse = true;
                Intake.intakeChangeState("REVERSE");
            }

            //Reset intakes flags
            if(intakeFlagFoward && !gamepad1.x){
                intakeFlagFoward = false;
            } else if(intakeFlagReverse && !gamepad1.b){
                intakeFlagReverse = false;
            }

            Intake.intakeUpdatePosition();

            //Turn shooter on
            if (gamepad1.a) {
                Shooter.setPosition("WHITE_LINE");
                Shooter.shoot(Shooter.SHOOTER_POWER);
            }
            //Set shooter to the furthest up position if it's not running so rings fall in the hopper easier
            else if (Shooter.shooter.getPower() == 0) {
                Shooter.setPosition("INDEX");
            }
            //Turn off shooter
            else if (gamepad1.y) {
                Shooter.shoot(0);
            }

            //Shoot in front of rings
            if (gamepad1.left_bumper) {
                Shooter.setPosition("RINGS");
                Intake.releaseAllRings();
            }else if (gamepad1.right_bumper) {
                Shooter.setPosition("WHITE_LINE");
                Intake.releaseAll();
            } else {
                Intake.defaultPos();
            }

            //Shoot power shot
            if(gamepad1.dpad_right){
                Shooter.setPosition("POWER_SHOT");
                sleep(200);
                Intake.shootOneNoClear();
            }

            //Lifter
            if (gamepad2.right_bumper && !wobbleFlag) {
                wobbleFlag = true;
                Wobble.wobbleChangeState();
            }
            else if(!gamepad2.right_bumper && wobbleFlag){
                wobbleFlag = false;
            }

            //Wobble claw open and close
            if (gamepad2.x) {
                Wobble.close();
            }
            if (gamepad2.b) {
                Wobble.open();
            }

            //Reset the wobble arm in case it is initialized wrong
            if(gamepad2.dpad_down){
                Wobble.wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Wobble.wobbleMotor.setPower(0.2);
                resetFlag = true;
            }
            else if (resetFlag){
                Wobble.wobbleMotor.setPower(0);
                Wobble.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Wobble.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                resetFlag = false;
            }
            else{
                Wobble.wobbleUpdatePosition();
            }

            //Spit one ring if we have 4 rings
            if(gamepad2.y){
                Intake.spit();
            }

            //Shoot one ring
            if (gamepad1.dpad_down) {
                Intake.shootOneNoClear();
            }

            //Set shooter PID constants
            Shooter.updateShooterConstants(P, I, D,0);

            //Call telemetry for each sub system
            DriveTrain.driveTelemetry(telemetry);
            Intake.intakeTelemetry(telemetry);
            Shooter.shooterTelemetry(telemetry);
            Wobble.wobbleTelemetry(telemetry);

            //Dashboard telemetry
            dashboardTelemetry.addData("Shooter Velocity", Shooter.getShooterSpeed());
            dashboardTelemetry.update();
            telemetry.update();
        }
    }
}