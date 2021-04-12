package org.firstinspires.ftc.teamcode;

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

    private static final long MIN_DELAY_MS = 500;

    private long mLastClickTime;

    private boolean wobbleFlag = false;
    private boolean resetFlag = false;

    private boolean intakeFlagFoward = false;
    private boolean intakeFlagReverse = false;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private double minBlue = Double.MAX_VALUE;
    private double maxBlue = Double.MIN_VALUE;

    private double minWhite = Double.MAX_VALUE;
    private double maxWhite = Double.MIN_VALUE;

    @Override
    public void runOpMode() throws InterruptedException {

        DriveTrain.initDriveTrain(hardwareMap);
        Wobble.initWobble(hardwareMap);
        Shooter.initShooter(hardwareMap);
        Intake.initIntake(hardwareMap);

        Intake.intakeRunMode("RUN_WITHOUT_ENCODER");

        DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");
        DriveTrain.setRunMode("RUN_WITHOUT_ENCODER");

        DriveTrain.composeTelemetry(telemetry);
        waitForStart();
        Wobble.drop();
        sleep(200);
        Wobble.grab();

        double P = 40;
        double I = 2;
        double D = 2;

        while (opModeIsActive()) {

            // 2m Distance sensor stuff
            Constants.Distance1.add(DriveTrain.frontDistanceSensor.getDistance(DistanceUnit.CM));

            DriveTrain.floorColorSensor.enableLed(true);

            //Mecanum Drive
            if(gamepad1.right_trigger > 0.4){
                DriveTrain.setRunMode("RUN_WITHOUT_ENCODER");
                DriveTrain.autoAlign();
            }
            else if(gamepad1.left_trigger > .2){
                DriveTrain.setRunMode("RUN_WITHOUT_ENCODER");
                DriveTrain.cartesianDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, (gamepad1.right_stick_x / 3));
            }
            else {
                DriveTrain.setRunMode("RUN_WITHOUT_ENCODER");
                DriveTrain.cartesianDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            if(gamepad1.dpad_up){
               DriveTrain.resetGyro();
            }

            if (gamepad1.x && !intakeFlagFoward) {
                intakeFlagFoward = true;
                Intake.intakeChangeState("FORWARD");
            }
            else if (gamepad1.b && !intakeFlagReverse){
                intakeFlagReverse = true;
                Intake.intakeChangeState("REVERSE");
            }

            if(intakeFlagFoward && !gamepad1.x){
                intakeFlagFoward = false;
            } else if(intakeFlagReverse && !gamepad1.b){
                intakeFlagReverse = false;
            }

            Intake.intakeUpdatePosition();

//            if (gamepad1.x) {
//                intake = !intake;
////                onClick(intake);
//            }
//
//            if (intake) {
//                Intake.intake();
//            } else {
//                Intake.stop();
//            }
//
//            if (gamepad1.b) {
//                Intake.setBackwards();
//            }

            if (gamepad1.a) {
                Shooter.setPosition("WHITE_LINE");
                Shooter.shoot(Shooter.SHOOTER_POWER);
            }
            else if (Shooter.shooter.getPower() == 0) {
                Shooter.setPosition("INDEX");
            }
            else if (gamepad1.y) {
                Shooter.shoot(0);
            }

//            if(gamepad2.a){
//                Shooter.shoot(Shooter.POWER_SHOT_POWER);
//            }

            //Intake Heights
//            if (gamepad1.dpad_up) {
//                Shooter.setPosition("WHITE_LINE");
//            }
            //In front of rings
            if (gamepad1.left_bumper) {
                Shooter.setPosition("RINGS");
                Intake.releaseAllRings();
            }else if (gamepad1.right_bumper) {
                Shooter.setPosition("WHITE_LINE");
                Intake.releaseAll();
            } else {
                Intake.defaultPos();
            }

            if(gamepad1.dpad_right){
                Shooter.setPosition("POWER_SHOT");
                sleep(200);
                Intake.shootOneNoClear();
            }

//            if(gamepad1.dpad_left){
//                Shooter.setPosition("POWER_SHOT");
//            }

//            if(gamepad2.a){
//                P = P + 2;
//            }
//            if(gamepad2.y){
//                P = P - 2;
//            }

            //Lifter
            if (gamepad2.right_bumper && !wobbleFlag) {
                wobbleFlag = true;
                Wobble.wobbleChangeState();
            }
            else if(!gamepad2.right_bumper && wobbleFlag){
                wobbleFlag = false;
            }

            if (gamepad2.x) {
                Wobble.close();
            }
            if (gamepad2.b) {
                Wobble.open();
            }

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

            if(gamepad2.y){
                Intake.spit();
            }


            if (gamepad1.dpad_down) {
                Intake.shootOneNoClear();
            } else {
                Intake.defaultPos();
            }

            Shooter.updateShooterConstants(P, I, D,0);

            DriveTrain.driveTelemetry(telemetry);
            Intake.intakeTelemetry(telemetry);
            telemetry.addData("New P: ", P);
            telemetry.addData("New I: ", I);
            Shooter.shooterTelemetry(telemetry);
            Wobble.wobbleTelemetry(telemetry);

            if(DriveTrain.floorColorSensor.blue() > maxBlue){
                maxBlue = DriveTrain.floorColorSensor.blue();
            }

            if (DriveTrain.floorColorSensor.blue() < minBlue){
                minBlue = DriveTrain.floorColorSensor.blue();
            }

            if(DriveTrain.floorColorSensor.alpha() > maxWhite){
                maxWhite = DriveTrain.floorColorSensor.alpha();
            }

            if (DriveTrain.floorColorSensor.alpha() < minWhite){
                minWhite = DriveTrain.floorColorSensor.alpha();
            }

            telemetry.addData("Max Blue: ", maxBlue);
            telemetry.addData("Min Blue: ", minBlue);
            telemetry.addData("Max White: ", maxWhite);
            telemetry.addData("Min White: ", minWhite);

//            telemetry.addData("wobble:", Wobble.wobbleMotor.getCurrentPosition());
//            telemetry.addData("wobble mode:", Wobble.wobbleMotor.getMode());
//            telemetry.addData("leftstick y value: ", gamepad1.left_stick_y);
//            telemetry.addData("leftstick x value: ", gamepad1.left_stick_x);
//            telemetry.addLine();
//            telemetry.addData("command heading: ", command);
//            telemetry.addLine();

            //telemetry.addData("grapfroot encoder", robot.grapfroot.getCurrentPosition());
//            telemetry.addData("Runtime", "%.03f", getRuntime());
            dashboardTelemetry.addData("Shooter Velocity", Shooter.getShooterSpeed());
            dashboardTelemetry.update();
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
};