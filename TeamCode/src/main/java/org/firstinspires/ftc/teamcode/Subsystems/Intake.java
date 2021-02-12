package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.Constants;
import java.util.ArrayList;

public class Intake {
    //Instance Fields
    public ArrayList rings = new ArrayList(4);
    public boolean ringCountFlag = false;
    public final double ringDistance = 13;

    public static DistanceSensor indexSensor;

    public static DcMotor frontIntakeMotor; //Control hub, port 1
    public static DcMotor rearIntakeMotor; //Control hub, port 2

    public static Servo vibrator; //Control hub, port  0

    //Constructor
    public Intake() {}

    //Methods
    public static void initIntake(HardwareMap hwm){
        Constants.HwMap = hwm;
        frontIntakeMotor = Constants.HwMap.dcMotor.get("frontIntakeMotor");
        rearIntakeMotor = Constants.HwMap.dcMotor.get("rearIntakeMotor");

        indexSensor = Constants.HwMap.get(DistanceSensor.class, "indexSensor");

        vibrator = Constants.HwMap.servo.get("vibrator");
    }

    //Release 1
    public void releaseOne() throws InterruptedException {
        if(rings.lastIndexOf(true) >= 0) {
            vibrator.setPosition(0.65);
            Thread.sleep(150);
            vibrator.setPosition(0.45);
            Thread.sleep(75);
            rings.remove(rings.lastIndexOf(true));
        }
    }
    //Release All
    public void releaseAll() throws InterruptedException {
        for(int i = 0; i < 9; i++) {
            vibrator.setPosition(0.65);
            Thread.sleep(100);
            vibrator.setPosition(0.45);
            Thread.sleep(100);
        }
        rings.clear();
    }
    public void shootAllNoClear()  throws InterruptedException {
        for(int i = 0; i < 6; i++) {
            vibrator.setPosition(0.65);
            Thread.sleep(100);
            vibrator.setPosition(0.45);
            Thread.sleep(100);
        }
    }


    //Spin backward (button hold)
    public void backwards(){
        frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.frontIntakeMotor.setTargetPosition(10000);
//                robot.rearIntakeMotor.setTargetPosition(10000);
        frontIntakeMotor.setPower(0.3);
        rearIntakeMotor.setPower(0.3);
//        robot.frontIntakeMotor.setPower(-0.85);
//        robot.rearIntakeMotor.setPower(-0.85);
    }

    //Spit one ring
    public void spit() throws InterruptedException {
        Shooter.mainShooter.shoot(.3);
        Thread.sleep(200);
        vibrator.setPosition(0.65);
        Thread.sleep(75);
        vibrator.setPosition(0.45);
        Thread.sleep(150);
        rings.remove(rings.lastIndexOf(true));
    }

    //Check to see if there are 3 rings
    public boolean isFull() {
        if (rings.lastIndexOf(true) == 2) {
            return true;
        } else {
            return false;
        }
    }

        //Index
        public void index ()  throws InterruptedException {
            if (rings.lastIndexOf(true) < 2) {
                ringCountFlag = true;
//                robot.frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.rearIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.frontIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.rearIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.frontIntakeMotor.setTargetPosition(10000);
//                robot.rearIntakeMotor.setTargetPosition(10000);
//                robot.frontIntakeMotor.setPower(0.3);
//                robot.rearIntakeMotor.setPower(0.3);
//                robot.frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.rearIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                ringCountFlag = true;
                spit();
//                robot.frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.rearIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.frontIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.rearIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.frontIntakeMotor.setTargetPosition(10000);
//                robot.rearIntakeMotor.setTargetPosition(10000);
//                robot.frontIntakeMotor.setPower(0.3);
//                robot.rearIntakeMotor.setPower(0.3);
//                robot.frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.rearIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

        }
    public void intake() throws InterruptedException {
        if (indexSensor.getDistance(DistanceUnit.CM) < ringDistance) {
            index();
        }
        else if (ringCountFlag) {
            rings.add(true);
            ringCountFlag = false;
        }
            frontIntakeMotor.setPower(0.85);
            rearIntakeMotor.setPower(0.85);

            if(isFull()){
                Shooter.mainShooter.shoot(0);
            }
    }

    public void intakeTwo(){
        frontIntakeMotor.setPower(.85);
        rearIntakeMotor.setPower(.85);
    }

    public static void intakeTelemetry(Telemetry telemetry){
//        telemetry.addLine()
//                .addData("Red", "%.3f", (double) indexColorSensor.red())
//                .addData("Blue", "%.3f", (double) indexColorSensor.blue())
//                .addData("Alpha", "%.3f", (double) indexColorSensor.alpha());
        telemetry.addData("Vibrator:", vibrator.getPosition());
        telemetry.addLine();
        telemetry.addData("indexSensor", String.format("%.3f cm", Intake.indexSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("rings: ", Constants.mainIntake.rings);
        telemetry.addData("Ring Flag: ", Constants.mainIntake.ringCountFlag);
        telemetry.addData("Intake Array Size:", Constants.mainIntake.rings.lastIndexOf(true));
        telemetry.addLine();
    }
}