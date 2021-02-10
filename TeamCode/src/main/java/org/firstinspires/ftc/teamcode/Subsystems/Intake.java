package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.StarterTeleop;
import org.firstinspires.ftc.teamcode.Util.HardwarePresets;
import java.sql.ResultSet;
import java.util.ArrayList;

public class Intake extends Shooter {
    //Instance Fields
    public ArrayList rings = new ArrayList(4);
    public boolean ringCountFlag = false;
    public final double ringDistance = 13;

    public DistanceSensor indexSensor;

    public Servo vibrator;
    public DcMotor frontIntakeMotor; //Control hub, port 1
    public DcMotor rearIntakeMotor; //Control hub, port 2

    //Constructor
    public Intake() {

    }
    //Methods

    public void initIntake(HardwareMap HwMap){
        frontIntakeMotor = HwMap.dcMotor.get("frontIntakeMotor");
        rearIntakeMotor = HwMap.dcMotor.get("rearIntakeMotor");
        indexSensor = HwMap.get(DistanceSensor.class, "indexSensor");
        vibrator = HwMap.servo.get("vibrator");

    }

    //Release 1
    public void releaseOne(){
        if(rings.lastIndexOf(true) >= 0) {
            vibrator.setPosition(0.65);
            sleep(150);
            vibrator.setPosition(0.45);
            sleep(75);
            rings.remove(rings.lastIndexOf(true));
        }
    }
    //Release All
    public void releaseAll(){
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        rings.clear();
    }
    public void shootAllNoClear(){
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
        vibrator.setPosition(0.65);
        sleep(100);
        vibrator.setPosition(0.45);
        sleep(100);
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
    public void spit(){
        mainShooter.shoot(.3);
        sleep(200);
        robot.vibrator.setPosition(0.65);
        sleep(75);
        robot.vibrator.setPosition(0.45);
        sleep(150);
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
        public void index () {
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
    public void intake(){
        if (robot.indexSensor.getDistance(DistanceUnit.CM) < ringDistance) {
            index();
        }
        else if (ringCountFlag) {
            rings.add(true);
            ringCountFlag = false;
        }
            robot.frontIntakeMotor.setPower(0.85);
            robot.rearIntakeMotor.setPower(0.85);

            if(isFull()){
                mainShooter.shoot(0);
            }
    }

    public void intakeTwo(){
        robot.frontIntakeMotor.setPower(.85);
        robot.rearIntakeMotor.setPower(.85);
    }
}