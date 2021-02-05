package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.StarterTeleop;
import org.firstinspires.ftc.teamcode.Util.HardwarePresets;
import java.sql.ResultSet;
import java.util.ArrayList;

public class Intake extends StarterTeleop {
    //Instance Fields
    public ArrayList rings = new ArrayList(4);
    public boolean ringCountFlag = false;
    public final double ringDistance = 13;

    //Constructor
    public Intake() {

    }
    //Methods

    //Release 1
    public void releaseOne(){
        robot.vibrator.setPosition(0.65);
        sleep(150);
        robot.vibrator.setPosition(0.45);
        sleep(75);
        rings.remove(rings.lastIndexOf(true));
    }
    //Release All
    public void releaseAll(){
        robot.vibrator.setPosition(0.65);
        sleep(150);
        robot.vibrator.setPosition(0.45);
        sleep(75);
        robot.vibrator.setPosition(0.65);
        sleep(150);
        robot.vibrator.setPosition(0.45);
        sleep(75);
        robot.vibrator.setPosition(0.65);
        sleep(150);
        robot.vibrator.setPosition(0.45);
        sleep(75);
        robot.vibrator.setPosition(0.65);
        sleep(150);
        robot.vibrator.setPosition(0.45);
        sleep(75);
        robot.vibrator.setPosition(0.65);
        sleep(150);
        robot.vibrator.setPosition(0.45);
        sleep(75);
        robot.vibrator.setPosition(0.65);
        sleep(150);
        robot.vibrator.setPosition(0.45);
        sleep(75);
        rings.clear();
    }

    //Spin backward (button hold)
    public void backwards(){
        robot.frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.frontIntakeMotor.setTargetPosition(10000);
//                robot.rearIntakeMotor.setTargetPosition(10000);
        robot.frontIntakeMotor.setPower(0.3);
        robot.rearIntakeMotor.setPower(0.3);
//        robot.frontIntakeMotor.setPower(-0.85);
//        robot.rearIntakeMotor.setPower(-0.85);
    }

    //Spit one ring
    public void spit(){
        robot.frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontIntakeMotor.setTargetPosition(2000);
        robot.rearIntakeMotor.setTargetPosition(2000);
        robot.frontIntakeMotor.setPower(-.85);
        robot.rearIntakeMotor.setPower(-.85);
        robot.frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Check to see if there are 3 rings
    public boolean isFull(){
        if(rings.lastIndexOf(true) == 2){
            return true;
        }
        else{
            return false;
        }
    }

    //Index
    public void index(){
            if (rings.lastIndexOf(true) < 2) {
                ringCountFlag = true;
                robot.frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rearIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rearIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontIntakeMotor.setTargetPosition(10000);
                robot.rearIntakeMotor.setTargetPosition(10000);
                robot.frontIntakeMotor.setPower(0.3);
                robot.rearIntakeMotor.setPower(0.3);
                robot.frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rearIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            else {
                spit();
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
        if(isFull()) {
            backwards();
        }
        else{
            robot.frontIntakeMotor.setPower(0.85);
            robot.rearIntakeMotor.setPower(0.85);
        }
    }

    public void intakeTwo(){
        robot.frontIntakeMotor.setPower(.85);
        robot.rearIntakeMotor.setPower(.85);
    }
}