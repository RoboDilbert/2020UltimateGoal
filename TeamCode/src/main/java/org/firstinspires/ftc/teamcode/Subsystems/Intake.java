package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Util.HardwarePresets;
import java.sql.ResultSet;
import java.util.ArrayList;

public class Intake extends HardwarePresets {
    //Instance Fields
    public ArrayList rings = new ArrayList(4);
    public boolean ringCountFlag = false;
    public final double orangeValue = 43;

    //Constructor
    public Intake() {

    }
    //Methods

    //Release 1
    public void releaseOne(){
        vibrator.setPosition(0.53);
        sleep(50);
        vibrator.setPosition(0.42);
        sleep(25);
        rings.remove(rings.lastIndexOf(true));
    }
    //Release All
    public void releaseAll(){
        vibrator.setPosition(0.53);
        sleep(50);
        vibrator.setPosition(0.42);
        sleep(25);
        vibrator.setPosition(0.53);
        sleep(50);
        vibrator.setPosition(0.42);
        sleep(25);
        vibrator.setPosition(0.53);
        sleep(50);
        vibrator.setPosition(0.42);
        sleep(25);
        vibrator.setPosition(0.53);
        sleep(50);
        vibrator.setPosition(0.42);
        sleep(25);
        vibrator.setPosition(0.53);
        sleep(50);
        vibrator.setPosition(0.42);
        sleep(25);
        vibrator.setPosition(0.53);
        sleep(50);
        vibrator.setPosition(0.42);
        sleep(25);
        rings.clear();
    }

    //Spin backward (button hold)
    public void backwards(){
        frontIntakeMotor.setPower(-0.85);
        rearIntakeMotor.setPower(-0.85);
    }

    //Spit one ring
    public void spit(){
        frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontIntakeMotor.setTargetPosition(200);
        rearIntakeMotor.setTargetPosition(200);
        frontIntakeMotor.setPower(-.85);
        rearIntakeMotor.setPower(-.85);
        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontIntakeMotor.setTargetPosition(50);
                rearIntakeMotor.setTargetPosition(50);
                frontIntakeMotor.setPower(.85);
                rearIntakeMotor.setPower(.85);
                frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rearIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            else {
                spit();
            }

    }
//    public void intake(){
//        if (orngi.red() > orangeValue) {
//            index();
//        }
//        else if (ringCountFlag) {
//            rings.add(true);
//            ringCountFlag = false;
//        }
//        if(isFull()) {
//            backwards();
//        }
//        else{
//            frontIntakeMotor.setPower(0.85);
//            rearIntakeMotor.setPower(0.85);
//        }
//
//    }
    public void intakeTwo(){
        frontIntakeMotor.setPower(.85);
        rearIntakeMotor.setPower(.85);
    }
}