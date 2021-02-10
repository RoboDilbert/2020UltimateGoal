package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.StarterTeleop;
import org.firstinspires.ftc.teamcode.Util.HardwarePresets;

public class Shooter{

    private DcMotorEx shooter; //Control hub, port 0
    public Servo vibrator;
    public Servo angleAdjust; //Control hub, port

    private static double NEW_P;//18.6
    private static double NEW_I;
    private static double NEW_D;
    private static double NEW_F;
    private static PIDFCoefficients pidOrig;
    private static PIDFCoefficients pidModified;

    //Constructor
    public Shooter(double P, double I, double D, double F, HardwareMap hardwareMap){

//        PIDFCoefficients pidOrig = robot.shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        NEW_P = P;
        NEW_I = I;
        NEW_D = D;
        NEW_F = F;

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Shooter(){

    }

    public void initShooter(HardwareMap hardwareMap){
        vibrator = hardwareMap.servo.get("vibrator");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        angleAdjust = hardwareMap.servo.get("angleAdjust");
    }
    //Methods
    public void shoot(double power){
        shooter.setPower(power);
    }
    public double getShooterSpeed(){
        return shooter.getVelocity();
    }
    public void angleAdjustUp(String position){
        if(position.equals("INDEX")){
            angleAdjust.setPosition(0.7);
        }
        else if(position.equals("WHITE_LINE")){
            angleAdjust.setPosition(0.43);
        }
        else if(position.equals("RINGS")){
            angleAdjust.setPosition(0.5);
        }
    }




    public double getNewP(){
        return pidModified.p;
    }
    public double getNewI(){
        return pidModified.i;
    }
    public double getNewD(){
        return pidModified.d;
    }
    public double getOldP(){
        return pidOrig.p;
    }
    public double getOldI(){
        return pidOrig.i;
    }
    public double getOldD(){
        return pidOrig.d;
    }


}
