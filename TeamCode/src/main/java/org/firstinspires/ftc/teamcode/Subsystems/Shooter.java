package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.StarterTeleop;
import org.firstinspires.ftc.teamcode.Util.HardwarePresets;

public class Shooter extends StarterTeleop {

    public static double NEW_P;//18.6
    public static double NEW_I;
    public static double NEW_D;
    public static double NEW_F;
    public static PIDFCoefficients pidOrig;
    public static PIDFCoefficients pidModified;

    //Constructor
    public Shooter(double P, double I, double D, double F){

        PIDFCoefficients pidOrig = robot.shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        NEW_P = P;
        NEW_I = I;
        NEW_D = D;
        NEW_F = F;

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        robot.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = robot.shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Shooter(){

    }

    //Methods
    public void getShooterSpeed(){

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
    public void shoot(double power){
        robot.shooter.setPower(power);
   }
}
