package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.Constants;

public class Shooter {

    public static DcMotorEx shooter; //Control hub, port 0
    public static Shooter mainShooter;
    private static Servo angleAdjust; //Control hub, port 2

    private static double NEW_P;//18.6
    private static double NEW_I;
    private static double NEW_D;
    private static double NEW_F;

    public static final double SHOOTER_POWER = 0.525; //previous .575 w/o flywheel weight
    public static final double POWER_SHOT_POWER = .45;

    //private static PIDFCoefficients pidOrig;
    //private static PIDFCoefficients pidModified;

    //Constructor
    public void Shooter(){

        //PIDFCoefficients pidOrig = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        NEW_P = 50;
        NEW_I = 1;
        NEW_D = 2;
        NEW_F = 0;

        updateShooterConstants(NEW_P, NEW_I, NEW_D, NEW_F);
    }

    public static void initShooter(HardwareMap hwm){
        Constants.HwMap = hwm;
        shooter = Constants.HwMap.get(DcMotorEx.class, "shooter");
        angleAdjust = Constants.HwMap.servo.get("angleAdjust");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        Shooter.shooterRunMode("STOP_AND_RESET_ENCODER");
        Shooter.shooterRunMode("RUN_USING_ENCODER");

        mainShooter = new Shooter();

        mainShooter.setPosition("RINGS");
    }
    public static void updateShooterConstants(double p, double i, double d, double f){
        NEW_P = p;
        NEW_I = i;
        NEW_D = d;
        NEW_F = f;

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        //PIDFCoefficients pidModified = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void shooterTelemetry(Telemetry telemetry){
//        telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
//                mainShooter.getOldP(), mainShooter.getOldI(), mainShooter.getOldD());
//        telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
//                mainShooter.getNewP(), mainShooter.getNewI(), mainShooter.getNewD());
//        telemetry.addData("Real P: ", NEW_P);
//        telemetry.addData("Real I: ", NEW_I);
//        telemetry.addData("Shooty velo", shooter.getVelocity());
        telemetry.addData("Shooter angle", angleAdjust.getPosition());
        telemetry.addLine();
    }

    //Methods
    public static double getShooterSpeed(){
        return shooter.getVelocity();
    }

    public static void setPosition(String position){
        if(position.equals("INDEX")){
            angleAdjust.setPosition(0.7);
        }
        else if(position.equals("WHITE_LINE")){
            angleAdjust.setPosition(0.44); //White line = .49
        }
        else if(position.equals("RINGS")){
            angleAdjust.setPosition(0.65); //.5
        }
//        else if (position.equals("SHOOTING")){
//            angleAdjust.setPosition(0.49); //white line = .53
//        }
        else if(position.equals("RINGS_ADJUST")) {
            angleAdjust.setPosition(.7); //.5
        }
        else if(position.equals("POWER_SHOT")){
            angleAdjust.setPosition(.35); //First: .4
        }
    }

//    public double getNewP(){
//        return pidModified.p;
//    }
//    public double getNewI(){
//        return pidModified.i;
//    }
//    public double getNewD(){
//        return pidModified.d;
//    }
//    public double getOldP(){return pidOrig.p;}
//    public double getOldI(){
//        return pidOrig.i;
//    }
//    public double getOldD(){
//        return pidOrig.d;
//    }

    public static void shoot(double power){
      //  shooter.setVelocity(0, AngleUnit.RADIANS);
        shooter.setPower(power);
   }

    public static void shooterRunMode(String mode){
        if(mode.equals("RUN_USING_ENCODER")){
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if(mode.equals("RUN_WITHOUT_ENCODER")){
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if(mode.equals("STOP_AND_RESET_ENCODER")){
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }
}
