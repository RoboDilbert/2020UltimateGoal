package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.UtilOG.Constants;

public class Shooter {

    //Declare motors and servos
    public static DcMotorEx shooter; //Control hub, port 0
    public static Shooter mainShooter;
    private static Servo angleAdjust; //Control hub, port 2
    private static Servo blocker;

    //PID constants for the shooter
    private static double NEW_P;//18.6
    private static double NEW_I;
    private static double NEW_D;
    private static double NEW_F;

    //Power constants
    public static final double SHOOTER_POWER = 0.525; //previous .575 w/o flywheel weight

    //Constructor
    public void Shooter(){

        NEW_P = 50;
        NEW_I = 1;
        NEW_D = 2;
        NEW_F = 0;

        updateShooterConstants(NEW_P, NEW_I, NEW_D, NEW_F);
    }

    public static void initShooter(HardwareMap hwm){
        //Declare motors in the hardware map
        Constants.HwMap = hwm;
        shooter = Constants.HwMap.get(DcMotorEx.class, "shooter");
        angleAdjust = Constants.HwMap.servo.get("angleAdjust");
        blocker = Constants.HwMap.servo.get("blocker");

        //Set run modes and directions for the shooter
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        Shooter.shooterRunMode("STOP_AND_RESET_ENCODER");
        Shooter.shooterRunMode("RUN_USING_ENCODER");

        mainShooter = new Shooter();

        mainShooter.setPosition("RINGS");
    }

    //Update the PID constants in case we want to change them
    public static void updateShooterConstants(double p, double i, double d, double f){
        NEW_P = p;
        NEW_I = i;
        NEW_D = d;
        NEW_F = f;

        //Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
    }

    //Shooter telemetry (this changes often)
    public static void shooterTelemetry(Telemetry telemetry){
        telemetry.addData("Shooter angle", angleAdjust.getPosition());
        telemetry.addData("Blocker Pos:", blocker.getPosition());
        telemetry.addLine();
    }

    //Return the speed of the shooter
    public static double getShooterSpeed(){
        return shooter.getVelocity();
    }

    //Set the height of the shooter using the angle adjust servo
    public static void setPosition(String position){
        if(position.equals("INDEX")){
            angleAdjust.setPosition(0.7);
        }
        else if(position.equals("WHITE_LINE")){
            angleAdjust.setPosition(.45); //.44
        }
        else if(position.equals("RINGS")){
            angleAdjust.setPosition(0.65); //.5
        }
        else if(position.equals("RINGS_ADJUST")) {
            angleAdjust.setPosition(.7); //.5
        }
        else if(position.equals("POWER_SHOT")){
            angleAdjust.setPosition(.3); //First: .35
        }
    }

    //Set the shooter power using a parameter
    public static void shoot(double power){
        shooter.setPower(power);
   }

   //Set the run mode of the shooter motor
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

    public static void block(){
        blocker.setPosition(.2);
    }

    public static void unblock(){
        blocker.setPosition(.42);
    }
}
