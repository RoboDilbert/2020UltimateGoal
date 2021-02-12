package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Constants;

public class Shooter {

    public static DcMotorEx shooter; //Control hub, port 0
    public static Shooter mainShooter;
    public static Servo angleAdjust; //Control hub, port

    private static double NEW_P;//18.6
    private static double NEW_I;
    private static double NEW_D;
    private static double NEW_F;
    private static PIDFCoefficients pidOrig;
    private static PIDFCoefficients pidModified;

    //Constructor
    public Shooter(double P, double I, double D, double F){

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

    public static void initShooter(HardwareMap hwm){
        Constants.HwMap = hwm;
        shooter = Constants.HwMap.get(DcMotorEx.class, "shooter");
        angleAdjust = Constants.HwMap.servo.get("angleAdjust");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        mainShooter = new Shooter(NEW_P, NEW_I, NEW_D, NEW_F);
    }

    public static void shooterTelemetry(Telemetry telemetry){
        telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                mainShooter.getOldP(), mainShooter.getOldI(), mainShooter.getOldD());
        telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                mainShooter.getNewP(), mainShooter.getNewI(), mainShooter.getNewD());
        telemetry.addLine();
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
        shooter.setPower(power);
   }
}
