package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Constants;

import java.io.CharArrayWriter;

public class Wobble {

    public static DcMotor wobbleMotor;

    public static Servo wobble1;
    public static Servo wobble2;

    public static Servo grabber; //Control hub, port 1

    //Constuctor
    public void Wobble(){}

    public static void initWobble(HardwareMap hwm){
        Constants.HwMap = hwm;
        wobbleMotor = Constants.HwMap.dcMotor.get("wobbleMotor");

        wobble1 = Constants.HwMap.servo.get("wobble1");
        wobble2 = Constants.HwMap.servo.get("wobble2");

        grabber = Constants.HwMap.servo.get("grabber");

        wobbleMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void wobbleTelemetry(Telemetry telemetry){
        telemetry.addData("Lifter Pos: ", Wobble.wobbleMotor.getCurrentPosition());
        telemetry.addData("Lifter TP: ", Wobble.wobbleMotor.getTargetPosition());
        telemetry.addData("Lifter Pow: ", Wobble.wobbleMotor.getPower());
        telemetry.addLine();
        telemetry.addData("wobble1:", wobble1.getPosition());
        telemetry.addData("wobble2:", wobble2.getPosition());
        telemetry.addLine();
    }
}
