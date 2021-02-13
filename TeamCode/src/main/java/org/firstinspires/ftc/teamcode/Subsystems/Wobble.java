package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import org.firstinspires.ftc.teamcode.Util.Constants;

import java.io.CharArrayWriter;

public class Wobble {

    private static DcMotor wobbleMotor;

    private static Servo wobble1;
    private static Servo wobble2;

    private static Servo grabber; //Control hub, port 1

    //Constuctor
    public Wobble(){}

    public static void initWobble(HardwareMap hwm){
        Constants.HwMap = hwm;
        wobbleMotor = Constants.HwMap.dcMotor.get("wobbleMotor");

        wobble1 = Constants.HwMap.servo.get("wobble1");
        wobble2 = Constants.HwMap.servo.get("wobble2");

        grabber = Constants.HwMap.servo.get("grabber");

        wobbleMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public static void open(){
        wobble2.setPosition(.1);
    }

    public static void close(){
        wobble2.setPosition(.5);
    }

    public static void lift(int lifterTP, boolean right_bumper){
        Wobble.wobbleMotor.setTargetPosition(lifterTP);
        Wobble.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Wobble.wobbleMotor.setPower(-.3);
        if (Math.abs(lifterTP - Wobble.wobbleMotor.getCurrentPosition()) < (-lifterTP * .1)) {
            Wobble.wobbleMotor.setPower(-0.4);
        }
        else if(Math.abs(lifterTP - Wobble.wobbleMotor.getCurrentPosition()) < (-lifterTP * .1) && !right_bumper) {
            Wobble.wobbleMotor.setTargetPosition(-350);
            Wobble.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Wobble.wobbleMotor.setPower(.3);
        }
        if (Math.abs(Wobble.wobbleMotor.getCurrentPosition()) < (-lifterTP * .1)) {
            Wobble.wobbleMotor.setPower(0.4);
        }
    }


    public static void wobbleTelemetry(Telemetry telemetry){
        telemetry.addData("Lifter Pos: ", Wobble.wobbleMotor.getCurrentPosition());
        telemetry.addData("Lifter TP: ", Wobble.wobbleMotor.getTargetPosition());
        telemetry.addData("Lifter Pow: ", Wobble.wobbleMotor.getPower());
        telemetry.addLine();
        telemetry.addData("wobble1:", wobble1.getPosition());
        telemetry.addData("wobble2:", wobble2.getPosition());
        telemetry.addLine();
        telemetry.update();
    }
}
