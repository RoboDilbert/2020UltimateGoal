package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import org.firstinspires.ftc.teamcode.Util.Constants;

import java.io.CharArrayWriter;
import java.security.PrivateKey;

public class Wobble {

    public static DcMotor wobbleMotor;//Control hub, port 1

    private static Servo wobble1;//Control hub, port 3
    private static Servo wobble2;//Control hub, port 4

    private static Servo grabber; //Control hub, port 1

    private static final double GRABBER_OPEN = 0.7;
    private static final double GRABBER_CLOSED = 1.0;

    private static final double WOBBLE_ONE_OPEN = 0.16;
    private static final double WOBBLE_ONE_MINI = 0.28;
    private static final double WOBBLE_ONE_CLOSED = 0.44; //.38
    private static final double WOBBLE_TWO_OPEN = 0.72;
    private static final double WOBBLE_TWO_MINI = 0.61;
    private static final double WOBBLE_TWO_CLOSED = 0.38; //.38

    private static final double WOBBLE_MOTOR_ERROR = 0.1;

    public static final int WOBBLE_UP_TICKS = -240;
    public static final int WOBBLE_DOWN_TICKS = -1130;
    public static final int WOBBLE_INIT_TICKS = 0;
    public static final int WOBBLE_DROP_TICKS = -500;
    private static WOBBLE_STATE currentState = WOBBLE_STATE.INIT;

    private enum WOBBLE_STATE{
        UP,
        DOWN,
        INIT,
        DROP;
    }

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
        grabber.setPosition(1);
    }

    public static void open(){
        wobble1.setPosition(WOBBLE_ONE_OPEN);
        wobble2.setPosition(WOBBLE_TWO_OPEN);
    }

    public static void openSmall(){
        wobble1.setPosition(WOBBLE_ONE_MINI);
        //wobble2.setPosition(WOBBLE_TWO_MINI);
    }
    public static void close(){
        wobble1.setPosition(WOBBLE_ONE_CLOSED);
        wobble2.setPosition(WOBBLE_TWO_CLOSED);
    }

    public static void grab(){
        grabber.setPosition(GRABBER_CLOSED);
    }
    public static void drop(){
        grabber.setPosition(GRABBER_OPEN);
    }

    public static void lowerArm(int lifterTP){
        wobbleMotor.setTargetPosition(lifterTP);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(Math.abs(lifterTP - Wobble.wobbleMotor.getCurrentPosition()) > (-lifterTP * WOBBLE_MOTOR_ERROR)) {
            wobbleMotor.setPower(.4);
        }
        if (Math.abs(lifterTP - Wobble.wobbleMotor.getCurrentPosition()) < (-lifterTP * WOBBLE_MOTOR_ERROR)) {
            Wobble.wobbleMotor.setPower(-0.1);
        }

//        if (Math.abs(Wobble.wobbleMotor.getCurrentPosition()) < (-lifterTP * .1)) {
//            Wobble.wobbleMotor.setPower(0.4);
//        }
    }

    public static void raiseArm(int lifterTP){
        //if(Math.abs(lifterTP - Wobble.wobbleMotor.getCurrentPosition()) < (-lifterTP * .1)) {
            Wobble.wobbleMotor.setTargetPosition(lifterTP);
            Wobble.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Wobble.wobbleMotor.setPower(-.4);
        //}
        if (Math.abs(Wobble.wobbleMotor.getCurrentPosition()) < Math.abs(lifterTP * WOBBLE_MOTOR_ERROR)) {
            Wobble.wobbleMotor.setPower(0);
        }
        close();
    }

    public static void wobbleChangeState(){
        if(currentState == WOBBLE_STATE.INIT){
            currentState = WOBBLE_STATE.DOWN;
        }
        else if (currentState == WOBBLE_STATE.DOWN){
            currentState = WOBBLE_STATE.UP;
        }
        else if(currentState == WOBBLE_STATE.UP){
            currentState = WOBBLE_STATE.DROP;
        }
        else if(currentState == WOBBLE_STATE.DROP){
            currentState = WOBBLE_STATE.DOWN;
        }
    }

    public static void wobbleUpdatePosition(){
        if(currentState == WOBBLE_STATE.INIT){
            wobbleMotor.setTargetPosition(wobbleMotor.getCurrentPosition());
            wobbleMotor.setPower(0);
            close();
        }
        else if (currentState == WOBBLE_STATE.DOWN){
            Wobble.lowerArm(WOBBLE_DOWN_TICKS);
        }
        else if(currentState == WOBBLE_STATE.UP){
            Wobble.raiseArm(WOBBLE_UP_TICKS);
            close();
        }
        else if(currentState == WOBBLE_STATE.DROP){
            Wobble.lowerArm(WOBBLE_DROP_TICKS);
        }
    }


    public static void wobbleTelemetry(Telemetry telemetry){
        telemetry.addData("Lifter Pos: ", Wobble.wobbleMotor.getCurrentPosition());
        telemetry.addData("Lifter TP: ", Wobble.wobbleMotor.getTargetPosition());
        telemetry.addData("Lifter Pow: ", Wobble.wobbleMotor.getPower());
        telemetry.addLine();
        //telemetry.addData("wobble1:", wobble1.getPosition());
        telemetry.addData("wobble:", wobble1.getPosition());
        telemetry.addData("wobble 2:", wobble2.getPosition());

        telemetry.addLine();
    }
}

