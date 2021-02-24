package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.Constants;
import java.util.ArrayList;

public class Intake {
    //Instance Fields
    private static ArrayList rings = new ArrayList(4);
    private static boolean ringCountFlag = false;
    private static final double ringDistance = 13;

    private static DistanceSensor indexSensor; //Expansion hub, port 1

    private static DcMotor frontIntakeMotor; //Control hub, port 2
    private static DcMotor rearIntakeMotor; //Control hub, port 3

    private static Servo vibrator; //Control hub, port 0

    private static final double VIBRATOR_CLOSED = 0.4;
    private static final double VIBRATOR_OPEN = 0.55;

    private static INTAKE_STATE currentState = INTAKE_STATE.OFF;

    private enum INTAKE_STATE{
        INTAKE,
        OFF,
        BACKWARDS;
    }

    //Constructor
    public Intake() {}

    //Methods
    public static void initIntake(HardwareMap hwm){
        Constants.HwMap = hwm;
        frontIntakeMotor = Constants.HwMap.dcMotor.get("frontIntakeMotor");
        rearIntakeMotor = Constants.HwMap.dcMotor.get("rearIntakeMotor");

        indexSensor = Constants.HwMap.get(DistanceSensor.class, "indexSensor");

        vibrator = Constants.HwMap.servo.get("vibrator");
        vibrator.setPosition(VIBRATOR_OPEN);

        frontIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //Release 1
    public static void releaseOne() throws InterruptedException {
        if(rings.lastIndexOf(true) >= 0) {
            vibrator.setPosition(VIBRATOR_OPEN);
            Thread.sleep(125);
            vibrator.setPosition(VIBRATOR_CLOSED);
            Thread.sleep(100);
            rings.remove(rings.lastIndexOf(true));
        }
    }
    //Release All
    public static void releaseAll() throws InterruptedException {
//        vibrator.setPosition(VIBRATOR_OPEN);
//        Thread.sleep(150);
//        vibrator.setPosition(VIBRATOR_CLOSED);
//        Thread.sleep(100);
//        Shooter.setPosition("SHOOTING");
//        //Thread.sleep(100);
        for(int i = 0; i < 8; i++) {
            vibrator.setPosition(VIBRATOR_OPEN);
            Thread.sleep(150);
            vibrator.setPosition(VIBRATOR_CLOSED);
            Thread.sleep(100);
        }
        Shooter.setPosition("WHITE_LINE");
        rings.clear();
    }

    public static void releaseAllRings() throws InterruptedException {
        Thread.sleep(250);
        vibrator.setPosition(VIBRATOR_OPEN);
        Thread.sleep(150);
        vibrator.setPosition(VIBRATOR_CLOSED);
        Thread.sleep(100);
        Shooter.setPosition("RINGS_ADJUST");
        //Thread.sleep(100);
        for(int i = 0; i < 5; i++) {
            vibrator.setPosition(VIBRATOR_OPEN);
            Thread.sleep(150);
            vibrator.setPosition(VIBRATOR_CLOSED);
            Thread.sleep(100);
        }
        Shooter.setPosition("WHITE_LINE");
        rings.clear();
    }
    public static void shootAllNoClear()  throws InterruptedException {
        vibrator.setPosition(VIBRATOR_OPEN);
        Thread.sleep(125);
        vibrator.setPosition(VIBRATOR_CLOSED);
        Thread.sleep(200);
        Shooter.setPosition("SHOOTING");
        for(int i = 0; i < 6; i++) {
            vibrator.setPosition(VIBRATOR_OPEN);
            Thread.sleep(125);
            vibrator.setPosition(VIBRATOR_CLOSED);
            Thread.sleep(100);
        }
        Shooter.setPosition("WHITE_LINE");
    }


    //Spit one ring
    public static void spit() throws InterruptedException {
        Shooter.mainShooter.shoot(.4);
        Thread.sleep(250);
        vibrator.setPosition(VIBRATOR_OPEN);
        Thread.sleep(100);
        vibrator.setPosition(VIBRATOR_CLOSED);
        Thread.sleep(200);
        rings.remove(rings.lastIndexOf(true));
    }

    //Check to see if there are 3 rings
    public static boolean isFull() {
        if (rings.lastIndexOf(true) == 2) {
            return true;
        }
        else {
            return false;
        }
    }

        //Index
     public static void index ()  throws InterruptedException {
        if (rings.lastIndexOf(true) < 2) {
            ringCountFlag = true;
        }
        else {
            ringCountFlag = true;
            spit();
        }
    }
    public static void intake() throws InterruptedException {
        if (indexSensor.getDistance(DistanceUnit.CM) < ringDistance) {
            index();
        }
        else if (ringCountFlag) {
            rings.add(true);
            ringCountFlag = false;
        }
        frontIntakeMotor.setPower(0.95);
        rearIntakeMotor.setPower(0.95);

        if(isFull()){
            Shooter.shoot(0);
        }
    }

    public static void intakeTwo(){
        frontIntakeMotor.setPower(.85);
        rearIntakeMotor.setPower(.85);
    }

    public static void intakeTelemetry(Telemetry telemetry){
        //telemetry.addData("Vibrator:", vibrator.getPosition());
//        telemetry.addLine();
//        telemetry.addData("indexSensor", String.format("%.3f cm", Intake.indexSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("rings: ", rings);
//        telemetry.addData("Ring Flag: ", ringCountFlag);
//        telemetry.addData("Intake Array Size:", rings.lastIndexOf(true));
//        telemetry.addLine();
    }

    public static void setForward(){
        frontIntakeMotor.setPower(.95);
        rearIntakeMotor.setPower(.95);
    }
    public static void setBackwards(){
        frontIntakeMotor.setPower(-.85);
        rearIntakeMotor.setPower(-.85);
    }
    public static void stop(){
        frontIntakeMotor.setPower(0);
        rearIntakeMotor.setPower(0);
    }
    public static void intakeRunMode(String mode){
        if(mode.equals("RUN_USING_ENCODER")){
            rearIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if(mode.equals("RUN_WITHOUT_ENCODER")){
            rearIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public static void shootOneNoClear() throws InterruptedException{
        vibrator.setPosition(0.65);
        Thread.sleep(100);
        vibrator.setPosition(VIBRATOR_CLOSED);
        Thread.sleep(100);
    }
    public static void defaultPos(){
        vibrator.setPosition(0.55);
    }

    public static void intakeChangeState(String direction){
        if(currentState == INTAKE_STATE.OFF && direction.equals("FORWARD")){
            currentState = INTAKE_STATE.INTAKE;
        }
        else if (currentState == INTAKE_STATE.INTAKE){
            currentState = INTAKE_STATE.OFF;
        }
        else if(currentState == INTAKE_STATE.OFF && direction.equals("REVERSE")){
            currentState = INTAKE_STATE.BACKWARDS;
        }
        else if(currentState == INTAKE_STATE.BACKWARDS){
            currentState = INTAKE_STATE.OFF;
        }
    }

    public static void intakeUpdatePosition() throws InterruptedException{
        if(currentState == INTAKE_STATE.OFF){
            stop();
        }
        else if (currentState == INTAKE_STATE.INTAKE){
            intake();
        }
        else if(currentState == INTAKE_STATE.BACKWARDS){
            setBackwards();
        }
    }
}