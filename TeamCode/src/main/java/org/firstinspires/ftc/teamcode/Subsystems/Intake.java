package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UtilOG.Constants;
import java.util.ArrayList;

public class Intake {
    //Instance Fields
    //This is the array for the number of rings in our robot
    private static ArrayList rings = new ArrayList(4);
    private static boolean ringCountFlag = false;
    private static final double ringDistance = 13;

    //Declare motors and sensors
    private static DistanceSensor indexSensor; //Expansion hub, port 1

    private static DcMotor frontIntakeMotor; //Control hub, port 2
    private static DcMotor rearIntakeMotor; //Control hub, port 3

    //This is the servo that vibrates to push rings into the shooter
    private static Servo vibrator; //Control hub, port 0

    //Constants for shooter claw
    private static final double VIBRATOR_CLOSED = 0.4;
    private static final double VIBRATOR_OPEN = 0.58;

    private static INTAKE_STATE currentState = INTAKE_STATE.OFF;

    private enum INTAKE_STATE{
        INTAKE,
        OFF,
        BACKWARDS;
    }

    //Constructor
    public Intake() {}

    //Init intake
    public static void initIntake(HardwareMap hwm){
        //Declare the motors and servos in the hardware map
        Constants.HwMap = hwm;
        frontIntakeMotor = Constants.HwMap.dcMotor.get("frontIntakeMotor");
        rearIntakeMotor = Constants.HwMap.dcMotor.get("rearIntakeMotor");

        indexSensor = Constants.HwMap.get(DistanceSensor.class, "indexSensor");

        vibrator = Constants.HwMap.servo.get("vibrator");
        vibrator.setPosition(VIBRATOR_OPEN);

        //Set directions for the intake motors
        frontIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //Release one ring from the shooter
    public static void releaseOne() throws InterruptedException {
        Shooter.setPosition("WHITE_LINE");
        vibrator.setPosition(VIBRATOR_OPEN);
        Thread.sleep(125);
        vibrator.setPosition(VIBRATOR_CLOSED);
        Thread.sleep(100);
        vibrator.setPosition(VIBRATOR_OPEN);
        Thread.sleep(125);
        vibrator.setPosition(VIBRATOR_CLOSED);
        Thread.sleep(100);
    }

    //Release all of the rings in the shooter
    public static void releaseAll() throws InterruptedException {
        Shooter.unblock();
        Thread.sleep(50);
        for(int i = 0; i < 8; i++) {
            vibrator.setPosition(VIBRATOR_OPEN);
            Thread.sleep(150);
            vibrator.setPosition(VIBRATOR_CLOSED);
            Thread.sleep(100);
        }
        Shooter.setPosition("WHITE_LINE");
        vibrator.setPosition(VIBRATOR_OPEN);
        rings.clear();
    }

    //Release all of the rings from a further back position on the playing field
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
    //This shoots all of the rings without clearing the array of rings in the robot
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

    //Spit one ring in case we have 4 rings in the robot
    public static void spit() throws InterruptedException {
        Shooter.mainShooter.shoot(.2);
        Thread.sleep(500);
        vibrator.setPosition(VIBRATOR_OPEN);
        Thread.sleep(100);
        vibrator.setPosition(VIBRATOR_CLOSED);
        Thread.sleep(200);
        Shooter.mainShooter.shoot(Shooter.SHOOTER_POWER);
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

    //Index one ring into the array of rings in our robot
     public static void index ()  throws InterruptedException {
        if (rings.lastIndexOf(true) < 2) {
            ringCountFlag = true;
        }
        else {
            ringCountFlag = true;
        }
    }

    //Turn on the intake
    public static void intake() throws InterruptedException {
        frontIntakeMotor.setPower(0.95);
        rearIntakeMotor.setPower(0.95);
    }

    //Intake at a slower speed
    public static void intakeTwo(){
        frontIntakeMotor.setPower(.85);
        rearIntakeMotor.setPower(.85);
    }

    //Telemtetry for the intake
    public static void intakeTelemetry(Telemetry telemetry){
        telemetry.addData("Vibrator Pos:", vibrator.getPosition());
    }

    //Intake backwards
    public static void setBackwards(){
        frontIntakeMotor.setPower(-.85);
        rearIntakeMotor.setPower(-.85);
    }

    //Stop the intake
    public static void stop(){
        frontIntakeMotor.setPower(0);
        rearIntakeMotor.setPower(0);
    }

    //Set run mode for the intake motors
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

    //This shoots one of the rings without clearing the array of rings in the robot
    public static void shootOneNoClear() throws InterruptedException{
        vibrator.setPosition(VIBRATOR_OPEN);
        Thread.sleep(200);
        vibrator.setPosition(VIBRATOR_CLOSED);
        Thread.sleep(100);
    }

    //Sets the vibrator servo to the back position (open)
    public static void defaultPos(){
        vibrator.setPosition(VIBRATOR_OPEN);
    }

    //States for the intake (Forward, backwards, stop)
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

    //Updates the powers for the intake based on the states above
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