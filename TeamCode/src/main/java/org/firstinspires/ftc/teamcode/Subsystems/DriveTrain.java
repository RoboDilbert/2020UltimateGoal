package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.HardwarePresets;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain{

    //Motors
    public DcMotor leftFrontMotor; //Expansion hub, port 0
    public DcMotor leftBackMotor; //Expansion hub, port 1
    public DcMotor rightFrontMotor; //Expansion hub, port 3
    public DcMotor rightBackMotor; //Expansion hub, port 2

    public ColorSensor floorColorSensor; //Control hub, I2C Bus 1

    //2m distance sensors
    public DistanceSensor laserboi; //Control hub, I2C Bus 2
    //public DistanceSensor pewpewboi; //Control hub, I2C Bus 3

    public void initDriveTrain(HardwareMap HwMap) {
        //super.init(hwm);
        leftFrontMotor = HwMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = HwMap.dcMotor.get("leftBackMotor");
        rightFrontMotor = HwMap.dcMotor.get("rightFrontMotor");
        rightBackMotor = HwMap.dcMotor.get("rightBackMotor");

        floorColorSensor = HwMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "floorColorSensor");

        laserboi = HwMap.get(DistanceSensor.class, "laserboi");
        //pewpewboi = HwMap.get(DistanceSensor.class, "pewpewboi");


        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Constructor
    public DriveTrain(){}

    public void Drive(String input, int encoderTicks, double power){
        if(input.equals("STRAFE_RIGHT")){
            leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + encoderTicks);
            leftBackMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - encoderTicks);
            rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - encoderTicks);
            rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + encoderTicks);
            setRunMode("RUN_TO_POSITION");

            while(anyDriveMotorsBusy()) {
                leftFrontMotor.setPower(power);
                leftBackMotor.setPower(-power);
                rightFrontMotor.setPower(-power);
                rightBackMotor.setPower(power);
            }
        }
        if(input.equals("STRAFE_LEFT")){
            leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - encoderTicks);
            leftBackMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + encoderTicks);
            rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + encoderTicks);
            rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - encoderTicks);
            setRunMode("RUN_TO_POSITION");

            while(anyDriveMotorsBusy()){
                leftFrontMotor.setPower(-power);
                leftBackMotor.setPower(power);
                rightFrontMotor.setPower(power);
                rightBackMotor.setPower(-power);
            }
        }
        if(input.equals("FORWARD")){
            leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + encoderTicks);
            leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + encoderTicks);
            rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + encoderTicks);
            rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + encoderTicks);
            setRunMode("RUN_TO_POSITION");

            while(anyDriveMotorsBusy()){
                leftFrontMotor.setPower(power);
                leftBackMotor.setPower(power);
                rightFrontMotor.setPower(power);
                rightBackMotor.setPower(power);
            }
        }
        if(input.equals("REVERSE")) {
            leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - encoderTicks);
            leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - encoderTicks);
            rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - encoderTicks);
            rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - encoderTicks);
            setRunMode("RUN_TO_POSITION");

            while (anyDriveMotorsBusy()) {
                leftFrontMotor.setPower(-power);
                leftBackMotor.setPower(-power);
                rightFrontMotor.setPower(-power);
                rightBackMotor.setPower(-power);
            }
        }
        if(input.equals("FORWARD_LEFT")){
            leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - encoderTicks);
            leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + (6 * encoderTicks));
            rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + (6 * encoderTicks));
            rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - encoderTicks);
            setRunMode("RUN_TO_POSITION");

            while(anyDriveMotorsBusy()){
                leftFrontMotor.setPower(power * -.5);
                leftBackMotor.setPower(power * 3);
                rightFrontMotor.setPower(power * 3);
                rightBackMotor.setPower(power * -.5);
            }
        }
        if(input.equals("SLIGHTLY_LEFT")){
            leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - encoderTicks);
            leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - encoderTicks);
            rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - (2 * encoderTicks));
            rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - (2 * encoderTicks));
            setRunMode("RUN_TO_POSITION");

            while(anyDriveMotorsBusy()){
                leftFrontMotor.setPower(power * -0.8);
                leftBackMotor.setPower(power * -0.8);
                rightFrontMotor.setPower(power * -1.6);
                rightBackMotor.setPower(power * -1.6);
            }
        }
        if(input.equals("FORWARD_LEFT")){
            leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - encoderTicks);
            leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + (6 * encoderTicks));
            rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + (6 * encoderTicks));
            rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - encoderTicks);
            setRunMode("RUN_TO_POSITION");

            while(anyDriveMotorsBusy()){
                leftFrontMotor.setPower(power * -.5);
                leftBackMotor.setPower(power * 3);
                rightFrontMotor.setPower(power * 3);
                rightBackMotor.setPower(power * -.5);
            }
        }

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
    public void driveToRing(double power){
        while(laserboi.getDistance(DistanceUnit.CM) > 10) {
                leftFrontMotor.setPower(power);
                leftBackMotor.setPower(power);
                rightFrontMotor.setPower(power);
                rightBackMotor.setPower(power);
        }

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

    }

    public void Turn(String input, double power, float degrees){
        if(input.equals("TURN_LEFT")){
            float targetLocation = angles.firstAngle - degrees;
            if(targetLocation < -180){
                targetLocation = Math.abs(targetLocation + 180);
            }
            while (angles.firstAngle < targetLocation - 2 && angles.firstAngle > targetLocation + 2) {
                leftFrontMotor.setPower(-power);
                leftBackMotor.setPower(-power);
                rightFrontMotor.setPower(power);
                rightBackMotor.setPower(power);
            }
        }
        if(input.equals("TURN_RIGHT")){
            float targetLocation = angles.firstAngle + degrees;
            if(targetLocation > 180){
                targetLocation = -(targetLocation - 180);
            }
            while (angles.firstAngle < targetLocation - 2 && angles.firstAngle > targetLocation + 2) {
                leftFrontMotor.setPower(power);
                leftBackMotor.setPower(power);
                rightFrontMotor.setPower(-power);
                rightBackMotor.setPower(-power);
            }
        }

    }
    public void setRunMode(String input) {
        if (input.equals("STOP_AND_RESET_ENCODER")) {
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (input.equals("RUN_WITHOUT_ENCODER")) {
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (input.equals("RUN_USING_ENCODER")) {
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (input.equals("RUN_TO_POSITION")) {
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }


    //Returns TRUE if any drive motors are busy and FALSE if not.
    public boolean anyDriveMotorsBusy() {
        if (leftFrontMotor.isBusy() /*|| leftBackMotor.isBusy() || rightFrontMotor.isBusy() || rightBackMotor.isBusy()*/) {
            return (true);
        } else {
            return (false);
        }
    }

    public void DriveTelemetry(Telemetry telemetry){
            telemetry.addData("left front encoder", leftFrontMotor.getCurrentPosition());
            telemetry.addData("left back encoder", leftBackMotor.getCurrentPosition());
            telemetry.addData("right front encoder", rightFrontMotor.getCurrentPosition());
            telemetry.addData("right back encoder", rightBackMotor.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("left front power", leftFrontMotor.getPower());
            telemetry.addData("left back power", leftBackMotor.getPower());
            telemetry.addData("right front power", rightFrontMotor.getPower());
            telemetry.addData("right back power", rightBackMotor.getPower());
    }

    public void DriveToLine(String color){
        if(color.equals("RED")){
            while(floorColorSensor.red() < 175){//240, 82
                leftFrontMotor.setPower(0.3);
                rightFrontMotor.setPower(0.3);
                leftBackMotor.setPower(0.3);
                rightBackMotor.setPower(0.3);
            }
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
        }
        else if(color.equals("WHITE")) {
            while(floorColorSensor.red() < 190){//480, 680
                leftFrontMotor.setPower(0.4);
                rightFrontMotor.setPower(0.4);
                leftBackMotor.setPower(0.4);
                rightBackMotor.setPower(0.4);
            }
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
        }
    }
}

