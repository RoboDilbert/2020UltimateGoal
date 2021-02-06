package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.HardwarePresets;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain extends HardwarePresets{

    @Override
    public void init(HardwareMap hwm) {
        super.init(hwm);
    }

    //Constructor
    public DriveTrain(){}

    public void Drive(String input, int encoderTicks, double power){
        super.init(HwMap);
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
        while(pewpewboi.getDistance(DistanceUnit.CM) > 10) {
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
        super.init(HwMap);
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
        super.init(HwMap);
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
        super.init(HwMap);
        if (leftFrontMotor.isBusy() /*|| leftBackMotor.isBusy() || rightFrontMotor.isBusy() || rightBackMotor.isBusy()*/) {
            return (true);
        } else {
            return (false);
        }
    }

    public void DriveTelemetry(){

    }
}
