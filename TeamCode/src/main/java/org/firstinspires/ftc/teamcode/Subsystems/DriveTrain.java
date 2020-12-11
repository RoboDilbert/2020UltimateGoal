package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Util.HardwarePresets;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain extends HardwarePresets{

    public DriveTrain(){

    }

    public void Drive(String input, int encoderTicks, double power){
        if(input.equals("STRAFE_LEFT")){
            leftFrontMotor.setTargetPosition(encoderTicks);
            leftBackMotor.setTargetPosition(-encoderTicks);
            rightFrontMotor.setTargetPosition(-encoderTicks);
            rightBackMotor.setTargetPosition(encoderTicks);

            while(anyDriveMotorsBusy()) {
                leftFrontMotor.setPower(power);
                leftBackMotor.setPower(-power);
                rightFrontMotor.setPower(-power);
                rightBackMotor.setPower(power);
            }
        }
        if(input.equals("STRAFE_RIGHT")){
            leftFrontMotor.setTargetPosition(-encoderTicks);
            leftBackMotor.setTargetPosition(encoderTicks);
            rightFrontMotor.setTargetPosition(encoderTicks);
            rightBackMotor.setTargetPosition(-encoderTicks);

            while(anyDriveMotorsBusy()){
                leftFrontMotor.setPower(-power);
                leftBackMotor.setPower(power);
                rightFrontMotor.setPower(power);
                rightBackMotor.setPower(-power);
            }
        }
        if(input.equals("FORWARD")){
            leftFrontMotor.setTargetPosition(encoderTicks);
            leftBackMotor.setTargetPosition(encoderTicks);
            rightFrontMotor.setTargetPosition(encoderTicks);
            rightBackMotor.setTargetPosition(encoderTicks);

            while(anyDriveMotorsBusy()){
                leftFrontMotor.setPower(power);
                leftBackMotor.setPower(power);
                rightFrontMotor.setPower(power);
                rightBackMotor.setPower(power);
            }
        }
        if(input.equals("REVERSE")) {
            leftFrontMotor.setTargetPosition(-encoderTicks);
            leftBackMotor.setTargetPosition(-encoderTicks);
            rightFrontMotor.setTargetPosition(-encoderTicks);
            rightBackMotor.setTargetPosition(-encoderTicks);

            while (anyDriveMotorsBusy()) {
                leftFrontMotor.setPower(-power);
                leftBackMotor.setPower(-power);
                rightFrontMotor.setPower(-power);
                rightBackMotor.setPower(-power);
            }
        }
    }

    public void Turn(String input, double power, float degrees){
//        telemetry.addData("heading", robot.angles);
//        telemetry.update();
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
        if (leftFrontMotor.isBusy() || leftBackMotor.isBusy() || rightFrontMotor.isBusy() || rightBackMotor.isBusy()) {
            return (true);
        } else {
            return (false);
        }
    }

    public void DriveTelemetry(){

    }
}
