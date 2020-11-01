package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Util.HardwarePresets;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class DriveTrain{

    HardwarePresets robot = new HardwarePresets();

    public void Drive(String input, int encoderTicks, double power){
        if(input.equals("STRAFE_LEFT")){
            robot.leftFrontMotor.setTargetPosition(encoderTicks);
            robot.leftBackMotor.setTargetPosition(-encoderTicks);
            robot.rightFrontMotor.setTargetPosition(-encoderTicks);
            robot.rightBackMotor.setTargetPosition(encoderTicks);

            while(anyDriveMotorsBusy()) {
                robot.leftFrontMotor.setPower(power);
                robot.leftBackMotor.setPower(-power);
                robot.rightFrontMotor.setPower(-power);
                robot.rightBackMotor.setPower(power);
            }
        }
        if(input.equals("STRAFE_RIGHT")){
            robot.leftFrontMotor.setTargetPosition(-encoderTicks);
            robot.leftBackMotor.setTargetPosition(encoderTicks);
            robot.rightFrontMotor.setTargetPosition(encoderTicks);
            robot.rightBackMotor.setTargetPosition(-encoderTicks);

            while(anyDriveMotorsBusy()){
                robot.leftFrontMotor.setPower(-power);
                robot.leftBackMotor.setPower(power);
                robot.rightFrontMotor.setPower(power);
                robot.rightBackMotor.setPower(-power);
            }
        }
        if(input.equals("FORWARD")){
            robot.leftFrontMotor.setTargetPosition(encoderTicks);
            robot.leftBackMotor.setTargetPosition(encoderTicks);
            robot.rightFrontMotor.setTargetPosition(encoderTicks);
            robot.rightBackMotor.setTargetPosition(encoderTicks);

            while(anyDriveMotorsBusy()){
                robot.leftFrontMotor.setPower(power);
                robot.leftBackMotor.setPower(power);
                robot.rightFrontMotor.setPower(power);
                robot.rightBackMotor.setPower(power);
            }
        }
        if(input.equals("REVERSE")) {
            robot.leftFrontMotor.setTargetPosition(-encoderTicks);
            robot.leftBackMotor.setTargetPosition(-encoderTicks);
            robot.rightFrontMotor.setTargetPosition(-encoderTicks);
            robot.rightBackMotor.setTargetPosition(-encoderTicks);

            while (anyDriveMotorsBusy()) {
                robot.leftFrontMotor.setPower(-power);
                robot.leftBackMotor.setPower(-power);
                robot.rightFrontMotor.setPower(-power);
                robot.rightBackMotor.setPower(-power);
            }
        }
    }

    public void Turn(String input, double power, int degrees){
        telemetry.addData("heading", robot.angles);
        telemetry.update();
        if(input.equals("TURN_LEFT")){
            while (robot.angles.firstAngle < degrees) {
                robot.leftFrontMotor.setPower(-power);
                robot.leftBackMotor.setPower(-power);
                robot.rightFrontMotor.setPower(power);
                robot.rightBackMotor.setPower(power);
            }
        }
        if(input.equals("TURN_RIGHT")){
            while (robot.angles.firstAngle > degrees) {
                robot.leftFrontMotor.setPower(power);
                robot.leftBackMotor.setPower(power);
                robot.rightFrontMotor.setPower(-power);
                robot.rightBackMotor.setPower(-power);
            }
        }
    }
    public void setRunMode(String input) {
        if (input.equals("STOP_AND_RESET_ENCODER")) {
            robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (input.equals("RUN_WITHOUT_ENCODER")) {
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (input.equals("RUN_USING_ENCODER")) {
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (input.equals("RUN_TO_POSITION")) {
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    //Returns TRUE if any drive motors are busy and FALSE if not.
    public boolean anyDriveMotorsBusy() {
        if (robot.leftFrontMotor.isBusy() || robot.leftBackMotor.isBusy() || robot.rightFrontMotor.isBusy() || robot.rightBackMotor.isBusy()) {
            return (true);
        } else {
            return (false);
        }
    }

    public void DriveTelemetry(){

    }
}