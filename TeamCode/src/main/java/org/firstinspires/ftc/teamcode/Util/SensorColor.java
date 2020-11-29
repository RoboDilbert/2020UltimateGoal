package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class SensorColor extends HardwarePresets{

    //Instance Fields

    //Constructor
    public SensorColor() {

    }

    //Methods
    public void DriveToLine(String color){
        if(color.equals("RED")){
            while(cranberi.red() < 200){
                leftFrontMotor.setPower(0.2);
                rightFrontMotor.setPower(0.2);
                leftBackMotor.setPower(0.2);
                rightBackMotor.setPower(0.2);
            }
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
        }
        else if(color.equals("WHITE")) {
            while(cranberi.argb() < 200){
                leftFrontMotor.setPower(0.2);
                rightFrontMotor.setPower(0.2);
                leftBackMotor.setPower(0.2);
                rightBackMotor.setPower(0.2);
            }
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
        }
    }
}
