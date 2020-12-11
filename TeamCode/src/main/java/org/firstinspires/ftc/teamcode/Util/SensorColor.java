package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class SensorColor extends HardwarePresets{

    //Instance Fields
    //Constructor
    public SensorColor() {
    }

    //Methods
    public void DriveToLine(String color){
        NormalizedColorSensor colorSensor;
        colorSensor = HwMap.get(NormalizedColorSensor.class, "cranberi");
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        if(color.equals("RED")){
            while(colors.red < 5 || colors.blue > 2){
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
            while(colors.red < 10 || colors.blue < 10){
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
