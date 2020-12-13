package org.firstinspires.ftc.teamcode.Util;

import android.app.Notification;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

public class SensorColor extends HardwarePresets{

    private Telemetry telemetry;

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
                telemetry.addLine()
                        .addData("Red", "%.3f", colors.red * 255)
                        .addData("Blue", "%.3f", colors.blue * 255)
                        .addData("Alpha", "%.3f", colors.alpha * 255);
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
                telemetry.addLine()
                        .addData("Red", "%.3f", colors.red * 255)
                        .addData("Blue", "%.3f", colors.blue * 255)
                        .addData("Alpha", "%.3f", colors.alpha * 255);
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
