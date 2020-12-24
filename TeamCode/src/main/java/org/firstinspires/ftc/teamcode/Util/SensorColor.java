package org.firstinspires.ftc.teamcode.Util;

import android.app.Notification;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

public class SensorColor extends HardwarePresets {

    @Override
    public void init(HardwareMap hwm) {
        super.init(hwm);
    }

    private Telemetry telemetry;

    //Instance Fields
    //Constructor
    public SensorColor() {
    }

    //Methods
    public void DriveToLine(String color){
        super.init(HwMap);
        if(color.equals("RED")){
            while(cranberi.red() < 150 /*|| robot.cranberi.blue() > 50*/){
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
            while(cranberi.red() < 300 && cranberi.blue() < 350){

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
    }
}
