package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class SensorColor extends HardwarePresets {

    @Override
    public void init(HardwareMap hwm) { super.init(hwm); }

    //Constructor
    public SensorColor() {}

    //Methods
    public void DriveToLine(String color){
        super.init(HwMap);

        if(color.equals("RED")){
            while(cranberi.red() < 150 || cranberi.alpha() > 500){//240, 82
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
            while(cranberi.red() < 300 && cranberi.blue() < 350){//480, 680

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
