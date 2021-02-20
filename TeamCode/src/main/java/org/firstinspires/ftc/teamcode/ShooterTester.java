package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

public class ShooterTester {

    private static DcMotorEx shooter; //Control hub, port 0
    private static Shooter mainShooter;
    private static Servo angleAdjust; //Control hub, port 2

    private static double NEW_P;//18.6
    private static double NEW_I;
    private static double NEW_D;
    private static double NEW_F;
}
