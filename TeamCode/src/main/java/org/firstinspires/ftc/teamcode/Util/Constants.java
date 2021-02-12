package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wobble;


public class Constants {

    public static HardwareMap HwMap;

    public static Rolling Distance1 = new Rolling(20);
    public static Rolling Distance2 = new Rolling (20);
    public static Intake mainIntake = new Intake();
    public static DriveTrain drive = new DriveTrain();
    public static Wobble wobb = new Wobble();

    public static double cal1 = 0;
    public static double cal2 = 0;

    public static final double TELEOP_LIMITER = 0.8;
}
