package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Util.*;

@TeleOp(name= "TeleOop", group= "TeleOp")
//@Disabled
public class StarterTeleop extends LinearOpMode {

    HardwarePresets robot = new HardwarePresets();
    //Constants constant = new Constants();

    public double drive = 0;
    public double strafe = 0;
    public double twist = 0;
    public final double TELEOP_LIMITER = 0.6;
    public float gyroVariation = 0;

    @Override
    public void runOpMode(){

        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            //Mecanum Drive
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double heading;

            if(x == 0 && y == 0)
                heading = 0;
            else if(x >= 0)
                heading = Math.PI - Math.atan(y / x);

            else
                heading = - Math.atan(y / x);

            double pow = Math.sqrt(Math.pow(x, 6) + Math.pow(y, 6));
            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            heading -= (angles.firstAngle + Math.PI / 4.0);

            double pow1 = Math.sqrt(2) * pow * Math.cos(heading + gyroVariation);
            double pow2 = Math.sqrt(2) * pow * Math.sin(heading + gyroVariation);

            robot.leftFrontMotor.setPower(turn + pow1);
            robot.leftBackMotor.setPower(turn + pow2);
            robot.rightFrontMotor.setPower(pow1 - turn);
            robot.rightBackMotor.setPower(pow2 - turn);

//            drive  = gamepad1.left_stick_y * TELEOP_LIMITER;
//            strafe = gamepad1.left_stick_x * TELEOP_LIMITER;
//            twist  = gamepad1.right_stick_x * TELEOP_LIMITER;
//
//            robot.leftFrontMotor.setPower(drive + strafe + twist);
//            robot.leftBackMotor.setPower(drive - strafe + twist);
//            robot.rightFrontMotor.setPower(drive - strafe - twist);
//            robot.rightBackMotor.setPower(drive + strafe - twist);
        }
    }
}