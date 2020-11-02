package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Util.*;

<<<<<<< HEAD
@Autonomous(name= "TeleOop", group= "Autonomous")
=======
@TeleOp(name= "TeleOop", group= "TeleOp")
>>>>>>> Initial commit
//@Disabled
public class StarterTeleop extends LinearOpMode {

    HardwarePresets robot = new HardwarePresets();
    //Constants constant = new Constants();

    public double drive = 0;
    public double strafe = 0;
    public double twist = 0;
    public final double TELEOP_LIMITER = 0.6;

    @Override
    public void runOpMode(){

        robot.init(hardwareMap);

<<<<<<< HEAD
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
=======
>>>>>>> Initial commit

        waitForStart();

        while(opModeIsActive()){

            //Mecanum Drive
            drive  = gamepad1.left_stick_y * TELEOP_LIMITER;
            strafe = gamepad1.left_stick_x * TELEOP_LIMITER;
            twist  = gamepad1.right_stick_x * TELEOP_LIMITER;

            robot.leftFrontMotor.setPower(drive + strafe + twist);
<<<<<<< HEAD
            robot.leftFrontMotor.setPower(drive - strafe + twist);
            robot.leftFrontMotor.setPower(drive - strafe - twist);
            robot.leftFrontMotor.setPower(drive + strafe - twist);
=======
            robot.leftBackMotor.setPower(drive - strafe + twist);
            robot.rightFrontMotor.setPower(drive - strafe - twist);
            robot.rightBackMotor.setPower(drive + strafe - twist);
>>>>>>> Initial commit
        }
    }
}
