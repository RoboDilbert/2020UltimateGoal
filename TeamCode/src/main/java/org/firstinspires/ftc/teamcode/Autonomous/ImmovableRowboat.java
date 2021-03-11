package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MasterVision;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Wobble;


@Autonomous(name= "Immovable Rowboat", group= "Autonomous")
@Disabled
public class ImmovableRowboat extends LinearOpMode {


    public void runOpMode() throws InterruptedException {

        DriveTrain.initDriveTrain(hardwareMap);
        Shooter.initShooter(hardwareMap);
        Intake.initIntake(hardwareMap);
        Wobble.initWobble(hardwareMap);

        waitForStart();

        DriveTrain.setRunMode("STOP_AND_RESET_ENCODER");

        while(opModeIsActive()) {
            DriveTrain.leftFrontMotor.setTargetPosition(0);
            DriveTrain.leftBackMotor.setTargetPosition(0);
            DriveTrain.rightFrontMotor.setTargetPosition(0);
            DriveTrain.rightBackMotor.setTargetPosition(0);
            DriveTrain.setRunMode("RUN_TO_POSITION");
            DriveTrain.leftFrontMotor.setPower(0.8);
            DriveTrain.leftBackMotor.setPower(0.8);
            DriveTrain.rightFrontMotor.setPower(0.8);
            DriveTrain.rightBackMotor.setPower(0.8);
        }
    }
}
