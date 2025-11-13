package org.firstinspires.ftc.teamcode;
import android.view.ViewGroup;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.*;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;


@TeleOp(name="Alert: Control Hub Is Calling", group="Linear OpMode")
//@Disabled
public class ControlHubIsCalling extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive();

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            drive.setDrivePower(
                    (gamepad1.x?1:0), // left front | Right back
                    (gamepad1.a?1:0), // left back | left Back
                    (gamepad1.y?1:0), // right front | right back
                    (gamepad1.b?1:0) // right back | left front
            );
        }
    }
}