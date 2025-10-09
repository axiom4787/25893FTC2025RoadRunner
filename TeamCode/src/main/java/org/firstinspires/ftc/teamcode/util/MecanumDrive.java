package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {
    DcMotor leftFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightFrontDrive;
    DcMotor rightBackDrive;
    private HardwareMap hardwareMap;


    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        leftFrontDrive = this.hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = this.hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = this.hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = this.hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setDrivePower(double leftFront, double leftBack, double rightFront, double rightBack) {
        leftFrontDrive.setPower(leftFront);
        leftBackDrive.setPower(leftBack);
        rightFrontDrive.setPower(rightFront);
        rightBackDrive.setPower(rightBack);
    }

    public void driveXYZ(double strafe, double forward, double heading) {
        leftFrontDrive.setPower(-heading + strafe + forward);
        leftBackDrive.setPower(-heading - strafe + forward);
        rightFrontDrive.setPower(heading + strafe + forward);
        rightBackDrive.setPower(heading - strafe + forward);
    }
}
