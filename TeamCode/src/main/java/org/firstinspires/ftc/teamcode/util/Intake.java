package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor intakeMotor;
    DcMotor shooter;
    private HardwareMap hardwareMap;
    // Make sure to rename motor port 0 of expansion hub to "intake"

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        intakeMotor = this.hardwareMap.get(DcMotor.class, "intake");
        shooter = this.hardwareMap.get(DcMotor.class, "shooter");

        // Set intake motor directions.
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setIntake(double power) {
        intakeMotor.setPower(power);
    }

    public void setShooter(double power) {
        shooter.setPower(power);
    }

    public void calculateShot(double[] target) {
        return;
    }
}