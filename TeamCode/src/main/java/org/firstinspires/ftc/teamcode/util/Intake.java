package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor intakeMotor;
    private HardwareMap hardwareMap;
    // Make sure to rename motor port 0 of expansion hub to "intake"

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        intakeMotor = this.hardwareMap.get(DcMotor.class, "intake");

        // Set intake motor directions.
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setIntake(double power) {
        intakeMotor.setPower(power);
    }
}
