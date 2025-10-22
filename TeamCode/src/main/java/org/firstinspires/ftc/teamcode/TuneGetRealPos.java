package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import org.firstinspires.ftc.teamcode.util.HuskyLensVision;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode.*;

@TeleOp (name="Tune Get Real Pos", group="Tuning")
public class TuneGetRealPos extends LinearOpMode {
    HuskyLensVision huskylens = new HuskyLensVision();

    @Override
    public void runOpMode() throws InterruptedException
    {
        huskylens.init(hardwareMap, HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.addData("Status", "Initialized");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            HuskyLens.Block block = huskylens.getTargetBlock(1);
            if (block != null) {
                telemetry.addData("X", block.x);
                telemetry.addData("Y", block.y);
                telemetry.addData("Width", block.width);
                telemetry.addData("Height", block.height);
                telemetry.addData("Break", "-------------------");
                double[] realPos = huskylens.getRealPos(block);
                telemetry.addData("Real X", realPos[0]);
                telemetry.addData("Real X", realPos[1]);
                telemetry.addData("Real X", realPos[2]);

                telemetry.update();
            }
        }
    }
}
