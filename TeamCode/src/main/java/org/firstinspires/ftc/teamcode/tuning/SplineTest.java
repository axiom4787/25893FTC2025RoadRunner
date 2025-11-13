package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.util.RoadRun;

public final class SplineTest extends LinearOpMode {
    Pose2d beginPose = new Pose2d(0, 0, 0);
    RoadRun roadRun = new RoadRun();
    @Override
    public void runOpMode() throws InterruptedException {

        roadRun.init(hardwareMap, beginPose);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {

            waitForStart();

            while (opModeIsActive()) {
                if (roadRun.runTo(new Pose2d(114.0, 6.0, 0.0), 0.0)){
                    break;
                } else {
                    continue;
                }

            }


        } else {
            throw new RuntimeException();
        }
    }
}
