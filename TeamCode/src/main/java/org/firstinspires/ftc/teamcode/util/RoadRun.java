package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class RoadRun {
    MecanumDrive drive;
    HardwareMap hardwareMap;
    Pose2d beginPose;

    public void init(HardwareMap HWM, Pose2d beginPos) {
        this.hardwareMap = HWM;
        this.beginPose = beginPos;
        this.drive = new MecanumDrive(hardwareMap, beginPose);
    }

    public void update(){
        this.drive.updatePoseEstimate();
        this.drive.localizer.update();
    }

    public boolean runTo(Pose2d pos, double tangent){
        if (pos != this.drive.localizer.getPose()) {
            Actions.runBlocking(
                    this.drive.actionBuilder(beginPose)
                            .splineTo(posToVec(pos), tangent)
                            .build());
            return false;
        } else {
            return true;
        }
    }

    public Pose2d getPos() {
        return this.drive.localizer.getPose();
    }

    private Vector2d posToVec(Pose2d pos) {
        return new Vector2d(pos.position.x,pos.position.y);
    }

}
