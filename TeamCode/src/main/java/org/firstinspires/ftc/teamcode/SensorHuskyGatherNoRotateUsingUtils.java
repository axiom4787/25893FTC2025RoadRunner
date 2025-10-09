/*
Copyright (c) 2023 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.HuskyLensVision;

@TeleOp(name = "HL Gather Without Rotating Using Utils", group = "Auto")
public class SensorHuskyGatherNoRotateUsingUtils extends LinearOpMode {
    HuskyLensVision vision = new HuskyLensVision();
    MecanumDrive drive = new MecanumDrive();
    Intake intake = new Intake();

    @Override
    public void runOpMode() {
        vision.init(hardwareMap, HuskyLens.Algorithm.COLOR_RECOGNITION);
        drive.init(hardwareMap);
        intake.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block target_block = vision.getTargetBlock(1);

            intake.setIntake(0.0);
            if (target_block != null) {
                intake.setIntake(1.0);

                double direction = vision.calculateStrafe(target_block, 160f);

                double SPEED = 1f;
                double forwardDir = SPEED * vision.calculateForward(target_block, 160f);

                double back = -SPEED * direction - forwardDir;
                double front = SPEED * direction - forwardDir;

                // Divide each part of powers based off of the max value in powers if one of the powers is greater than 1
                double max = Math.max(Math.abs(back), Math.abs(front));
                if (max > 1) {
                    back /= max;
                    front /= max;
                }

                drive.setDrivePower(front, back, front, back);
            } else {
                drive.setDrivePower(0.0f, 0.0f, 0.0f, 0.0f);
            }
        }
    }
}