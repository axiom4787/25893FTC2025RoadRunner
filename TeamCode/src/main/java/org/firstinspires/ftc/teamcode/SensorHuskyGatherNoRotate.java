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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.Comparator;
import java.util.concurrent.TimeUnit;
import java.util.stream.Stream;

@TeleOp(name = "HL Gather Without Rotating", group = "Auto")
public class SensorHuskyGatherNoRotate extends LinearOpMode {
    boolean run_intake = false;
    final int READ_PERIOD = 10; // ms

    private double direction;

    double stopAtWidth = 160f;

    HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        PIDController strafePID = new PIDController(1.35, 0, 0.12, -1, 1);

        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DcMotor intakeFront  = hardwareMap.get(DcMotor.class, "intakeFront");
        DcMotor intakeBack = hardwareMap.get(DcMotor.class, "intakeBack");

        // Set intake motor directions.
        intakeFront.setDirection(DcMotor.Direction.REVERSE);
        intakeBack.setDirection(DcMotor.Direction.FORWARD);

        // Motor Powers
        double frontPower = 1.0f;
        double backPower = 1.0f;

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        huskyLens = hardwareMap.get(HuskyLens.class, "eyeball");

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.MILLISECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        // Persistent telemetry fields
        String lastBlockData = "None";
        int lastBlockId = -1;

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            int target_id = 1;

            HuskyLens.Block[] blocks = huskyLens.blocks();

            HuskyLens.Block targetBlock = Stream.of(blocks)
                    .filter(b -> b.id == target_id)
                    .max(Comparator.comparingInt(b -> b.width * b.height))
                    .orElse(null);
            telemetry.addData("Block", lastBlockData);
            telemetry.addData("Block ID", lastBlockId);
            telemetry.addData("Block count", blocks.length);
            telemetry.addData("Direction", direction);
            for (HuskyLens.Block block : blocks) {
                lastBlockData = block.toString();
                lastBlockId = block.id;
            }

            if (run_intake) {
                intakeFront.setPower(frontPower);
                intakeBack.setPower(backPower);
            } else {
                intakeFront.setPower(0.0f);
                intakeBack.setPower(0.0f);
            }

            if (blocks.length > 0 && targetBlock != null) {
                run_intake = true;

                direction = strafePID.calculate(0f, ((targetBlock.x - stopAtWidth) / stopAtWidth));
                double SPEED = 1f;
                double forward_dir = SPEED * (1f - (targetBlock.width / 170f));
                HuskyLens.Block lastBlock = blocks[blocks.length - 1];
                lastBlockData = lastBlock.toString();
                lastBlockId = lastBlock.id;

                // ir)};

                double[] powers = {((-SPEED * direction) - forward_dir),((SPEED * direction) - forward_dir)};
                // Divide each part of powers based off of the max value in powers if one of the powers is greater than 1
                double max = Math.max(Math.abs(powers[0]), Math.abs(powers[1]));
                if (max > 1) {
                    powers[0] /= max;
                    powers[1] /= max;
                }

                if (targetBlock.width > stopAtWidth) {
                    powers[0] = 0;
                    powers[1] = 0;
                }

                leftBackDrive.setPower(powers[0]); // + forward_dir);
                rightBackDrive.setPower(powers[0]); // - forward_dir);
                leftFrontDrive.setPower(powers[1]); // + forward_dir);
                rightFrontDrive.setPower(powers[1]); // - forward_dir);
            } else {
                //run_intake = false;

                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
            }
            // Always show last seen block data n' ID
            telemetry.addData("Last Block", lastBlockData);
            telemetry.addData("Last Block ID", lastBlockId);
            telemetry.update();

        }
    }
}