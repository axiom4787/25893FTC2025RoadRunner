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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.Comparator;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.stream.Stream;

/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 *
 * For detailed instructions on how a HuskyLens is used in FTC, please see this tutorial:
 * https://ftc-docs.firstinspires.org/en/latest/devices/huskylens/huskylens.html
 * 
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Aristocratic Ball Knowledge", group = "Auto")
public class AristocraticBallKnowledge extends LinearOpMode {

    private Limelight3A limelight;

    private final float TURN_SPEED = 0.25f;

    double LimeLightFrontPower;
    double LimeLightBackPower;

    final int READ_PERIOD = 10; // ms

    private double direction;

    HuskyLens huskyLens;

    PIDController turnController = new PIDController(0.075, 0, 0.025, -15, 15);
    PIDController strafePID = new PIDController(1.35, 0, 0.12, -1, 1);

    double SPEED = 1f;
    double forward_dir;

    @Override
    public void runOpMode() throws InterruptedException
    {
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

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

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         *
         * Other algorithm choices for FTC might be: OBJECT_RECOGNITION, COLOR_RECOGNITION or OBJECT_CLASSIFICATION.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        // Persistent telemetry fields
        String lastBlockData = "None";
        int lastBlockId = -1;

        telemetry.update();
        waitForStart();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            /*/ Huskeylens /*/
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            int target_id = 1;

            HuskyLens.Block[] blocks = huskyLens.blocks();

            HuskyLens.Block target_block = Stream.of(blocks)
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
                /*
                 * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
                 * - blocks[i].width and blocks[i].height   (size of box, in pixels)
                 * - blocks[i].left and blocks[i].top       (edges of box)
                 * - blocks[i].x and blocks[i].y            (center location)
                 * - blocks[i].id                           (Color ID)
                 * These values have Java type int (integer).
                 */
            }

            // Always show last seen block data n' ID
            telemetry.addData("Last Block", lastBlockData);
            telemetry.addData("Last Block ID", lastBlockId);
            telemetry.update();

            LimeLightBackPower = 0.0;
            LimeLightFrontPower = 0.0;
            direction = 0.0;
            forward_dir = 0.0;

            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {

                double tx = result.getTx();

                double turn = turnController.calculate(0f, tx);

                LimeLightBackPower = -TURN_SPEED * turn;
                LimeLightFrontPower = TURN_SPEED * turn;
            }

            if (blocks.length > 0 && target_block != null) {
                direction = strafePID.calculate(0f, ((target_block.x - 160f) / 160f));
                forward_dir = SPEED * (1f - (target_block.width / 170f));
                HuskyLens.Block lastBlock = blocks[blocks.length - 1];
                lastBlockData = lastBlock.toString();
                lastBlockId = lastBlock.id;
            }

            double[] powers = {((-SPEED * direction) - forward_dir),((SPEED * direction) - forward_dir)};
            // Divide each part of powers based off of the max value in powers if one of the powers is greater than 1
            double max = Math.max(Math.abs(powers[0]), Math.abs(powers[1]));
            if (max > 1) {
                powers[0] /= max;
                powers[1] /= max;
            }

            if (target_block != null) {
                if (target_block.width > 160f) {
                    powers[0] = 0;
                    powers[1] = 0;
                }
            }

            leftBackDrive.setPower(powers[0] + LimeLightBackPower); // + forward_dir);
            rightBackDrive.setPower(powers[0] + LimeLightFrontPower); // - forward_dir);
            leftFrontDrive.setPower(powers[1] + LimeLightBackPower); // + forward_dir);
            rightFrontDrive.setPower(powers[1] + LimeLightFrontPower); // - forward_dir);

            telemetry.update();
        }
        limelight.stop();
    }
}