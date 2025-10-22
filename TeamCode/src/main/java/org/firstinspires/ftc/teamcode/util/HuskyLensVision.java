package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Comparator;
import java.util.stream.Stream;

import com.qualcomm.hardware.dfrobot.HuskyLens.Block;

import  com.qualcomm.robotcore.*;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

public class HuskyLensVision {

    private double direction;

    HuskyLens huskyLens;
    PIDController strafePID;
    PIDController turnController;
    private HardwareMap hardwareMap;
    IMU imu;
    AngularVelocity myRobotAngularVelocity;

    public void init(HardwareMap hardwareMap, HuskyLens.Algorithm recognitionMode) {
        this.hardwareMap = hardwareMap;
        huskyLens = this.hardwareMap.get(HuskyLens.class, "eyeball");

        strafePID = new PIDController(1.35, 0, 0.12, -1, 1);
        turnController = new PIDController(0.075, 0, 0.025, -15, 15); // TODO: Tune please!

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParameters);

        huskyLens.selectAlgorithm(recognitionMode);
    }

    public Block getTargetBlock(int targetId) {
        Block[] blocks = huskyLens.blocks();

        Block target_block = Stream.of(blocks)
                .filter(b -> b.id == targetId)
                .max(Comparator.comparingInt(b -> b.width * b.height))
                .orElse(null);

        return target_block;
    }


    // This one attempts to denoise and correct for errors in the target block
    private int persist = 0;
    private Block lastBlock;
    public Block getTargetBlockAdvanced(int targetId, int centerPersistence, int edgePersistence) { // Good values for getTargetBlockAdvanced are (1, 6, 3), making sure the centerPersistence is higher than the edgePersistence is larger than edgePersistence

        // Blocks that we just saw near the center are more likely to be there for the next cycle, but if they are near the edge they may have moved out of view.
        // The point of this equation is to interpolate between the center persistence and edge persistence based on where the last block was
        int persistence = Math.round(centerPersistence + (edgePersistence - centerPersistence) * Math.abs((lastBlock.x - 160f) / 160f)); // Essentially lerp(centerPersistence, edgePersistence, abs((lastBlock.x - 160f) / 160f)). The 160f is half the max width value

        Block[] blocks = huskyLens.blocks();

        Block targetBlock = Stream.of(blocks)
                .filter(b -> b.id == targetId)
                .max(Comparator.comparingInt(b -> b.width * b.height))
                .orElse(null);

        if (targetBlock != null) {
            lastBlock = targetBlock;
            persist = 0;
        } else {
            persist++;

            if (persist < persistence) {
                targetBlock = lastBlock;
            }
        }

        return targetBlock;
    }

    public double calculateStrafe(Block targetBlock, double center) { // Pass in targetBlock and the center of the view, by default it is 160f
        return strafePID.calculate(0f, ((targetBlock.x - center) / center));
    }

    public double calculateForward(Block targetBlock, double stopAtWidth) { // Pass in targetBlock and a stopAtWidth of 160f or 170f
        if (targetBlock.width > stopAtWidth) {
            return 0.0f;
        }
        return (1f - (targetBlock.width / stopAtWidth));
    }

    public double calculateRotation(Block targetBlock, double center) { // Pass in targetBlock and the center of the view, by default it is 160f
        return turnController.calculate(0f, ((targetBlock.x - center) / center));
    }

    // Convert a targetBlock into approx world coordinates relative to the view
    public double[] getRealPos(Block targetBlock, double REAL_WIDTH) {
        double HALFCAMERAWIDTH = 320f / 2f;
        double HALFCAMERAHEIGHT = 180f / 2f;

        // Should be how much you need to multiply the distance calculation by to be accurate
        // You can find by running `TuneGetRealPos` and finding how much you need to multiply by to get the actual value, then multiply whichever of these you are tuning by that value to get your new value. These are the default values that produce +- 0.5 in of accuracy
        double ZSCALAR = 333.333; // How much you need to multiply the z distance calculation by to be accurate
        double XSCALAR = 0.555; // Same as above but for x distance
        double YSCALAR = 0.555; // """

        double targetBlockScale = Math.max(targetBlock.width, targetBlock.height); // our tag could be rotated, so we will take the largest of these to prevent errors

        double zDistance = (REAL_WIDTH / targetBlockScale) * ZSCALAR; // distance away in inches

        double xDistance = ((targetBlock.x - HALFCAMERAWIDTH) / HALFCAMERAWIDTH) * zDistance * XSCALAR; // the distance from the center in inches locally
        double yDistance = ((HALFCAMERAHEIGHT - targetBlock.y) / HALFCAMERAHEIGHT) * zDistance * YSCALAR;

        return new double[] {xDistance, yDistance, zDistance}; // Local x, y, and z, all in inches relative to the view
    }
}