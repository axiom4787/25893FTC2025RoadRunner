package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Comparator;
import java.util.stream.Stream;

import com.qualcomm.hardware.dfrobot.HuskyLens.Block;

import  com.qualcomm.robotcore.*;

public class HuskyLensVision {

    private double direction;

    HuskyLens huskyLens;
    PIDController strafePID;
    PIDController turnController;
    private HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap, HuskyLens.Algorithm recognitionMode) {
        this.hardwareMap = hardwareMap;
        huskyLens = this.hardwareMap.get(HuskyLens.class, "eyeball");

        strafePID = new PIDController(1.35, 0, 0.12, -1, 1);
        turnController = new PIDController(0.075, 0, 0.025, -15, 15); // TODO: Tune please!

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
}