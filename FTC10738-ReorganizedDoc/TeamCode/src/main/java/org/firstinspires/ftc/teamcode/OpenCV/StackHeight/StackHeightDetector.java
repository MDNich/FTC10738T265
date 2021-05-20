package org.firstinspires.ftc.teamcode.OpenCV.StackHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.BaseDetector;

public class StackHeightDetector extends BaseDetector {

    private StackHeightPipeline pipeline;

    public StackHeightDetector(LinearOpMode op) {
        super(op);

        pipeline = new StackHeightPipeline();
        setPipeline(pipeline);
    }

    public StackHeightPipeline.RingCase getResult() {
        return pipeline.getResult();
    }

    public StackHeightPipeline.RingCase getModeResult() {
        return pipeline.getModeResult();
    }

    public double[] getRawResult() {
        return pipeline.getRawResult();
    }
}