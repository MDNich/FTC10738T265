package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.StackHeight.StackHeightPipeline;

public class Vision extends BaseDetector {

    private StackHeightPipeline stackHeightPipeline;

    public enum Pipeline {StackHeight, RingLocator}

    public Vision(LinearOpMode op) {
        super(op);

        stackHeightPipeline = new StackHeightPipeline();
    }

    public Vision(LinearOpMode op, Pipeline pipeline) {
        super(op);

        stackHeightPipeline = new StackHeightPipeline();
        setPipeline(pipeline);
    }

    public void setPipeline(Pipeline pipeline) {
        if (pipeline == Pipeline.StackHeight) {
            setPipeline(stackHeightPipeline);
        }
    }

    public StackHeightPipeline getStackPipe() {
        return stackHeightPipeline;
    }
}
