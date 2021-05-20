package org.sbs.bears.ftc.robot.lib;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingDeterminationPipeline extends OpenCvPipeline {
    static final int STREAM_WIDTH = 640;
    static final int STREAM_HEIGHT = 360;

    /*
     * An enum to define the ring amount
     */
    public enum RingAmount
    {
        FOUR,
        ONE,
        NONE
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255,0,0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final int REGION_WIDTH_RING = 130; // for ini rings window
    static final int REGION_HEIGHT_RING = 110; // for ini rings window
    static final Point REGION1_TOPLEFT_ANCHOR_POINT_RING = new Point((STREAM_WIDTH - REGION_WIDTH_RING) / 2+50, ((STREAM_HEIGHT - REGION_HEIGHT_RING) / 2) + 100);

    static final int REGION_WIDTH_GOAL = 165; // for goal alignment window
    static final int REGION_HEIGHT_GOAL = 60; // for goal alignment window
    static final Point REGION1_TOPLEFT_ANCHOR_POINT_GOAL = new Point((STREAM_WIDTH - REGION_WIDTH_GOAL) / 2 + 0, 0);


    final int FOUR_RING_THRESHOLD = 137;
    final int ONE_RING_THRESHOLD = 130;

    Point region1_pointA_ring = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT_RING.x,
            REGION1_TOPLEFT_ANCHOR_POINT_RING.y);
    Point region1_pointB_ring = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT_RING.x + REGION_WIDTH_RING,
            REGION1_TOPLEFT_ANCHOR_POINT_RING.y + REGION_HEIGHT_RING);

    Point region1_pointA_goal = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT_GOAL.x,
            REGION1_TOPLEFT_ANCHOR_POINT_GOAL.y);
    Point region1_pointB_goal = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT_GOAL.x + REGION_WIDTH_GOAL,
            REGION1_TOPLEFT_ANCHOR_POINT_GOAL.y + REGION_HEIGHT_GOAL);

    /*
     * Working variables
     */
    Mat region1_Cb;
    Mat regionGoal_Cb;
    //Mat regionGoal_Cr;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    ///Mat Cr = new Mat();
    int avg1;
    int avgGoal;
    //int avgGoalCr;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile RingDeterminationPipeline.RingAmount position = RingDeterminationPipeline.RingAmount.NONE;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame)
    {
        inputToCb(firstFrame);

        region1_Cb = Cb.submat(new Rect(region1_pointA_ring, region1_pointB_ring));
        regionGoal_Cb = Cb.submat(new Rect(region1_pointA_goal, region1_pointB_goal));
        /////regionGoal_Cr = Cr.submat(new Rect(region1_pointA_goal, region1_pointB_goal));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];
        avgGoal = (int) Core.mean(regionGoal_Cb).val[0];
        //avgGoalCr = (int) Core.mean(regionGoal_Cr).val[0]; // need to fix val[0]



        position = RingDeterminationPipeline.RingAmount.NONE; // Record our analysis
        if (avg1 >= FOUR_RING_THRESHOLD) {
            position = RingDeterminationPipeline.RingAmount.FOUR;
        } else if (avg1 >= ONE_RING_THRESHOLD) {
            position = RingDeterminationPipeline.RingAmount.ONE;
        } else {
            position = RingDeterminationPipeline.RingAmount.NONE;
        }

        Imgproc.rectangle( // rings
                input, // Buffer to draw on
                region1_pointA_ring, // First point which defines the rectangle
                region1_pointB_ring, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                1); // Thickness of the rectangle lines


        Imgproc.rectangle( // goal
                input, // Buffer to draw on
                region1_pointA_goal, // First point which defines the rectangle
                region1_pointB_goal, // Second point which defines the rectangle
                RED, // The color the rectangle is drawn in
                1); // Negative thickness means solid fill

        return input;
    }

    public int getAnalysis()
    {
        return avg1;
    }
    public int getGoalAnalysis()
    {
        return avgGoal;
    }
    public int getNumRings()
    {
        switch(position)
        {
            case ONE:
                return 1;
            case FOUR:
                return 4;
            case NONE:
                return 0;
            default:
                return -1;
        }
    }
}
