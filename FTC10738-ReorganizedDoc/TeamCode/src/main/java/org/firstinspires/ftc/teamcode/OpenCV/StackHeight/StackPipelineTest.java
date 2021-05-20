package org.firstinspires.ftc.teamcode.OpenCV.StackHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Stack Height Pipeline Test")
public class StackPipelineTest extends LinearOpMode {
    private StackHeightDetector detector;

    @Override
    public void runOpMode() {
        detector = new StackHeightDetector(this);
        detector.start();
        telemetry.addLine("Pending Start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", detector.getFrameCount());
            telemetry.addData("FPS", detector.getFPS());
            telemetry.addData("W/H Ratio", detector.getRawResult()[2]);
            telemetry.addData("Result", detector.getResult());
            telemetry.addData("Mode Result", detector.getModeResult());
            telemetry.update();
        }

        detector.stop();
    }
}