package org.sbs.bears.ftc.robot.controller;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.sbs.bears.ftc.robot.lib.RingDeterminationPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.sbs.bears.ftc.util.RobotSubsytemManager;

public class CameraController extends RobotSubsytemManager {


    static final int STREAM_WIDTH = 640;
    static final int STREAM_HEIGHT = 360;
    RingDeterminationPipeline pipeline;
    OpenCvWebcam webcam;
    WebcamName webcamName = null;
    public CameraController(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        webcamName = hardwareMap.get(WebcamName.class, "RightWebcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    @Override
    public void shutDown() {
        webcam.stopStreaming();
    }

    public int readNumRings() throws InterruptedException {
        int[] values = new int[5];
        int nSamples = 40;
        for(int k = 0; k < nSamples ; k++) {
            if(pipeline.getNumRings() >= 0) {
                values[pipeline.getNumRings()]++;
                Thread.sleep(50);
            }
            else {
                Thread.sleep(200);
            }
        }
        int numRings = 0;
        if(values[1] > values[numRings]) {
            return 1;
        }
        if(values[4] > values[numRings]) {
            return 4;
        }
        webcam.stopStreaming();
        return 0;
    }

}
