package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.sbs.bears.ftc.robot.Robot;
import org.sbs.bears.ftc.robot.controller.CamController;
import org.sbs.bears.ftc.robot.controller.RRDriveControllerNoOdomCam;

import javax.net.ssl.HostnameVerifier;

@Autonomous
@Config
public class Triangle265 extends OpMode {


    //SampleMecanumDriveNoOdom drive;
    Robot theRobot;
    RRDriveControllerNoOdomCam rrCtrl;
    CamController camCtrl;
    private boolean qA;
    private boolean qB;
    private boolean qX;
    private boolean qY;
    int status = 0;
    private int overallCounter;

    @Override
    public void init() {
        //theRobot = new Robot(hardwareMap,telemetry);
        telemetry = new MultipleTelemetry();
        telemetry.addData("Initialization","In Progress...");
        telemetry.update();
        rrCtrl = null;
        camCtrl = new CamController(hardwareMap);
        rrCtrl = new RRDriveControllerNoOdomCam(hardwareMap,telemetry,camCtrl.getCam());
        telemetry = new MultipleTelemetry(telemetry);
        msStuckDetectLoop = 50000000;
        camCtrl.startCam(rrCtrl);
        camCtrl.resetPosCamFromRR();
        overallCounter = 0;
        telemetry.addData("Initialization","Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("overallCounter",overallCounter);
        telemetry.addData("status",status);
        telemetry.update();
        camCtrl.setPosFromCam();
        overallCounter = overallCounter++;
        if(overallCounter > 9)
            requestOpModeStop();
        switch(status){
            case 0:
                rrCtrl.doLineToSpline(rrCtrl.getCurrentPos(),new Pose2d(48,10,-Math.PI/2));
                rrCtrl.drive.update();
                status = 1;
                break;
            case 1:
                rrCtrl.doLineToSpline(rrCtrl.getCurrentPos(),new Pose2d(24,-24,-Math.PI));
                rrCtrl.drive.update();
                status = 2;
                break;
            case 2:
                rrCtrl.doLineToSpline(rrCtrl.getCurrentPos(),new Pose2d(0,0,Math.PI/2));                rrCtrl.drive.update();
                rrCtrl.drive.update();
                status = 3;
                break;
            case 3:
                rrCtrl.turn(-Math.PI/2);
                rrCtrl.drive.update();
                try {
                    Thread.sleep(5000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                status = 0;
        }
    }

    @Override
    public void stop() {
        super.stop();
        camCtrl.stopCam();
    }
}
