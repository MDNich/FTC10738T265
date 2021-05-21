package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.sbs.bears.ftc.robot.Robot;
import org.sbs.bears.ftc.robot.controller.CamController;
import org.sbs.bears.ftc.robot.controller.RRDriveControllerNoOdom;
import org.sbs.bears.ftc.robot.controller.RRDriveControllerNoOdomCam;

@Autonomous
@Config
public class PathTest265 extends OpMode {


    //SampleMecanumDriveNoOdom drive;
    Robot theRobot;
RRDriveControllerNoOdomCam rrCtrl;
    CamController camCtrl;
    private boolean qA;
    private boolean qB;
    private boolean qX;
    private boolean qY;
    int status = 0;

    @Override
    public void init() {
        //theRobot = new Robot(hardwareMap,telemetry);
        rrCtrl = null;
        camCtrl = new CamController(hardwareMap);
        rrCtrl = new RRDriveControllerNoOdomCam(hardwareMap,telemetry,camCtrl.getCam());
        telemetry = new MultipleTelemetry(telemetry);
        msStuckDetectLoop = 50000000;
        camCtrl.startCam(rrCtrl);
        camCtrl.resetPosCamFromRR();

    }

    @Override
    public void loop() {
        camCtrl.setPosFromCam();
        switch(status){
            case 0:
                rrCtrl.doLineToSpline(rrCtrl.getCurrentPos(),new Pose2d(48,0,-Math.PI/2));
                status = 1;
                break;
            case 1:
                rrCtrl.doLineToSpline(rrCtrl.getCurrentPos(),new Pose2d(48,-48,-Math.PI));
                status = 2;
                break;
            case 2:
                rrCtrl.doLineToSpline(rrCtrl.getCurrentPos(),new Pose2d(24,-48,Math.PI/2));
                status = 3;
                break;
            case 3:
                rrCtrl.doLineToSpline(rrCtrl.getCurrentPos(),new Pose2d(24,-24,0));
                requestOpModeStop();
        }
    }

    @Override
    public void stop() {
        super.stop();
        camCtrl.stopCam();
    }
}
