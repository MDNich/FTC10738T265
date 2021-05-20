package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.sbs.bears.ftc.robot.Robot;
import org.sbs.bears.ftc.robot.controller.CamController;
import org.sbs.bears.ftc.robot.controller.RRDriveControllerNoOdom;
import org.sbs.bears.ftc.robot.controller.RingSubsytemController;
import org.sbs.bears.ftc.robot.lib.ShootingModes;

@TeleOp
@Config
public class LocalizationTest265 extends OpMode {

    public static double DIST = 8;

    //SampleMecanumDriveNoOdom drive;
    Robot theRobot;
    RRDriveControllerNoOdom rrCtrl;
    CamController camCtrl;
    private boolean qA;
    private boolean qB;
    private boolean qX;
    private boolean qY;


    @Override
    public void init() {
        theRobot = new Robot(hardwareMap,telemetry);
        rrCtrl = theRobot.rrCtrlNoOdom;
        camCtrl = new CamController(rrCtrl,hardwareMap);
        telemetry = new MultipleTelemetry(telemetry);
        msStuckDetectLoop = 50000000;
        camCtrl.startCam();
        camCtrl.resetPosCamFromRR();

    }

    @Override
    public void loop() {
        rrCtrl.doGamepadDriving(gamepad1);
        camCtrl.setPosFromCam();
        rrCtrl.drive.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        camCtrl.stopCam();
    }
}