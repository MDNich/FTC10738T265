package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveOnlyCam1;
import org.sbs.bears.ftc.robot.controller.CamController0;
import org.sbs.bears.ftc.robot.controller.RRDriveControllerCamOnly;

@TeleOp
@Config
public class LocalizationTest265New extends OpMode {

    public static double DIST = 8;

    //SampleMecanumDriveNoOdom drive;
    //Robot theRobot;
    SampleMecanumDriveOnlyCam1 drive;
    CamController0 camCtrl;
    private boolean qA;
    private boolean qB;
    private boolean qX;
    private boolean qY;
    private T265Camera intelCam;

    @Override
    public void init() {
        //theRobot = new Robot(hardwareMap,telemetry);
        //rrCtrl = null;
        camCtrl = new CamController0(hardwareMap);
        //rrCtrl = new RRDriveControllerCamOnly(hardwareMap,telemetry,camCtrl.getCam());
        intelCam = camCtrl.getCam();
        drive = new SampleMecanumDriveOnlyCam1(hardwareMap,false,intelCam);
        telemetry = new MultipleTelemetry(telemetry);
        msStuckDetectLoop = 50000000;
        camCtrl.startCam(null);
        //camCtrl.resetPosCamFromRR();
    }

    @Override
    public void loop() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );
        //camCtrl.setPosFromCam();
        drive.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        camCtrl.stopCam();
    }
}
