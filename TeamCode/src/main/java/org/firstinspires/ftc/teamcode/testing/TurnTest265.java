package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.sbs.bears.ftc.robot.Robot;
import org.sbs.bears.ftc.robot.controller.CamController;
import org.sbs.bears.ftc.robot.controller.RRDriveControllerNoOdomCam;
import org.sbs.bears.ftc.robot.controller.RingSubsytemController;
import org.sbs.bears.ftc.robot.lib.ShootingModes;

@TeleOp
@Config
public class TurnTest265 extends OpMode {
    public static double DIST = 8;
    public static double turnAng = 4;

    //SampleMecanumDriveNoOdom drive;
    Robot theRobot;
RRDriveControllerNoOdomCam rrCtrl;
    CamController camCtrl;
    private boolean qA;
    private boolean qB;
    private boolean qX;
    private boolean qY;
    int status = 0;
    private boolean isShooting;
    private RingSubsytemController ringCtrl;

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
        theRobot = new Robot(hardwareMap,telemetry);
        ringCtrl = theRobot.ringCtrl;
        ringCtrl.setAngleToTeleOpPSHOT();

    }

    @Override
    public void loop() {
        camCtrl.setPosFromCam();
        if(gamepad1.a && !qA)
        {
            qA = true;
            rrCtrl.doStrafeLeft(rrCtrl.getCurrentPos(),DIST);
        }
        if(!gamepad1.a && qA)
        {
            qA = false;
        }
        if(gamepad1.b && !qB)
        {
            qB = true;
            ringCtrl.setRingManagementMode(ShootingModes.SHOOTING);
            for(int i = 0; i < 3; i++) {
                rrCtrl.doStrafeLeft(rrCtrl.getCurrentPos(), DIST);
                ringCtrl.shootOneRing();
            }
            ringCtrl.shutDown();
        }
        if(!gamepad1.b && qB)
        {
            qB = false;
        }
        if(gamepad1.right_trigger > 0.3 && !qB)
        {
            qB = true;
            ringCtrl.setRingManagementMode(ShootingModes.SHOOTING);
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            for(int i = 0; i < 3; i++) {
                rrCtrl.turn(Math.toRadians(turnAng));
                ringCtrl.shootOneRing();
            }
            ringCtrl.shutDown();
        }
        if(!(gamepad1.right_trigger > 0.3) && qB)
        {
            qB = false;
        }

        rrCtrl.doGamepadDriving(gamepad1);


        if(gamepad1.x && !qX)
        {
            qX = true;
            ringCtrl.shootOneRing();
        }
        if(!gamepad1.x && qX)
        {
            qX = false;
        }
        if(gamepad1.y && !qY)
        {
            qY = true;
            isShooting = !isShooting;
        }
        if(!gamepad1.y && qY)
        {
            qY = false;
        }

        if(isShooting){
            ringCtrl.setRingManagementMode(ShootingModes.SHOOTING);
        }
        else {
            ringCtrl.setRingManagementMode(ShootingModes.IDLE);
        }
        rrCtrl.drive.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        camCtrl.stopCam();
    }
}
