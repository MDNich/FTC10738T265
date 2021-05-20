package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveNoOdom;
import org.sbs.bears.ftc.robot.Robot;
import org.sbs.bears.ftc.robot.controller.RRDriveControllerNoOdom;
import org.sbs.bears.ftc.robot.controller.RingSubsytemController;
import org.sbs.bears.ftc.robot.lib.ShootingModes;

@TeleOp
@Config
public class PathTestingNewArch extends OpMode {

    public static double DIST = 8;

    //SampleMecanumDriveNoOdom drive;
    Robot theRobot;
    RRDriveControllerNoOdom rrCtrl;
    //LaunchController launchCtrl;
    //BaseServoController servoCtrl;
    RingSubsytemController ringCtrl;
    private boolean qA;
    private boolean qB;
    private boolean qX;
    private boolean qY;
    private boolean isShooting;


    @Override
    public void init() {
        theRobot = new Robot(hardwareMap,telemetry);
        rrCtrl = theRobot.rrCtrlNoOdom;
        ringCtrl = theRobot.ringCtrl;
        telemetry = new MultipleTelemetry(telemetry);
        msStuckDetectLoop = 50000000;
    }

    @Override
    public void loop() {
        Pose2d myPose = rrCtrl.getCurrentPos();
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

        rrCtrl.drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

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
}
