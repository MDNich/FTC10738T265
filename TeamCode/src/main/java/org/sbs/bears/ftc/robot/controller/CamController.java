package org.sbs.bears.ftc.robot.controller;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveNoOdom;
import org.firstinspires.ftc.teamcode.util.Sleep;

public class CamController {
    SampleMecanumDriveNoOdom drive;
    final int robotRadius = 9; // inches
    private static T265Camera slamra = null;
    private double whichX;
    private double dX;
    private double whichY;
    private double dY;
    private double whichH;
    private double dH;

    public CamController(RRDriveControllerNoOdom rrDriveControllerNoOdom, HardwareMap hwMap)
    {
        this.drive = rrDriveControllerNoOdom.drive;
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(new Translation2d(2.5,-8.5),new Rotation2d()), 0.1, hwMap.appContext);
        }

    }
    public T265Camera getCam()
    {
        return this.slamra;
    }
    public void startCam()
    {
        try {
            slamra.start();}
        catch (Exception e)
        {
            slamra.stop();
            Sleep.sleep(2);
            slamra.start();
        }
        whichX = drive.getPoseEstimate().getX();
        whichY = drive.getPoseEstimate().getY();
        whichH = drive.getPoseEstimate().getHeading();
        dX = (slamra.getLastReceivedCameraUpdate().pose.getTranslation().getX() / 0.0254);
        dY = (slamra.getLastReceivedCameraUpdate().pose.getTranslation().getY() / 0.0254);
        dH = (slamra.getLastReceivedCameraUpdate().pose.getHeading());
    }
    public void setPosFromCam()
    {
        getCamPos();
        drive.setPoseEstimate(new Pose2d(whichX,whichY, whichH));
    }
    public void resetPosCamFromRR() {
        dX = whichX + drive.getPoseEstimate().getX();
        dY = whichY + drive.getPoseEstimate().getY();
        dH = whichH + drive.getPoseEstimate().getHeading();
    }
    public void getCamPos()
    {
        T265Camera.CameraUpdate depthCamUpdate = slamra.getLastReceivedCameraUpdate();
        whichX = slamra.getLastReceivedCameraUpdate().pose.getTranslation().getX() / 0.0254 - dX;
        whichY = slamra.getLastReceivedCameraUpdate().pose.getTranslation().getY() / 0.0254 - dY;
        whichH = slamra.getLastReceivedCameraUpdate().pose.getHeading() - dH;
    }
    public void stopCam()
    {
        slamra.stop();
    }

}
