package org.sbs.bears.ftc.robot.controller;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveNoOdomCam;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveOnlyCam;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Sleep;

@Config
public class CamController0 {

    public static double X_Transform = 0;
    public static double Y_Transform = 0;

    private FtcDashboard dashboard;

    //SampleMecanumDriveOnlyCam drive;
    final int robotRadius = 9; // inches
    private static T265Camera slamra = null;
    private double whichX;
    private double dX;
    private double whichY;
    private double dY;
    private double whichH;
    private double dH;

    public CamController0(HardwareMap hwMap)
    {

        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(new Translation2d(X_Transform,Y_Transform),new Rotation2d()), 0.1, hwMap.appContext);
        }
        slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(0,0,new Rotation2d()));
        dashboard = FtcDashboard.getInstance();

    }
    public T265Camera getCam()
    {
        return this.slamra;
    }
    public void startCam(RRDriveControllerCamOnly rrDriveControllerCamOnly)
    {
        //this.drive = rrDriveControllerCamOnly.drive;
        try {
            slamra.start();}
        catch (Exception e)
        {
            slamra.stop();
            Sleep.sleep(2);
            slamra.start();
        }
        whichX = 0;
        whichY = 0;
        whichH = 0;
        dX = (slamra.getLastReceivedCameraUpdate().pose.getTranslation().getX() / 0.0254);
        dY = (slamra.getLastReceivedCameraUpdate().pose.getTranslation().getY() / 0.0254);
        dH = (slamra.getLastReceivedCameraUpdate().pose.getHeading());
    }
    public void setPosFromCam()
    {
        getCamPos();
        //drive.setPoseEstimate(new Pose2d(whichX,whichY, whichH));
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStroke("#FF00000");
        DashboardUtil.drawRobot(fieldOverlay, new Pose2d(whichX,whichY, whichH));

        dashboard.sendTelemetryPacket(packet);

    }
    public void resetPosCamFromRR() {
        dX = whichX;
        dY = whichY;
        dH = whichH;
    }
    public void getCamPos()
    {
        T265Camera.CameraUpdate depthCamUpdate = slamra.getLastReceivedCameraUpdate();
        whichX = depthCamUpdate.pose.getTranslation().getX() / 0.0254 - dX;
        whichY = depthCamUpdate.pose.getTranslation().getY() / 0.0254 - dY;
        whichH = depthCamUpdate.pose.getHeading() - dH;
    }
    public void stopCam()
    {
        slamra.stop();
    }





}
