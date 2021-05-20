package org.sbs.bears.ftc.robot.controller;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.ftc.util.RobotSubsytemManager;

/**@deprecated */
public class T265Controller extends RobotSubsytemManager {
    static T265Camera intelCam;
    Transform2d camToRobot;
    private double whichX;
    private double dX;
    private double whichY;
    private double dY;
    private double whichH;
    private double dH;
    public T265Controller(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
        camToRobot = new Transform2d(new Translation2d(2.5,-8.5),new Rotation2d());
        intelCam = new T265Camera(camToRobot,0.1,hwMap.appContext);
        try {intelCam.start();}
        catch (Exception e)
        {
            e.printStackTrace();
            intelCam.stop();
            telemetry.addData("You forgot to stop","the camera!!");
            telemetry.update();
            intelCam.start();
        }
    }
    public Pose2d convert265PosToRR(com.arcrobotics.ftclib.geometry.Pose2d input)
    {
        return new Pose2d(input.getTranslation().getX(),input.getTranslation().getY(),input.getHeading());
    }
    public com.arcrobotics.ftclib.geometry.Pose2d reverseConvert265PosToRR(Pose2d input)
    {
        return new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(input.getX(),input.getY()),new Rotation2d(input.getHeading()));
    }
    public Pose2d getIntelPos()
    {
        double x = intelCam.getLastReceivedCameraUpdate().pose.getTranslation().getX() / 0.0254;
        double y = intelCam.getLastReceivedCameraUpdate().pose.getTranslation().getY() / 0.0254;
        double h = intelCam.getLastReceivedCameraUpdate().pose.getHeading();
        return new Pose2d(x,y,h);
    }

    @Override
    public void shutDown() {
        intelCam.stop();
    }
}