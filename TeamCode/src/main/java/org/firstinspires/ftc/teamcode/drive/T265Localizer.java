package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.spartronics4915.lib.T265Camera;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.sbs.bears.ftc.util.Beta;

/**
 * Localizer based on T265 Camera.
 */
@Beta
public class T265Localizer implements Localizer {
    private Pose2d currentPose;
    private Pose2d currentPoseVel;
    private T265Camera intelCam;
    private double x;
    private double y;
    private double h;
    private double xvel;
    private double yvel;
    private double hvel;
    private T265Camera.CameraUpdate camData;

    public T265Localizer(T265Camera cam)
    {
        this.intelCam = cam;
        try {
        camData = intelCam.getLastReceivedCameraUpdate();}
        catch (NullPointerException e)
        {
            System.out.println("cam not init yet kid");
        }
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return currentPose;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        try {
            intelCam.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(pose2d.getX(), pose2d.getY()), new Rotation2d(pose2d.getHeading())));
        }
        catch (NullPointerException e)
        {
            System.out.println("cam not init yet kid: trying to set pose");
        }
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return currentPoseVel;
    }

    @Override
    public void update() {
        try {
        camData = intelCam.getLastReceivedCameraUpdate();
        }
        catch (NullPointerException e)
        {
            System.out.println("cam not init yet kid: trying to get update");
        }
        x = camData.pose.getTranslation().getX() / 0.0254;
        y = camData.pose.getTranslation().getY() / 0.0254;
        h = camData.pose.getRotation().getRadians();
        xvel = camData.velocity.vxMetersPerSecond / 0.0254;
        yvel = camData.velocity.vyMetersPerSecond / 0.0254;
        hvel = camData.velocity.omegaRadiansPerSecond;
        currentPose = new Pose2d(x, y, h);
        currentPoseVel = new Pose2d(xvel,yvel,hvel);
    }
}
