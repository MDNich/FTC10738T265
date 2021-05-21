package org.sbs.bears.ftc.robot.controller;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveNoOdom;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveNoOdomCam;
import org.sbs.bears.ftc.util.RobotSubsytemManager;

import java.util.Arrays;

public class RRDriveControllerNoOdomCam extends RobotSubsytemManager {
    public SampleMecanumDriveNoOdomCam drive;
    Telemetry telemetry;
    HardwareMap hwMap;


    public RRDriveControllerNoOdomCam(HardwareMap hwMap, Telemetry telemetry, T265Camera intelCam)
    {
        super(hwMap,telemetry);
        drive = new SampleMecanumDriveNoOdomCam(hwMap,false, intelCam);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void shutDown() {
        // nothing.
    }

    public void turn(double rad)
    {
        double iniAngle = drive.getPoseEstimate().getHeading();
        if(rad == 0) return;
        else if(rad > 0) { // turn left
            while(drive.getPoseEstimate().getHeading() < iniAngle + rad)
            {
                drive.setWeightedDrivePower(new Pose2d(0,0,-0.3));
                drive.update();
            }
        }
        else
        {
            while(drive.getPoseEstimate().getHeading() > iniAngle + rad)
            {
                drive.setWeightedDrivePower(new Pose2d(0,0,0.3));
                drive.update();
            }
        }
        drive.setWeightedDrivePower(new Pose2d());
    }

    private Vector2d pose2Vector(Pose2d input)
    {
        return new Vector2d(input.getX(),input.getY());
    }
    private Pose2d vector2Pose(Vector2d input)
    {
        return new Pose2d(input.getX(),input.getY());
    }


    public Pose2d getCurrentPos()
    {
        return drive.getPoseEstimate();
    }
    public void setCurrentPos(Pose2d pos)
    {
        drive.setPoseEstimate(pos);
    }

    public double calculateAngleToFace(Vector2d start, Vector2d end)
    {
        double dy = end.getY() - start.getY();
        double dx = end.getX() - start.getX();
        double ang = dy/dx; // slope
        /*
        dx
        ____
     dy |  /
        | /
        |/


        */
        return ang;
    }
    public double calculateAngleToFace(Vector2d end)
    {
        return calculateAngleToFace(getCurrentPos(), vector2Pose(end));
    }
    public double calculateAngleToFace(Pose2d start, Pose2d end)
    {
        return calculateAngleToFace(pose2Vector(start), pose2Vector(end));
    }



    public void doGamepadDriving(Gamepad gamepad1)
    {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();
    }




















    public void doLineToSpline(Pose2d start, Pose2d end) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .lineToSplineHeading(end)
                .build());
        drive.update();
    }
    public void doSplineToSpline(Pose2d start, Pose2d end) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .splineToSplineHeading(end,end.getHeading())
                .build());
        drive.update();
    }
    public void doSplineToConstant(Pose2d start, Pose2d end) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .splineToConstantHeading(pose2Vector(end), end.getHeading())
                .build());
        drive.update();
    }
    public void doForward(Pose2d start, double dist) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .forward(dist)
                .build());
        drive.update();
    }
    public void doBackward(Pose2d start, double dist) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .back(dist)
                .build());
        drive.update();
    }
    public void doStrafeRight(Pose2d start, double dist) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .strafeRight(dist)
                .build());
        drive.update();
    }
    public void doStrafeLeft(Pose2d start, double dist) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .strafeLeft(dist)
                .build());
        drive.update();
    }



    // overloads
    public void doLineToSpline(Pose2d start, Pose2d end, double maxVel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .lineToSplineHeading(end, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build());
        drive.update();
    }
    public void doSplineToSpline(Pose2d start, Pose2d end, double maxVel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .splineToSplineHeading(end,end.getHeading(), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build());
        drive.update();
    }
    public void doSplineToConstant(Pose2d start, Pose2d end, double maxVel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .splineToConstantHeading(pose2Vector(end), end.getHeading(), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build());
        drive.update();
    }
    public void doForward(Pose2d start, double dist, double maxVel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .forward(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build());
        drive.update();
    }
    public void doBackward(Pose2d start, double dist, double maxVel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .back(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build());
        drive.update();
    }
    public void doStrafeRight(Pose2d start, double dist, double maxVel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .strafeRight(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build());
        drive.update();
    }
    public void doStrafeLeft(Pose2d start, double dist, double maxVel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .strafeLeft(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build());
        drive.update();
    }

    // overloads
    public void doLineToSpline(Pose2d start, Pose2d end, double maxVel, double maxAccel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .lineToSplineHeading(end, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(maxAccel))
                .build());
        drive.update();
    }
    public void doSplineToSpline(Pose2d start, Pose2d end, double maxVel, double maxAccel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .splineToSplineHeading(end,end.getHeading(), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(maxAccel))
                .build());
        drive.update();
    }
    public void doSplineToConstant(Pose2d start, Pose2d end, double maxVel, double maxAccel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .splineToConstantHeading(pose2Vector(end), end.getHeading(), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(maxAccel))
                .build());
        drive.update();
    }
    public void doForward(Pose2d start, double dist, double maxVel, double maxAccel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .forward(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(maxAccel))
                .build());
        drive.update();
    }
    public void doBackward(Pose2d start, double dist, double maxVel, double maxAccel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .back(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(maxAccel))
                .build());
        drive.update();
    }
    public void doStrafeRight(Pose2d start, double dist, double maxVel, double maxAccel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .strafeRight(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(maxAccel))
                .build());
        drive.update();
    }
    public void doStrafeLeft(Pose2d start, double dist, double maxVel, double maxAccel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .strafeLeft(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(maxAccel))
                .build());
        drive.update();
    }

}
