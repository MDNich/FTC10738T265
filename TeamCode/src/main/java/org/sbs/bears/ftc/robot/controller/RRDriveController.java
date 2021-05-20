package org.sbs.bears.ftc.robot.controller;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.ftc.util.RobotSubsytemManager;

import java.util.Arrays;

public class RRDriveController extends RobotSubsytemManager {
    public SampleMecanumDrive drive;
    Telemetry telemetry;
    HardwareMap hwMap;


    public RRDriveController(HardwareMap hwMap, Telemetry telemetry)
    {
        super(hwMap,telemetry);
        drive = new SampleMecanumDrive(hwMap,true);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void shutDown() {
        //nothing
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





















    public void doLineToSpline(Pose2d start, Pose2d end) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .lineToSplineHeading(end)
                .build());
    }
    public void doSplineToSpline(Pose2d start, Pose2d end) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .splineToSplineHeading(end,end.getHeading())
                .build());
    }
    public void doSplineToConstant(Pose2d start, Pose2d end) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .splineToConstantHeading(pose2Vector(end), end.getHeading())
                .build());
    }
    public void doForward(Pose2d start, double dist) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .forward(dist)
                .build());
    }
    public void doBackward(Pose2d start, double dist) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .back(dist)
                .build());
    }
    public void doStrafeRight(Pose2d start, double dist) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .strafeRight(dist)
                .build());
    }
    public void doStrafeLeft(Pose2d start, double dist) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .strafeLeft(dist)
                .build());
    }



    // overloads
    public void doLineToSpline(Pose2d start, Pose2d end, double maxVel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .lineToSplineHeading(end, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build());
    }
    public void doSplineToSpline(Pose2d start, Pose2d end, double maxVel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .splineToSplineHeading(end,end.getHeading(), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build());
    }
    public void doSplineToConstant(Pose2d start, Pose2d end, double maxVel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .splineToConstantHeading(pose2Vector(end), end.getHeading(), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build());
    }
    public void doForward(Pose2d start, double dist, double maxVel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .forward(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build());
    }
    public void doBackward(Pose2d start, double dist, double maxVel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .back(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build());
    }
    public void doStrafeRight(Pose2d start, double dist, double maxVel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .strafeRight(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build());
    }
    public void doStrafeLeft(Pose2d start, double dist, double maxVel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .strafeLeft(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build());
    }

    // overloads
    public void doLineToSpline(Pose2d start, Pose2d end, double maxVel, double maxAccel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .lineToSplineHeading(end, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(maxAccel))
                .build());
    }
    public void doSplineToSpline(Pose2d start, Pose2d end, double maxVel, double maxAccel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .splineToSplineHeading(end,end.getHeading(), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(maxAccel))
                .build());
    }
    public void doSplineToConstant(Pose2d start, Pose2d end, double maxVel, double maxAccel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .splineToConstantHeading(pose2Vector(end), end.getHeading(), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(maxAccel))
                .build());
    }
    public void doForward(Pose2d start, double dist, double maxVel, double maxAccel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .forward(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(maxAccel))
                .build());
    }
    public void doBackward(Pose2d start, double dist, double maxVel, double maxAccel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .back(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(maxAccel))
                .build());
    }
    public void doStrafeRight(Pose2d start, double dist, double maxVel, double maxAccel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .strafeRight(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(maxAccel))
                .build());
    }
    public void doStrafeLeft(Pose2d start, double dist, double maxVel, double maxAccel) {
        drive.followTrajectory(drive.trajectoryBuilder(start)
                .strafeLeft(dist, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(maxVel, DriveConstants.TRACK_WIDTH)
                )), new ProfileAccelerationConstraint(maxAccel))
                .build());
    }

}
