package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;
public class Trajectories2 {
    // Base Trajectories
    public static Trajectory startShootingTrajectory;
    public static Trajectory readNumberOfRingsTrajectory;

    // Zero Ring Trajectories
    public static Trajectory zeroRingsTrajectory;
    public static Trajectory zeroRingsSecondWobbleTrajectory;
    public static Trajectory zeroRingsReturnTrajectory;
    public static Trajectory zeroRingsSecondWobble;
    public static Trajectory zeroFirstWobbleBackFive;
    public static Trajectory zeroSecondWobbleBackFive;

    // One Ring Trajectories
    public static Trajectory oneRingPickupTrajectory;
    public static Trajectory oneRingShootTrajectory;
    public static Trajectory oneRingFirstWobbleDropOffTrajectory;
    public static Trajectory oneRingSecondWobblePickupTrajectory;
    public static Trajectory oneRingSecondWobbleDropOffTrajectory;
    public static Trajectory oneRingEndTrajectory;

    // Four Ring Trajectories
    public static Trajectory fourRingPrepStackTrajectory;
    public static Trajectory fourRingFirstHitTrajectory;
    public static Trajectory fourRingSecondHitTrajectory;
    public static Trajectory fourRingWobbleOneTrajectory;
    public static Trajectory fourRingWobbleTwoTrajectory;
    public static Trajectory fourRingWobbleTwoDropOffTrajectory;
    public static Trajectory fourRingGoEndTrajectory;


    // Base Poses
    public static Pose2d startingPosition = new Pose2d(-61, -27, 0);
    public static Pose2d startShootingPosition = new Pose2d(-45, -31, 0);
    public static Pose2d readRingsPosition = new Pose2d(-56, -32.5, 6.26);
    public static Pose2d secondWobblePosition = new Pose2d(-35.8, -40, 3.08);
    public static Pose2d endPosition = new Pose2d(13, -34.7, 6.2);
    public static Pose2d powerShotOne = new Pose2d(-45, -31, 6.24);

    // Zero Rings
    public static Pose2d zeroRingsWobblePosition = new Pose2d(10.5, -50, 4.7);
    public static Pose2d zeroRingsSecondWobbleDropPosition = new Pose2d(1.5, -46, 4.7);

    // One Ring
    public static Pose2d oneRingPickupPosition = new Pose2d(-25.2,-37.7, 6.3);
    public static Pose2d oneRingShootPosition = new Pose2d(-24.8, -36.3, 0);
    public static Pose2d oneRingWobbleDropPosition = new Pose2d(26.8, -47.8, 0); // new Pose2d(26.8, -37.8, 0);
    public static Pose2d oneRingSecondWobbleDropPosition = new Pose2d(17.2, -56, 0);

    // Four Rings
    public static Pose2d fourRingPrepPosition = new Pose2d(-36.8,-34.2, 6.26);
    public static Pose2d fourRingWobbleDropPosition = new Pose2d(45.4, -59.3, 0);
    public static Pose2d fourRingSecondWobbleDropPosition = new Pose2d(38.98, -55, 5.4);


    public static void initialize(SampleMecanumDrive drive) {
        drive.setPoseEstimate(new Pose2d(-61, -27, 0));


        startShootingTrajectory = drive.trajectoryBuilder(startingPosition)
                .lineToSplineHeading(startShootingPosition)
                .build();

        readNumberOfRingsTrajectory = drive.trajectoryBuilder(startShootingTrajectory.end())
                .lineToSplineHeading(readRingsPosition)
                .build();


        //////////////////////////////////////////////////////////////////////////////////////////////////////
        //                                      Zero Ring Trajectories                                      //
        //////////////////////////////////////////////////////////////////////////////////////////////////////

        zeroRingsTrajectory = drive.trajectoryBuilder(startShootingTrajectory.end())
                .lineToSplineHeading(zeroRingsWobblePosition)
                .build();


        zeroFirstWobbleBackFive = drive.trajectoryBuilder(zeroRingsTrajectory.end())
                .back(8)
                .build();

        zeroRingsSecondWobbleTrajectory = drive.trajectoryBuilder(zeroFirstWobbleBackFive.end())
                .lineToSplineHeading(secondWobblePosition)
                .build();

        zeroRingsSecondWobble = drive.trajectoryBuilder(zeroFirstWobbleBackFive.end())
                .lineToSplineHeading(zeroRingsSecondWobbleDropPosition)
                .build();
        zeroSecondWobbleBackFive = drive.trajectoryBuilder(zeroRingsSecondWobble.end())
                .back(8)
                .build();

        zeroRingsReturnTrajectory = drive.trajectoryBuilder(zeroSecondWobbleBackFive.end())
                .lineToSplineHeading(endPosition)
                .build();

        //////////////////////////////////////////////////////////////////////////////////////////////////////
        //                                      One Ring Trajectories                                       //
        //////////////////////////////////////////////////////////////////////////////////////////////////////

        oneRingPickupTrajectory = drive.trajectoryBuilder(startShootingTrajectory.end())
                .lineToSplineHeading(oneRingPickupPosition)
                .build();

        oneRingShootTrajectory = drive.trajectoryBuilder(oneRingPickupTrajectory.end())
                .lineToSplineHeading(oneRingShootPosition)
                .build();

        oneRingFirstWobbleDropOffTrajectory = drive.trajectoryBuilder(oneRingShootTrajectory.end())
                .lineToSplineHeading(oneRingWobbleDropPosition)
                .build();

        oneRingSecondWobblePickupTrajectory = drive.trajectoryBuilder(oneRingFirstWobbleDropOffTrajectory.end())
                .lineToSplineHeading(secondWobblePosition)
                .build();

        oneRingSecondWobbleDropOffTrajectory = drive.trajectoryBuilder(oneRingSecondWobblePickupTrajectory.end())
                .lineToSplineHeading(oneRingSecondWobbleDropPosition)
                .build();

        oneRingEndTrajectory = drive.trajectoryBuilder(oneRingSecondWobbleDropOffTrajectory.end())
                .lineToSplineHeading(endPosition)
                .build();


        //////////////////////////////////////////////////////////////////////////////////////////////////////
        //                                     Four Ring Trajectories                                       //
        //////////////////////////////////////////////////////////////////////////////////////////////////////

        fourRingPrepStackTrajectory = drive.trajectoryBuilder(startShootingTrajectory.end())
                .lineToSplineHeading(fourRingPrepPosition)
                .build();


        // Constraints
        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(
                Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_VEL),
                        new MecanumVelocityConstraint(22, DriveConstants.TRACK_WIDTH)
                )
        );

        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL);


        fourRingFirstHitTrajectory = drive.trajectoryBuilder(fourRingPrepStackTrajectory.end())
                .forward(12, velConstraint, accelConstraint)
                .build();

        fourRingSecondHitTrajectory = drive.trajectoryBuilder(fourRingFirstHitTrajectory.end())
                .forward(15, velConstraint, accelConstraint)
                .build();

        // TODO: FIX THIS TO FIRST WOBBLE POSITION AFTER COMP
        fourRingWobbleOneTrajectory = drive.trajectoryBuilder(fourRingSecondHitTrajectory.end())
                .lineToSplineHeading(fourRingSecondWobbleDropPosition)
                .build();

        fourRingWobbleTwoTrajectory = drive.trajectoryBuilder(fourRingWobbleOneTrajectory.end())
                .lineToSplineHeading(secondWobblePosition)
                .build();

        fourRingWobbleTwoDropOffTrajectory = drive.trajectoryBuilder(fourRingWobbleTwoTrajectory.end())
                .lineToSplineHeading(fourRingSecondWobbleDropPosition)
                .build();

        fourRingGoEndTrajectory = drive.trajectoryBuilder(fourRingWobbleOneTrajectory.end())
                .lineToSplineHeading(endPosition)
                .build();



    }
}
