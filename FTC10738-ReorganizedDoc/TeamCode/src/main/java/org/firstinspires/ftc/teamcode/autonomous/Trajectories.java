package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

public class Trajectories {

    // Base Poses
    public static Pose2d startingPosition = new Pose2d(-61, -27, 0);
    public static Pose2d startShootingPosition = new Pose2d(-45, -31, 0);
    public static Pose2d readRingsPosition = new Pose2d(-56, -32.5, 6.26);
    public static Pose2d secondWobblePosition = new Pose2d(-35.8, -40, 3.08);
    public static Pose2d endPosition = new Pose2d(13, -34.7, 6.2);
    public static Pose2d powerShotOne = new Pose2d(-45, -31, 6.24);

    // Zero Rings Positions
    public static Pose2d zeroRingsWobblePosition = new Pose2d(10.5, -50, 4.7);
    public static Pose2d zeroRingsSecondWobbleDropPosition = new Pose2d(1.5, -46, 4.7);

    // One Ring Positions
    public static Pose2d oneRingPickupPosition = new Pose2d(-25.2,-37.7, 6.3);
    public static Pose2d oneRingShootPosition = new Pose2d(-24.8, -36.3, 0);
    public static Pose2d oneRingWobbleDropPosition = new Pose2d(26.8, -47.8, 0); // new Pose2d(26.8, -37.8, 0);
    public static Pose2d oneRingSecondWobbleDropPosition = new Pose2d(17.2, -56, 0);

    ////////////////////////////////////////////////////////////////////
    //                   TODO: Four Ring Positions                    //
    ////////////////////////////////////////////////////////////////////
    // Final Positions MAY have a drift difference of CURRENT_HEADING - 6.17 (DRIFT CORRECTION)
    // resulting in 6.17 being the final 0 heading instead of 0
    // which is TODO: TBD
    public static Pose2d FPOS_PSHOT_PREP = new Pose2d(-25.36, -13.61, 6.17);
    public static Pose2d FPOS_PSHOT_POSITION = new Pose2d(0.69, -17.48, 0);
    public static Pose2d FPOS_WOBBLE_ONE = new Pose2d(45.14, -62.45, 5.31 - 6.17);
    // Technically Backups up (4) amount of inches
    public static Pose2d FPOS_WOBBLE_TWO_PREP = new Pose2d(-40.10, -52.16, 1.82 - 6.17);
    public static Pose2d FPOS_WOBBLE_TWO_PICKUP = new Pose2d(-41, -48.39, 1.82 - 6.17);
    public static Pose2d FPOS_PREP_STACK = new Pose2d(-47.31, -28.6, 0);
    // Technically Goes Forward with Constraints (24.9)
    // Shoot One
    // Technically Goes Forward with Constraints (20)
    // Shoot Three
    public static Pose2d FPOS_WOBBLE_TWO = new Pose2d(38.25, -55.66, 5.31 - 6.17);
    // Finish with backwards (30)
    // May require
    // public static Pose2d FPOS_FINAL_POSITION;
    // Arm Up Close Claw




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

    public static Trajectory FRING_PSHOT_PREP_TRAJ;
    public static Trajectory FRING_PSHOT_POSITION_TRAJ;
    public static Trajectory FRING_WOBBLE_ONE_TRAJ;
    public static Trajectory FRING_WOBBLE_ONE_BACKUP_TRAJ;
    public static Trajectory FRING_WOBBLE_TWO_PREP_TRAJ;
    public static Trajectory FRING_WOBBLE_TWO_PICKUP_TRAJ;
    public static Trajectory FRING_WOBBLE_STACK_PREP_TRAJ;
    public static Trajectory FRING_FIRST_RING_TRAJ;
    public static Trajectory FRING_FINAL_THREE_TRAJ;
    public static Trajectory FRING_SECOND_WOBBLE_TRAJ;
    public static Trajectory FRING_FINAL_TRAJ;




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


        TrajectoryVelocityConstraint velConstraint1 = new MinVelocityConstraint(
                Arrays.asList(
                        new AngularVelocityConstraint(30),
                        new MecanumVelocityConstraint(22, DriveConstants.TRACK_WIDTH)
                )
        );

        TrajectoryAccelerationConstraint accelConstraint2 = new ProfileAccelerationConstraint(30);

        zeroRingsSecondWobble = drive.trajectoryBuilder(zeroFirstWobbleBackFive.end())
                .lineToSplineHeading(zeroRingsSecondWobbleDropPosition, velConstraint1, accelConstraint2)
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

        // Go to Left Side Of Ring
        FRING_PSHOT_PREP_TRAJ = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(FPOS_PSHOT_PREP)
                .build();
        // Go to the powershot location
        FRING_PSHOT_POSITION_TRAJ = drive.trajectoryBuilder(FRING_PSHOT_PREP_TRAJ.end())
                .lineToSplineHeading(FPOS_PSHOT_POSITION)
                .build();
        // Go to the first wobble location
        FRING_WOBBLE_ONE_TRAJ = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(FPOS_WOBBLE_ONE)
                .build();
        // Backup a certain amount
        FRING_WOBBLE_ONE_BACKUP_TRAJ = drive.trajectoryBuilder(FRING_WOBBLE_ONE_TRAJ.end())
                .back(15)
                .build();
        // Go to second wobble pickup location
        FRING_WOBBLE_TWO_PREP_TRAJ = drive.trajectoryBuilder(FRING_WOBBLE_ONE_BACKUP_TRAJ.end())
                .lineToSplineHeading(FPOS_WOBBLE_TWO_PREP)
                .build();
        // Go forward
        FRING_WOBBLE_TWO_PICKUP_TRAJ = drive.trajectoryBuilder(FRING_WOBBLE_TWO_PREP_TRAJ.end())
                .lineToSplineHeading(FPOS_WOBBLE_TWO_PICKUP)
                .build();
        // Go to stack prep position
        FRING_WOBBLE_STACK_PREP_TRAJ = drive.trajectoryBuilder(FRING_WOBBLE_TWO_PICKUP_TRAJ.end())
                .lineToSplineHeading(FPOS_PREP_STACK)
                .build();
        // Hit the stack and pick up one ring and shoot one
        FRING_FIRST_RING_TRAJ = drive.trajectoryBuilder(FRING_WOBBLE_STACK_PREP_TRAJ.end())
                .forward(24)
                .build();
        // Go to the second position and shoot three
        FRING_FINAL_THREE_TRAJ = drive.trajectoryBuilder(FRING_FIRST_RING_TRAJ.end())
                .forward(20)
                .build();
        // Go to the second wobble drop location
        FRING_SECOND_WOBBLE_TRAJ = drive.trajectoryBuilder(FRING_FINAL_THREE_TRAJ.end())
                .lineToSplineHeading(FPOS_WOBBLE_TWO)
                .build();
        // Park on the line aka backup a certain amount
        FRING_FINAL_TRAJ = drive.trajectoryBuilder(FRING_SECOND_WOBBLE_TRAJ.end())
                .back(30)
                .build();

















    }
}
