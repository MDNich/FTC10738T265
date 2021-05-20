package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveNoOdom;

@TeleOp
public class PathTesting extends OpMode {

    SampleMecanumDriveNoOdom drive;
    private boolean qA;
    private boolean qB;


    @Override
    public void init() {
        drive = new SampleMecanumDriveNoOdom(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry);
        msStuckDetectLoop = 50000000;
    }

    @Override
    public void loop() {
        Pose2d myPose = drive.getPoseEstimate();
        if(gamepad1.a && !qA)
        {
            qA = true;
            Trajectory lineToSplineTraj = drive.trajectoryBuilder(myPose)
                    .lineToSplineHeading(new Pose2d(myPose.getX() + 30,myPose.getY() + 30,myPose.getHeading() + Math.PI))
                    .build();
            drive.followTrajectory(lineToSplineTraj);
        }
        if(!gamepad1.a && qA)
        {
            qA = false;
        }
        if(gamepad1.b && !qB)
        {
            qB = true;
            Trajectory lineToSplineTraj = drive.trajectoryBuilder(myPose)
                    .lineToSplineHeading(new Pose2d(myPose.getX() - 30,myPose.getY() + 10,myPose.getHeading() - Math.PI/2))
                    .build();
            drive.followTrajectory(lineToSplineTraj);
        }
        if(!gamepad1.b && qB)
        {
            qB = false;
        }

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();
        telemetry.update();

    }
}
