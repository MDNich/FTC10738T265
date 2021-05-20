package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.Vision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.autonomous.AutonomousHandle.doPowershots;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousV2.ringCase;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FRING_PSHOT_POSITION_TRAJ;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FRING_PSHOT_PREP_TRAJ;

@Config
@Autonomous(name="Autonomous Main",group = "drive")
public class AutonomousV3 extends LinearOpMode {
    // Vision Pipeline
    //
    // RoadRunner
    public static SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException
    {
        // Creates a Vision Handle for Vision
        Vision VisionHandle = new Vision(this, Vision.Pipeline.StackHeight);
        // Init Robot
        AutonomousReferences.initializeRobot(hardwareMap);
        // Returns RoadRunner driver
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectories roadRunnerTrajectories = new Trajectories();
        roadRunnerTrajectories.initialize(drive);
        // Creates Autonomous Handle
        AutonomousHandle autonomousHandle = new AutonomousHandle(drive, hardwareMap, telemetry);
        // Start the Vision detector
        VisionHandle.start();
        // Gets Beginning Ring Case
        ringCase = VisionHandle.getStackPipe().getModeResult();
        // Return Start Information
        autonomousHandle.displayHardwareReport();
        // Queue All Operations
        waitForStart();
        // Clears Telemetry
        telemetry.clear();
        // Reconfirm Rings
        ringCase = VisionHandle.getStackPipe().getModeResult();
        telemetry.addData("Ring Case", ringCase);
        telemetry.update();
        // Display Vision Information
        // Setup RoadRunner
        drive.setPoseEstimate(new Pose2d(-61, -27, 0));


        // Begin Main Operations
        switch(ringCase) {
            case Zero:
                break;

            case One:
                break;

            case Four:
                fourRingOperation();
                break;
        }
        while (!isStopRequested()) {


        }
    }


    public void fourRingOperation() {
        // Move to the Powershot Location
        drive.followTrajectory(FRING_PSHOT_PREP_TRAJ);
        drive.followTrajectory(FRING_PSHOT_POSITION_TRAJ);
        // Start the Flywheel
        AutonomousHandle.startFlywheel.start();
        // Do Three Powershots
        doPowershots.run();
    }
}





