package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.StackHeight.StackHeightPipeline;
import org.firstinspires.ftc.teamcode.OpenCV.Vision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.autonomous.AutonomousHandle.doPowershots;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousHandle.shootOneRing;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FRING_PSHOT_POSITION_TRAJ;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FRING_PSHOT_PREP_TRAJ;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.initialize;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.oneRingEndTrajectory;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.oneRingFirstWobbleDropOffTrajectory;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.oneRingPickupTrajectory;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.oneRingSecondWobbleDropOffTrajectory;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.oneRingSecondWobblePickupTrajectory;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.oneRingShootTrajectory;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.startShootingTrajectory;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.zeroFirstWobbleBackFive;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.zeroRingsReturnTrajectory;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.zeroRingsSecondWobble;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.zeroRingsSecondWobbleTrajectory;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.zeroRingsTrajectory;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.zeroSecondWobbleBackFive;

@Config
@Autonomous(name="Autonomous 123123",group = "drive")
public class AutonomousV2 extends LinearOpMode {
    // Variables
    boolean pressingA = false;
    boolean robotArmed = false;


    static ArmAndGrabberController armAndGrabberController;
    public static StackHeightPipeline.RingCase ringCase;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive.setPoseEstimate(new Pose2d(-61, -27, 0));
        drive = new SampleMecanumDrive(hardwareMap);
        armAndGrabberController = new ArmAndGrabberController(hardwareMap);
        //initializeRobotConstants(hardwareMap);
        initialize(drive);
        //pusherServo.setDirection(Servo.Direction.FORWARD);
        //shooterAngleServo.setPosition(0.59);

        Vision detector = new Vision(this, Vision.Pipeline.StackHeight);
        detector.start();

        ringCase = detector.getStackPipe().getModeResult();
        telemetry.addData("Ring Case", ringCase);
        //telemetry.addData("Pusher Position", pusherServo.getPosition());
        telemetry.update();

        waitForStart();

        ringCase = detector.getStackPipe().getModeResult();
        telemetry.addData("Ring Case", ringCase);
        telemetry.update();




        switch(ringCase) {
            case Zero:
                drive.followTrajectory(startShootingTrajectory);
                //arm.start();
                //arm.join();
                sleep(400);
                //disarm.start();
                //disarm.join();


                // Goes to First Wobble Drop Zone
                drive.followTrajectory(zeroRingsTrajectory);
                // Drops Wobble
                doWobble();
                sleep(400);
                // Goes Back 8 Inches
                drive.followTrajectory(zeroFirstWobbleBackFive);
                // Goes To The Wobble
                drive.followTrajectory(zeroRingsSecondWobbleTrajectory);
                // CLoses the claw arm
                armAndGrabberController.closeClaw();
                sleep(400);
                // Goes To Second Wobble Drop Zone
                drive.followTrajectory(zeroRingsSecondWobble);
                // Drops Wobble
                armAndGrabberController.openClaw();
                sleep(400);
                // Goes Back 8 Inches
                drive.followTrajectory(zeroSecondWobbleBackFive);
                // Goes back home
                drive.followTrajectory(zeroRingsReturnTrajectory);

                break;
            case One:
                drive.followTrajectory(startShootingTrajectory);
                //arm.start();
                //arm.join();
                sleep(400);
                //disarm.start();
                //disarm.join();

                // Starts Intake
                startIntake();
                // Goes to Pickup One Ring
                drive.followTrajectory(oneRingPickupTrajectory);
                // Goes to Shoot Trajectory
                drive.followTrajectory(oneRingShootTrajectory);
                stopIntake();
                // Arm the Robot
                //arm.start();
                // Shoots One Ring
                //shooterAngleServo.setPosition(.51);
                shootOneRing.start();
                shootOneRing.join();
                sleep(1000);
                // Disarms Robot
                //disarm.start();
                // Goes to Drop Wobble One
                drive.followTrajectory(oneRingFirstWobbleDropOffTrajectory);
                // Drops Wobble One
                doWobble();
                sleep(400);
                // Goes to Pickup Wobble Two
                drive.followTrajectory(oneRingSecondWobblePickupTrajectory);
                // Closes Arm
                armAndGrabberController.closeClaw();
                // Goes to drop zone for Wobble Two
                drive.followTrajectory(oneRingSecondWobbleDropOffTrajectory);
                // Drops
                armAndGrabberController.openClaw();
                sleep(400);
                // Goes to End Position
                drive.followTrajectory(oneRingEndTrajectory);

                break;
            case Four:
                drive.followTrajectory(FRING_PSHOT_PREP_TRAJ);
                drive.followTrajectory(FRING_PSHOT_POSITION_TRAJ);
                AutonomousHandle.startFlywheel.start();
                doPowershots.run();













                break;

        }
        // Finishes Arm Movements
        armAndGrabberController.closeClaw();
        armAndGrabberController.posControllerNoWobble(100,1);
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if(gamepad1.a && !pressingA) {
                pressingA = true;
            } else if(!gamepad1.a && pressingA) {
                pressingA = false;
            }




            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("Finished", true);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
            telemetry.update();

        }
    }




    public void doWobble() {
        armAndGrabberController.goToPosArm(130,true);
    }

    public void startIntake() {
        //intakeMotor.setPower(-1);
    }

    public void stopIntake() {
        //intakeMotor.setPower(0);
    }

}
