package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.intakeMotor;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.liftServo;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.pusherServo;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.pusherServoInPos;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.pusherServoOutPos;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.s1;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.s2;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.shooterAngleServo;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FPOS_WOBBLE_ONE;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FRING_FINAL_THREE_TRAJ;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FRING_FINAL_TRAJ;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FRING_FIRST_RING_TRAJ;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FRING_PSHOT_POSITION_TRAJ;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FRING_PSHOT_PREP_TRAJ;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FRING_SECOND_WOBBLE_TRAJ;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FRING_WOBBLE_ONE_BACKUP_TRAJ;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FRING_WOBBLE_ONE_TRAJ;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FRING_WOBBLE_STACK_PREP_TRAJ;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FRING_WOBBLE_TWO_PICKUP_TRAJ;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.FRING_WOBBLE_TWO_PREP_TRAJ;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.initialize;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.initializeFourSecond;
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
import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@Config
@Autonomous(name="Autonomous New Four",group = "drive")
public class AutonNew extends LinearOpMode {
    public static SampleMecanumDrive drive;
    public static AutonomousHandle autonomousHandle;

    private Servo lJeff;
    private Servo rJeff;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Returns RoadRunner driver
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-61, -27, 0));
        // Creates Autonomous Handle
        autonomousHandle = new AutonomousHandle(drive, hardwareMap, telemetry);
        // Arm
        ArmAndGrabberController armAndGrabberController = new ArmAndGrabberController(hardwareMap);
        // Vision
        // Return Start Information
        autonomousHandle.displayHardwareReport();
        // Initialize First Trajectories
        Trajectories.initialize(drive);
        // lJeff + rJeff
        rJeff = hardwareMap.servo.get("rJeff");
        lJeff = hardwareMap.servo.get("lJeff");
        //TOD SHOOTER ANGLE FOR THE THREE, CHANGE THIS UP BRIGNS LOW DOWN BRIGNS UP, INITIAL THREE
        shooterAngleServo.setPosition(.7); // bro i stg don't do anything stupid
        waitForStart();
        // Clears Telemetry
        telemetry.clear();

        lJeffUp.start();
        rJeffIn.start();


        drive.followTrajectory(FRING_PSHOT_PREP_TRAJ);


        // Start the Flywheel
        startFlywheel.start();
        drive.followTrajectory(FRING_PSHOT_POSITION_TRAJ);

        shooterAngleServo.setPosition(0.55); // Raising # = Down, Lowering # = Up
        doPowerShots();
        stopFlywheel.run();




        int ringCase = 4;
        // Begin DropWobble Operations
        switch(ringCase) {
            case 0:
                drive.followTrajectory(zeroRingsTrajectory);
                // Drops Wobble
                armAndGrabberController.goToPosArm(125, true);
                sleep(400);
                // Goes Back 8 Inches
                /*drive.followTrajectory(zeroFirstWobbleBackFive);
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
                // Goes back home*/
                drive.followTrajectory(zeroRingsReturnTrajectory);
                break;


            case 1:
                drive.followTrajectory(oneRingFirstWobbleDropOffTrajectory);
                // Drops Wobble One
                armAndGrabberController.goToPosArm(125, true);
                sleep(500);
                // Goes to Pickup Wobble Two
                /*drive.followTrajectory(oneRingSecondWobblePickupTrajectory);
                // Closes Arm
                armAndGrabberController.closeClaw();
                // Goes to drop zone for Wobble Two
                drive.followTrajectory(oneRingSecondWobbleDropOffTrajectory);
                // Drops
                armAndGrabberController.openClaw();
                sleep(400);
                // Goes to End Position*/
                drive.followTrajectory(oneRingEndTrajectory);
                break;

            case 4:
                // Move to the Powershot Location
                // TODO: Optimize The Prep Traj
                                // Initialize Second Set of Trajectories
                initializeFourSecond(drive);
                // End

                // First Wobble
                drive.followTrajectory(FRING_WOBBLE_ONE_TRAJ);
                armAndGrabberController.goToPosArm(125, true);
                drive.followTrajectory(FRING_WOBBLE_ONE_BACKUP_TRAJ);
                drive.followTrajectory(FRING_FINAL_TRAJ);
                break;
        }
        lJeffUp.start();
        new Thread(()->{armAndGrabberController.posControllerNoWobble(50,1.5);}).start();
        armAndGrabberController.closeClaw();
        while (!isStopRequested()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("X", poseEstimate.getX());
            telemetry.addData("Y", poseEstimate.getY());
            telemetry.addData("H", poseEstimate.getHeading());
            telemetry.update();
        }
    }


    public void fourRingOperation() throws InterruptedException {



    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                   Publicly Availible References                                       //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    static ArmAndGrabberController armAndGrabberController;
    // Starts Intake
    public void startIntake() {
        intakeMotor.setPower(-1);
    }
    // Stops Intake
    public void stopIntake() {
        intakeMotor.setPower(0);
    }
    // Puts Lift Up
    public static Thread liftUp = new Thread(() -> {
        liftServo.setPosition(0.86);
    });

    // Puts Lift Down
    public static Thread liftDown = new Thread(() -> {
        liftServo.setPosition(1);
    });

    // Stops FlyWheel Independently
    public static Thread stopFlywheel = new Thread(() -> {
        // 0 Stop
        s1.setPower(0);
        s2.setPower(0);
    });

    // Starts FlyWheel Independently
    public static Thread startFlywheel = new Thread(() -> {
        // -1 Start
        s1.setPower(-1);
        s2.setPower(-1);
    });

    public void doPowerShots() throws InterruptedException {
        liftUp.run();
        s1.setPower(-0.9);
        s2.setPower(-0.9);
        shootOneRing.start();
        shootOneRing.join();
        sleep(350);
        drive.turn(Math.toRadians(8));
        shootOneRing.start();
        shootOneRing.join();
        sleep(350);
        shooterAngleServo.setPosition(0.56); // Testing / Experimental
        drive.turn(Math.toRadians(10));
        shootOneRing.start();
        shootOneRing.join();
        sleep(350);
        liftDown.start();
        drive.update();
        Trajectories.initialize(drive);
    }

    public Thread shootOneRing = new Thread(() -> {
        sleep(400);
        pusherServo.setPosition(pusherServoOutPos);
        sleep(400);
        pusherServo.setPosition(pusherServoInPos);
        sleep(400);
        pusherServo.setPosition(pusherServoOutPos);
    });

    public Thread shootThreeRings = new Thread(() -> {
        sleep(400);
        pusherServo.setPosition(pusherServoOutPos);
        sleep(400);
        pusherServo.setPosition(pusherServoInPos);
        sleep(400);
        pusherServo.setPosition(pusherServoOutPos);
        sleep(400);
        pusherServo.setPosition(pusherServoInPos);
        sleep(400);
        pusherServo.setPosition(pusherServoOutPos);
        sleep(400);
        pusherServo.setPosition(pusherServoInPos);
    });


    //////////////////////////////////////////////
    //          Jeff Control System             //
    //////////////////////////////////////////////

    public double lJeffUpPos = .49;
    public double lJeffDownPos = .149;
    public Thread lJeffUp = new Thread(() -> {
        lJeff.setPosition(lJeffUpPos);
    });

    public Thread lJeffDown = new Thread(() -> {
        lJeff.setPosition(lJeffDownPos);
    });

    public double rJeffUpPos = .3;
    public double rJeffDownPos = .67;
    public double rJeffInPos = .06;
    public Thread rJeffUp = new Thread(() -> {
        rJeff.setPosition(rJeffUpPos);
    });

    public Thread rJeffDown = new Thread(() -> {
        rJeff.setPosition(rJeffDownPos);
    });

    public Thread rJeffIn = new Thread(() -> {
        rJeff.setPosition(rJeffInPos);
    });


}