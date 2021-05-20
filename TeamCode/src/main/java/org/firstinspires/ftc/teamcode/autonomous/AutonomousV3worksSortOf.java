package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories.zeroSecondWobblePart1;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories2.fourRingFirstHitTrajectory;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories2.fourRingGoEndTrajectory;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories2.fourRingPrepStackTrajectory;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories2.fourRingSecondHitTrajectory;
import static org.firstinspires.ftc.teamcode.autonomous.Trajectories2.fourRingWobbleOneTrajectory;

@Config
@Autonomous(name="Autonomous Maybe",group = "drive")
public class AutonomousV3worksSortOf extends LinearOpMode {
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
        // Start the Vision detector
        // Gets Beginning Ring Case
        // Return Start Information
        autonomousHandle.displayHardwareReport();
        // Initialize First Trajectories
        Trajectories.initialize(drive);
        Trajectories2.initialize(drive);
        // lJeff + rJeff
        rJeff = hardwareMap.servo.get("rJeff");
        lJeff = hardwareMap.servo.get("lJeff");
        shooterAngleServo.setPosition(.52);
        waitForStart();
        // Clears Telemetry
        startFlywheel.run();
        telemetry.clear();
        // Reconfirm Rings

        lJeffUp.start();
        rJeffIn.start();

        int ringCase = 4;
        // Begin Main Operations
        switch(ringCase) {
            case 0:
                drive.followTrajectory(startShootingTrajectory);
                liftUp.run();
                liftUp.join();
                //pusherServo.setPosition(0);
                sleep(1000);
                shootThreeRings.start();
                shootThreeRings.join();
                sleep(400);
                stopFlywheel.run();
                liftDown.run();
                // Goes to First Wobble Drop Zone
                drive.followTrajectory(zeroRingsTrajectory);
                // Drops Wobble
                armAndGrabberController.goToPosArm(125, true);
                sleep(400);
                // Goes Back 8 Inches
                drive.followTrajectory(zeroFirstWobbleBackFive);
                // Goes To The Wobble
                drive.followTrajectory(zeroRingsSecondWobbleTrajectory);
                // CLoses the claw arm
                armAndGrabberController.closeClaw();
                sleep(400);
                // Goes To Second Wobble Drop Zone
                drive.followTrajectory(zeroSecondWobblePart1);
                drive.followTrajectory(zeroRingsSecondWobble);
                // Drops Wobble
                armAndGrabberController.openClaw();
                sleep(400);
                // Goes Back 8 Inches
                drive.followTrajectory(zeroSecondWobbleBackFive);
                // Goes back home
                drive.followTrajectory(zeroRingsReturnTrajectory);
                break;


            case 1:
                drive.followTrajectory(startShootingTrajectory);
                liftUp.run();
                liftUp.join();
                sleep(1000);
                shootThreeRings.start();
                shootThreeRings.join();
                sleep(400);
                //stopFlywheel.run();
                liftDown.run();
                // Starts Intake
                startIntake();
                // Goes to Pickup One Ring
                drive.followTrajectory(oneRingPickupTrajectory);
                // Goes to Shoot Trajectory
                drive.followTrajectory(oneRingShootTrajectory);
                stopIntake();
                // Arm the Robot
                //startFlywheel.run();
                //startFlywheel.join();
                liftUp.start();
                liftUp.join();
                // Shoots One Ring
                shooterAngleServo.setPosition(.53);
                shootOneRing.start();
                shootOneRing.join();
                sleep(1000);
                // Disarms Robot
                stopFlywheel.run();
                liftDown.run();
                // Goes to Drop Wobble One
                drive.followTrajectory(oneRingFirstWobbleDropOffTrajectory);
                // Drops Wobble One
                armAndGrabberController.goToPosArm(125, true);
                sleep(500);
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

            case 4:
                /* // Move to the Powershot Location
                // TODO: Optimize The Prep Traj
                drive.followTrajectory(FRING_PSHOT_PREP_TRAJ);


                // Start the Flywheel
                drive.followTrajectory(FRING_PSHOT_POSITION_TRAJ);

                shooterAngleServo.setPosition(0.55); // Raising # = Down, Lowering # = Up
                doPowerShots();
                stopFlywheel.run();

                // Initialize Second Set of Trajectories
                initializeFourSecond(drive);
                // End

                // First Wobble
                drive.followTrajectory(FRING_WOBBLE_ONE_TRAJ);
                armAndGrabberController.goToPosArm(125, true);
                drive.followTrajectory(FRING_WOBBLE_ONE_BACKUP_TRAJ);
                drive.followTrajectory(FRING_WOBBLE_TWO_PREP_TRAJ);
                drive.followTrajectory(FRING_WOBBLE_TWO_PICKUP_TRAJ);
                sleep(300);
                armAndGrabberController.closeClaw();
                startFlywheel.run();
                drive.followTrajectory(FRING_WOBBLE_STACK_PREP_TRAJ);
                startIntake();
                drive.followTrajectory(FRING_FIRST_RING_TRAJ);
                //sleep(800);
                stopIntake();
                shooterAngleServo.setPosition(.515);
                liftUp.run();
                shootOneRing.run();
                //sleep(1000);
                liftDown.run();
                intakeMotor.setPower(-1);
                startIntake();
                drive.followTrajectory(FRING_FINAL_THREE_TRAJ);
                //sleep(1500);
                stopIntake();
                shooterAngleServo.setPosition(0.4925);
                liftUp.run();
                shootThreeRings.run();
                sleep(500);
                stopFlywheel.run();
                drive.followTrajectory(FRING_SECOND_WOBBLE_TRAJ);
                //sleep(300);
                armAndGrabberController.openClaw();
                drive.followTrajectory(FRING_FINAL_TRAJ);
                armAndGrabberController.closeClaw();
                break; */

                // Go to Prep 4 Stack
                drive.followTrajectory(startShootingTrajectory);
                liftUp.run();
                liftUp.join();
                sleep(1000);
                shootThreeRings.start();
                shootThreeRings.join();
                sleep(400);
                liftDown.run();
                liftDown.join();
                sleep(200);


                drive.followTrajectory(fourRingPrepStackTrajectory);
                // Hit the Stack Once
                startIntake();
                drive.followTrajectory(fourRingFirstHitTrajectory);
                sleep(1000);
                // Shoot the Current Rings
                stopIntake();
                liftUp.run();
                liftUp.join();
                shooterAngleServo.setPosition(.50); // was decreased by 0.01
                sleep(100);
                startFlywheel.run();
                sleep(1000);
                shootThreeRings.start();
                shootThreeRings.join();
                sleep(400);
                //stopFlywheel.run();
                //stopFlywheel.run();

                liftDown.run();
                /*arm.start();
                arm.join(); */
                //shooterAngleServo.setPosition(.51);
                /* shootThreeRings.start();
                shootThreeRings.join(); */
                sleep(400);
                //disarm.start();
                //disarm.join();
                // Hit the stack again
                startIntake();
                drive.followTrajectory(fourRingSecondHitTrajectory);
                sleep(1000);
                // Shoot the rings
                stopIntake();
                liftUp.run();
                //arm.start();
                //arm.join();
                shooterAngleServo.setPosition(.48); // was decreased by 0.1
                sleep(100);
                //startFlywheel.run();
                sleep(1000);
                shootThreeRings.start();
                shootThreeRings.join();
                sleep(400);
                stopFlywheel.run();
                liftDown.run();
                //disarm.start();
                //drive.followTrajectory(fourRingWobbleOneTrajectory);
                // Drops Wobble One
                drive.followTrajectory(fourRingWobbleOneTrajectory);
                // Drops Wobble One
                armAndGrabberController.goToPosArm(125, true);
                sleep(500);
                //doWobble();
                //sleep(400);
                // Goes to Pickup Wobble Two
                //drive.followTrajectory(fourRingWobbleTwoTrajectory);
                // Closes Arm
                //armAndGrabberController.closeClaw();
                // Goes to drop zone for Wobble Two
                //drive.followTrajectory(fourRingWobbleTwoDropOffTrajectory);
                // Drops
                //armAndGrabberController.openClaw();
                //sleep(400);
                // Goes to End Position
                drive.followTrajectory(fourRingGoEndTrajectory);
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
        shootOneRing.start();
        shootOneRing.join();
        sleep(150);
        drive.turn(Math.toRadians(8));
        shootOneRing.start();
        shootOneRing.join();
        sleep(150);
        shooterAngleServo.setPosition(0.56); // Testing / Experimental
        drive.turn(Math.toRadians(10));
        shootOneRing.start();
        shootOneRing.join();
        sleep(150);
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

    int timing1 = 200;
    int timing2 = 200;
    /////////// END OF ADDED BY MARC ///
    Thread shootThreeRings = new Thread(() -> {
        pusherServo.setPosition(0.35);
        sleep(timing1); // variablized
        pusherServo.setPosition(0.05);
        sleep(timing2); // variablized
        pusherServo.setPosition(0.35);
        sleep(timing1); // variablized
        pusherServo.setPosition(0.05);
        sleep(timing2); // variablized
        pusherServo.setPosition(0.35);
        sleep(timing1); // variablized
        pusherServo.setPosition(0.05);
        sleep(timing2);
        pusherServo.setPosition(.35);
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
