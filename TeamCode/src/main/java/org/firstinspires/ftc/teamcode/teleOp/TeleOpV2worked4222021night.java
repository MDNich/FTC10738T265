package org.firstinspires.ftc.teamcode.teleOp;
// Robot Core / Main

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.autonomous.ArmAndGrabberController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.sbs.bears.ftc.robot.controller.PIDshooterController;


@TeleOp(name="TeleOp OLD", group="Linear Opmode")
public class TeleOpV2worked4222021night extends LinearOpMode {

    // Rev Extensions 2 References
    ArmAndGrabberController armAndGrabberController;
    ExpansionHubEx expansionHub;
    ExpansionHubEx controlHub;
    RevBulkData bulkData;

    // Motors + Servo Declarations
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private DcMotor intake = null;
    private DcMotor s1 = null;
    private DcMotor s2 = null;
    private Servo shooterServo;
    private Servo lJeff;
    private Servo rJeff;
    private Servo liftServo;
    private Servo pusherServo;

    boolean intakeOn = false;
    boolean pressingX = false;
    boolean pressingB = false;
    boolean pressingA = false;
    boolean pressingY = false;
    boolean pressingRightBumper = false;
    boolean pressingLeftBumper = false;
    boolean pressingUp = false;
    boolean pressingDown = false;
    boolean wingsUp = true;
    private SampleMecanumDrive drive;
    private boolean isPIDenabled = false;
    private boolean isRunnningShooter;

    public enum angleState {
        PSHOT,
        BSHOT;
    }

    boolean robotArmed = false;
    boolean shootingAsync = false;
    double PShotAngle = 0.5;
    double RegularAngle =  0.395; // changed.

    private boolean qRB;
    private boolean qStop;
    private boolean qLB;
    private boolean qLT;
    private boolean qArmInFront;

    public enum lJeffStates {
        UP, DOWN
    }

    public enum rJeffStates {
        UP, DOWN, IN
    }

    public enum armStates {
        IN, OUT
    }

    public lJeffStates lJeffState;
    public rJeffStates rJeffState;
    public armStates armState;

    ///////// ADDED BY MARC ////////////
    private PIDshooterController shooterPIDController;
    Thread threadPIDcontroller = new Thread(()->{
        while(opModeIsActive())
        {
            if(isPIDenabled)
            {
                try {
                    shooterPIDController.doPIDLoop(2200);
                }
                catch (Exception e)
                {
                    telemetry.addData("hey!","initialize PID");
                }
            }
            else
            {
                shooterPIDController.doPIDLoop(100);
                //shooterPIDController.setPowShooter(0);
            }
        }
    });
    ///////// END ADDED BY MARC /////////




    @Override
    public void runOpMode() throws InterruptedException {

        armAndGrabberController = new ArmAndGrabberController(hardwareMap);
        /////////////// ADDED BY MARC ///////////////
        shooterPIDController = new PIDshooterController(hardwareMap,telemetry);
        /////////////// END OF ADDED BY MARC ////////


        // Expansion Hub + ControlHub Hardware Mapping
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        controlHub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");

        // Motor Mapping
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        intake = hardwareMap.get(DcMotor.class, "intake");
        s1 = hardwareMap.get(DcMotor.class, "s1"); // maybe delete?
        s2 = hardwareMap.get(DcMotor.class, "s2"); // same


        // Servo Mapping
        shooterServo = hardwareMap.get(Servo.class, "angle");
        pusherServo = hardwareMap.servo.get("pusherServo");
        liftServo = hardwareMap.get(Servo.class, "lift4");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servo Setup
        shooterServo.setDirection(Servo.Direction.REVERSE);
        liftServo.setDirection(Servo.Direction.FORWARD);
        liftServo.setPosition(1);
        pusherServo.setPosition(0.05);
        //threadPIDcontroller.setPriority(9);

        // RoadRunner
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(-61, -27, 0);
        //Pose2d startPose = new Pose2d(12, -6, 0);
        drive.setPoseEstimate(startPose);

        // RGB TIME
        expansionHub.setLedColor(255, 0, 255);
        controlHub.setLedColor(255, 0, 255);

        // Limb States
        lJeffState = lJeffStates.UP;
        rJeffState = rJeffStates.IN;
        armState = armStates.OUT;

       // threadPIDcontroller.start();

        // Threading / Asynchronous Code  / Multithreading
        Thread arm = new Thread(() -> {
            intakeOn = false;
            //s1.setPower(-1);
            //s2.setPower(-1);
            liftServo.setPosition(0.86);
            robotArmed = true;
            //isPIDenabled = true;
        });

        Thread disarm = new Thread(() -> {
            //s1.setPower(0);
            //s2.setPower(0);
            liftServo.setPosition(1);
            robotArmed = false;
            isPIDenabled = false;
        });


        Thread shootAsync = new Thread(() -> {
            pusherServo.setPosition(0.35);
            sleep(300);
            pusherServo.setPosition(0.05);
            shootingAsync = false;
        });
        /////////// ADDED BY MARC //////////
        int timing1 = 200;
        int timing2 = 700;
        /////////// END OF ADDED BY MARC ///
        Thread shootThreeAsync = new Thread(() -> {
            shootingAsync = true;
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
            shootingAsync = false;
        });

        shooterServo.setPosition(0.455);
        angleState currentAngleState = angleState.BSHOT;


        lJeff = hardwareMap.servo.get("lJeff");
        rJeff = hardwareMap.servo.get("rJeff");
        lJeffUp.run();


        waitForStart();

        while (opModeIsActive()) {




            //shooterPIDController.printTelemetry();
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            if(isRunnningShooter) {
                isPIDenabled = (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && robotArmed);
                if(!isPIDenabled && robotArmed) shooterPIDController.setPowShooter(0.7);

                if(isPIDenabled)
                {
                    try {
                        shooterPIDController.doPIDLoop(2200);
                    }
                    catch (Exception e)
                    {
                        telemetry.addData("hey!","initialize PID");
                    }
                }
            }
            else
                shooterPIDController.setPowShooter(0);



            if(gamepad1.x && !pressingX) {
                pressingX = true;
            } else if (!gamepad1.x && pressingX) {

                if(!robotArmed) {
                    isRunnningShooter = true;
                    arm.start();
                } else {
                    isRunnningShooter = false;
                    disarm.start();
                }
                pressingX = false;
            }


            if(gamepad1.left_bumper && !pressingLeftBumper) {
                pressingLeftBumper = true;
            } else if (!gamepad1.left_bumper && pressingLeftBumper) {
                intake.setDirection(intake.getDirection().inverted());
                pressingLeftBumper = false;
            }

            if(gamepad1.right_trigger > 0.3) {

            } else {

            }

            if(gamepad1.right_bumper && !pressingRightBumper) {
                pressingRightBumper = true;
            } else if (!gamepad1.right_bumper && pressingRightBumper) {
                shootThreeAsync.start();
                pressingRightBumper = false;
            }


            // TODO Button B -----------------------------------------------------------------------
            if (gamepad1.b && !pressingB) {
                pressingB = true;
            } else if (pressingB && !gamepad1.b) {
                switch(currentAngleState) {
                    case PSHOT:
                        shooterServo.setPosition(RegularAngle);
                        currentAngleState = angleState.BSHOT;
                        break;
                    case BSHOT:
                        shooterServo.setPosition(PShotAngle);
                        currentAngleState = angleState.PSHOT;
                        break;
                }
                pressingB = false;
            }
            // TODO Intake Logic -------------------------------------------------------------------
            if(!robotArmed) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
                intakeOn = false;
            }
            // -------------------------------------------------------------------------------------


            if(gamepad1.a && !pressingA) {
                pressingA = true;
            } else if (!gamepad1.a && pressingA) {
                if(!shootingAsync) {
                    shootAsync.start();
                }
                pressingA = false;
            }


            // Reserved For Wobble Lifting And Holding
            if(gamepad1.y && !pressingY) {
                pressingY = true;
            } else if (!gamepad1.y && pressingY) {
                if(rJeffState == rJeffStates.UP && lJeffState == lJeffStates.UP) {
                    rJeffUp.run();
                    lJeffUp.run();
                } else if(rJeffState == rJeffStates.DOWN && lJeffState == lJeffStates.DOWN) {
                    rJeffDown.run();
                    lJeffDown.run();
                }
                pressingY = false;
            }


            // Arm Up and Down
            if(gamepad1.dpad_up && !pressingUp) {
                pressingUp = true;
            } else if(!gamepad1.dpad_up && pressingUp) {
                if(rJeffState == rJeffStates.UP || rJeffState == rJeffStates.DOWN) { // If the left blocker is down or up
                    if(armState == armStates.IN) {
                        rJeffIn.run();
                        rJeffIn.join();
                        sleep(2000);
                        armAndGrabberController.goToPosArm(120, true);
                    }
                } else if(rJeffState == rJeffStates.IN) { // If the left blocker is inside
                    if(armState == armStates.IN) {
                        armAndGrabberController.goToPosArm(120, true);
                    }
                }
                pressingUp = false;
            }

            if(gamepad1.dpad_down && !pressingDown) {
                pressingDown = true;
            } else if(!gamepad1.dpad_down && pressingDown) {
                if(rJeffState == rJeffStates.UP || rJeffState == rJeffStates.DOWN) {  // If the left blocker is down or up
                    if(armState == armStates.IN) {
                        rJeffIn.run();
                        rJeffIn.join();
                        sleep(2000);
                        armAndGrabberController.goToPosArm(0, false);
                    }
                } else if(rJeffState == rJeffStates.IN) { // If the left blocker is inside
                    if(armState == armStates.IN) {
                        armAndGrabberController.goToPosArm(120, false);
                    }
                }
                pressingDown = false;
            }



            if(gamepad1.left_trigger > 0.3 && !qLT)
            {
                qLB = true;
                turnLeft(10);
            }
            else if(gamepad1.left_trigger <= 0.3 && qLT)
            {
                qLB = false;
            }



            // Intense Shooter Angle Adjust - D-Pad Left + Right --------------------------------------------
            if (gamepad1.dpad_left) {
                shooterServo.setPosition(Range.clip(shooterServo.getPosition() + .04, 0, 1));
            } else if (gamepad1.dpad_right) {
                shooterServo.setPosition(Range.clip(shooterServo.getPosition() - .04, 0, 1));
            }
            // Intense Shooter Angle Controller End ---------------------------------------------------------


            // Telemetry ------------------------------------------------------------------------------------
            telemetry.addData("Shot State ", currentAngleState);
            telemetry.addData("POS", shooterServo.getPosition());
            telemetry.addData("Robot Armed", robotArmed);
            telemetry.addData("s1", s1.getPower());
            telemetry.addData("s2", s2.getPower());
            telemetry.update();
            drive.update();
            // Telemetry End --------------------------------------------------------------------------------

        }
        // after loop, when stop button is hit
        ////////// ADDED BY MARC //////////
        isPIDenabled = false;
        ////////// END ADDED BY MARC //////

    }



    private void turnLeft(double deg) {
        //Pose2d current = drive.getPoseEstimate();
        //drive.setPoseEstimate(new Pose2d(current.getX(),current.getY(),3));
        double iniAngle = drive.getPoseEstimate().getHeading();
        if(iniAngle > 180)
            iniAngle -= 360;
        double finAngle = (iniAngle + Math.toRadians(deg));
        while(drive.getPoseEstimate().getHeading() > finAngle)
        {
            rf.setPower(0.3);
            rb.setPower(-0.3);
            lf.setPower(0.3);
            lb.setPower(-0.3);
            drive.update();
            telemetry.addData("turning","left");
            telemetry.addData("heading",Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.update();
        }
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
    }

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

    public double rJeffUpPos = .3; // was .49
    public double rJeffDownPos = .66; // was 0.149
    public double rJeffInPos = .059; // was .219
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