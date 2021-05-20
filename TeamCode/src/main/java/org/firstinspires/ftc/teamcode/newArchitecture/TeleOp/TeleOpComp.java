package org.firstinspires.ftc.teamcode.newArchitecture.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.newArchitecture.Autonomous.Positions;
import org.sbs.bears.ftc.robot.Robot;
import org.sbs.bears.ftc.robot.controller.ArmAndGrabberManager;
import org.sbs.bears.ftc.robot.controller.BlockerController;
import org.sbs.bears.ftc.robot.controller.PIDarmController;
import org.sbs.bears.ftc.robot.controller.RRDriveControllerNoOdom;
import org.sbs.bears.ftc.robot.controller.RingSubsytemController;
import org.sbs.bears.ftc.robot.lib.ShootingModes;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

@TeleOp(name="TeleOp Official", group="Linear Opmode")
public class TeleOpComp extends LinearOpMode {

    Robot theRobot;
    ArmAndGrabberManager armCtrl;
    RRDriveControllerNoOdom rrCtrl;
    RingSubsytemController ringCtrl;
    BlockerController blockerCtrl;


    PIDarmController PIDarmThread;
    private boolean pressingX;

    private boolean intakeOn;
    private boolean pressingLeftBumper;
    private boolean pressingRightBumper;
    private boolean pressingB;
    private boolean pressingA;
    private boolean pressingY;
    private boolean pressingDown;
    private boolean pressingLeftDpad;
    private boolean qLT;
    private boolean pressingRightDpad;
    private boolean qLB;
    private boolean robotArmed;
    private boolean qRT;
    private boolean qReversed;


    @Override
    public void runOpMode() throws InterruptedException {
        theRobot = new Robot(hardwareMap,telemetry);
        armCtrl = theRobot.armCtrl;
        rrCtrl = theRobot.rrCtrlNoOdom;
        ringCtrl = theRobot.ringCtrl;
        blockerCtrl = theRobot.blockerCtrl;
        PIDarmThread = new PIDarmController(armCtrl,240);

        try {
            getCurrentPosFromFile();
        } catch (FileNotFoundException e) {
            telemetry.addData("Could not read from file.","Using default position");
            rrCtrl.setCurrentPos(new Pose2d());
        }
        ringCtrl.setAngleToTeleOpGSHOT();


        waitForStart();
        blockerCtrl.lJeffUp();
        ringCtrl.setRingManagementMode(ShootingModes.INTAKING);

        while(opModeIsActive())
        {
            rrCtrl.doGamepadDriving(gamepad1);

            if(gamepad1.left_trigger > 0.3 && !qLT )
            {
                qLB = true;
                if(gamepad1.right_bumper)
                    rrCtrl.drive.turn(Math.toRadians(8));
            }
            else if(gamepad1.left_trigger <= 0.3 && qLT)
            {
                qLB = false;
            }


            if(gamepad1.x && !pressingX) {
                pressingX = true;
            } else if (!gamepad1.x && pressingX) {

                if(!robotArmed) {
                    ringCtrl.setRingManagementMode(ShootingModes.SHOOTING);

                } else {
                    ringCtrl.launchCtrl.shutDown();
                    ringCtrl.setRingManagementMode(ShootingModes.INTAKING);

                }
                robotArmed = !robotArmed;
                pressingX = false;
            }

            if(gamepad1.left_bumper && !pressingLeftBumper) {
                pressingLeftBumper = true;
            } else if (!gamepad1.left_bumper && pressingLeftBumper) {
                if(!qReversed) {
                    qReversed = true;
                    ringCtrl.setRingManagementMode(ShootingModes.REVERSE_INTAKE);
                }
                else {
                    qReversed = false;
                    ringCtrl.setRingManagementMode(ShootingModes.INTAKING);
                }

                pressingLeftBumper = false;
            }

            if(gamepad1.right_trigger > 0.3 && !qRT) {
                qRT = true;
                goToShootPos();
            } else if (gamepad1.right_trigger <= 0.3 && qRT) {
                qRT = false;
            }

            if(gamepad1.right_bumper && !pressingRightBumper) {
                pressingRightBumper = true;
            } else if (!gamepad1.right_bumper && pressingRightBumper) {
                if(ringCtrl.getRingManagementMode().equals(ShootingModes.SHOOTING))
                    new Thread(()->{
                        ringCtrl.shootManyRings(3);
                    }).start();
                pressingRightBumper = false;
            }

            if (gamepad1.b && !pressingB) {
                pressingB = true;
            } else if (pressingB && !gamepad1.b) {
                ringCtrl.applyAngleStateChange(); // will flip automatically
                pressingB = false;
            }

            if(gamepad1.a && !pressingA) {
                pressingA = true;
            } else if (!gamepad1.a && pressingA) {
                new Thread(()->{
                    ringCtrl.shootOneRing();
                }).start();
                pressingA = false;
            }

            if(gamepad1.y && !pressingY) {
                pressingY = true;
            } else if (!gamepad1.y && pressingY) {
                armCtrl.toggleClaw();
                pressingY = false;
            }

            if(gamepad1.dpad_down && !pressingDown) {
                pressingDown = true;
            } else if(!gamepad1.dpad_down && pressingDown) {
                blockerCtrl.toggleJeffs();
                pressingDown = false;
            }

            if(gamepad1.dpad_up) {
                blockerCtrl.checkRJeff();
                armCtrl.armMotor.setPower(0.3);
            }
            else if(gamepad1.right_trigger > 0.3)
                armCtrl.armMotor.setPower(-0.5);
            else
                armCtrl.armMotor.setPower(0);

            if(gamepad1.left_trigger > 0.3 && !qLT && pressingRightBumper){

            }

            // Intense Shooter Angle Adjust - D-Pad Left + Right --------------------------------------------
            if (gamepad1.dpad_left && !pressingLeftDpad) {
                pressingLeftDpad = true;
            } else if (!gamepad1.dpad_left && pressingLeftDpad) {
                ringCtrl.incrementAngle();
            }


            if (gamepad1.dpad_right && !pressingRightDpad) {
                pressingRightDpad = true;
            } else if (!gamepad1.dpad_right && pressingRightDpad) {
                ringCtrl.decrementAngle();
            }
            // Intense Shooter Angle Controller End ---------------------------------------------------------


            // Telemetry ------------------------------------------------------------------------------------
            telemetry.addData("Shot State ", ringCtrl.angleState);
            telemetry.addData("POS", ringCtrl.getAngle());
            telemetry.addData("Robot State", ringCtrl.getRingManagementMode());
            telemetry.addData("s1", ringCtrl.s1Power());
            telemetry.addData("s2", ringCtrl.s2Power());
            telemetry.update();



        }

    }

    private void goToShootPos() {
        rrCtrl.doLineToSpline(rrCtrl.getCurrentPos(),Positions.teleOpShootPos); // beta
    }

    private void getCurrentPosFromFile() throws FileNotFoundException {
        //TODO: write file input.
        Pose2d posFromAuton;
        double x, y, h;
        ArrayList<String> arr = new ArrayList<>(3);
        File file = new File("../PosForTeleOp.txt");
        Scanner myReader = new Scanner(file);
        while (myReader.hasNextLine()) {
            String data = myReader.nextLine();
            arr.add(data);
        }
        x = Double.parseDouble(arr.get(0));
        y = Double.parseDouble(arr.get(1));
        h = Double.parseDouble(arr.get(2));
        posFromAuton = new Pose2d(x,y,h);
        rrCtrl.setCurrentPos(posFromAuton);

    }
}
