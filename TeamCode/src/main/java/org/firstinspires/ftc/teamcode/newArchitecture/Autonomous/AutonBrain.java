package org.firstinspires.ftc.teamcode.newArchitecture.Autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.ftc.robot.Robot;
import org.sbs.bears.ftc.robot.controller.ArmAndGrabberManager;
import org.sbs.bears.ftc.robot.controller.CameraController;
import org.sbs.bears.ftc.robot.controller.PIDarmController;
import org.sbs.bears.ftc.robot.controller.RRDriveControllerNoOdom;
import org.sbs.bears.ftc.robot.controller.RingSubsytemController;
import org.sbs.bears.ftc.robot.lib.ShootingModes;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.atomic.AtomicBoolean;

public class AutonBrain {
    Robot robot;
    private boolean qHoldingArm;

    PIDarmController PIDarm;

    enum DRIVER_STATE {
        SHOOTING_PSHOT,
        SHOOTING_GOAL,
        DROPPING_WOBBLE,
        PICKING_WOBBLE_UP,
        DRIVING_RR,
        DRIVING_MANUAL,
        STOPPED
    }
    enum CURRENT_COMMAND {
        PREPARING,
        MEASURING_RINGS,
        DOING_PSHOT,
        MOVING_TO_PICKUP,
        PICKING_UP_AND_SHOOTING,
        MOVING_TO_WOBBLE1_DROPOFF,
        DROP_WOBBLE_1,
        MOVING_TO_WOBBLE2_PICKUP,
        PICKUP_WOBBLE_2,
        MOVING_TO_WOBBLE2_DROPOFF,
        DROP_WOBBLE_2,
        MOVING_TO_ENDPOS,
        DONE
    }

    CameraController camCtrl;
    ArmAndGrabberManager armCtrl;
    RRDriveControllerNoOdom RRctrl;
    RingSubsytemController ringCtrl;
    static CURRENT_COMMAND brainState;
    static DRIVER_STATE driveState;
    HardwareMap hardwareMap;
    Telemetry telemetry;


    protected int numRings;
    public AutonBrain(HardwareMap hardwareMap, Telemetry telemetry)
    {
        robot = new Robot(hardwareMap,telemetry);
        camCtrl = new CameraController(hardwareMap,telemetry);
        armCtrl = robot.armCtrl;
        //RRctrl = robot.rrCtrlNoOdom;
        ringCtrl = robot.ringCtrl;
        brainState = CURRENT_COMMAND.PREPARING;
        driveState = DRIVER_STATE.STOPPED;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        PIDarm = new PIDarmController(armCtrl,240);
        telemetry.addData("Ready","Ready");
        telemetry.update();

    }

    public void doBrainTask()
    {
        updateTelemetry();
        switch(brainState) {
            case PREPARING:
                driveState = DRIVER_STATE.STOPPED;
                doPrepareTasks();
                brainState = CURRENT_COMMAND.MEASURING_RINGS;
                return;
            case MEASURING_RINGS:
                driveState = DRIVER_STATE.STOPPED;
                measureRings();
                brainState = CURRENT_COMMAND.DOING_PSHOT;
                return;
            case DOING_PSHOT:
                driveState = DRIVER_STATE.SHOOTING_PSHOT;
                doPShot();
                if(numRings != 0) {
                    brainState = CURRENT_COMMAND.MOVING_TO_PICKUP;
                }
                else {
                    brainState = CURRENT_COMMAND.MOVING_TO_WOBBLE1_DROPOFF;
                }
                return;
            case MOVING_TO_PICKUP:
                driveState = DRIVER_STATE.DRIVING_RR;
                doMoveToPickup();
                brainState = CURRENT_COMMAND.PICKING_UP_AND_SHOOTING;
                return;
            case PICKING_UP_AND_SHOOTING:
                driveState = DRIVER_STATE.DRIVING_RR;
                pickupAndShootRings();
                brainState = CURRENT_COMMAND.MOVING_TO_WOBBLE1_DROPOFF;
                return;
            case MOVING_TO_WOBBLE1_DROPOFF:
                driveState = DRIVER_STATE.DRIVING_RR;
                moveToDropWobbleOne();
                brainState = CURRENT_COMMAND.DROP_WOBBLE_1;
                return;
            case DROP_WOBBLE_1:
                driveState = DRIVER_STATE.DROPPING_WOBBLE;
                dropWobbleOne();
                brainState = CURRENT_COMMAND.MOVING_TO_WOBBLE2_PICKUP;
                return;
            case MOVING_TO_WOBBLE2_PICKUP:
                driveState = DRIVER_STATE.DRIVING_RR;
                moveToPickupWobbleTwo();
                brainState = CURRENT_COMMAND.PICKUP_WOBBLE_2;
                return;
            case PICKUP_WOBBLE_2:
                driveState = DRIVER_STATE.PICKING_WOBBLE_UP;
                pickupWobbleTwo();
                brainState = CURRENT_COMMAND.MOVING_TO_WOBBLE2_DROPOFF;
                break;
            case MOVING_TO_WOBBLE2_DROPOFF:
                driveState = DRIVER_STATE.DRIVING_RR;
                moveToDropWobbleTwo();
                brainState = CURRENT_COMMAND.DROP_WOBBLE_2;
                return;
            case DROP_WOBBLE_2:
                driveState = DRIVER_STATE.DROPPING_WOBBLE;
                dropWobbleTwo();
                brainState = CURRENT_COMMAND.MOVING_TO_ENDPOS;
                return;
            case MOVING_TO_ENDPOS:
                driveState = DRIVER_STATE.DRIVING_RR;
                moveToEndPos();
                storeEndPosInFile();
                brainState = CURRENT_COMMAND.DONE;
                return;
            case DONE:
                driveState = DRIVER_STATE.STOPPED;
                telemetry.addData("DONE","");
                telemetry.update();
                return;

        }
    }

    private void updateTelemetry()
    {
        telemetry.addData("Task",brainState);
        telemetry.addData("Driving State",driveState);
        telemetry.update();
    }

    private void doPrepareTasks() {
        RRctrl.setCurrentPos(Positions.startingPosition);
    }


    private void measureRings() {
        try {
            numRings = camCtrl.readNumRings();
        } catch (InterruptedException e) {
            telemetry.addData("Failed"," to read ringNum");
            telemetry.update();
        }
    }

    private void doPShot() {
        prepShooter();
        for(int i = 0; i < 3; i++)
        {
            shootOne();
            strafeLeftPShot();
        }
        stopShooter();
    }

    private void stopShooter() {
        ringCtrl.setRingManagementMode(ShootingModes.IDLE);
    }

    private void strafeLeftPShot() {
        RRctrl.doStrafeLeft(RRctrl.getCurrentPos(),7);
    }

    private void shootOne() {
        ringCtrl.shootOneRing();
    }

    private void prepShooter() {
        ringCtrl.setRingManagementMode(ShootingModes.SHOOTING);
    }

    private void doMoveToPickup() {
        RRctrl.doLineToSpline(RRctrl.getCurrentPos(),Positions.oneRingPickupPosition);
    }

    private void pickupAndShootRings() {
        pickupAndShootRings(numRings);
    }

    private void pickupAndShootRings(int numRings)
    {
        driveState = DRIVER_STATE.DRIVING_RR;
        switch(numRings)
        {

            case 1:
                ringCtrl.setRingManagementMode(ShootingModes.INTAKING);
                updateTelemetry();
                RRctrl.doForward(RRctrl.getCurrentPos(),7);
                driveState = DRIVER_STATE.SHOOTING_GOAL;
                updateTelemetry();
                ringCtrl.setRingManagementMode(ShootingModes.SHOOTING);
                brainSleep(1000);
                ringCtrl.shootManyRings(2);
                break;
            case 4:
                for(int i = 0; i < 2; i++) {
                    driveState = DRIVER_STATE.DRIVING_RR;
                    ringCtrl.setRingManagementMode(ShootingModes.INTAKING);
                    updateTelemetry();
                    RRctrl.doForward(RRctrl.getCurrentPos(), 5);
                    driveState = DRIVER_STATE.SHOOTING_GOAL;
                    ringCtrl.setRingManagementMode(ShootingModes.SHOOTING);
                    updateTelemetry();
                    brainSleep(1000);
                    ringCtrl.shootManyRings(2);
                }
                break;
        }
        ringCtrl.setRingManagementMode(ShootingModes.IDLE);
        driveState = DRIVER_STATE.STOPPED;
        updateTelemetry();
    }




    private void pickupWobbleTwo() {
        armCtrl.closeClaw();
        PIDarm.start();
    }

    private void moveToDropWobbleTwo() {
        switch(numRings) {
            case 0:
                RRctrl.doLineToSpline(RRctrl.getCurrentPos(),Positions.zeroRingsSecondWobbleDropPosition);
                break;
            case 1:
                RRctrl.doLineToSpline(RRctrl.getCurrentPos(),Positions.oneRingSecondWobbleDropPosition);
                break;

            case 2:
                RRctrl.doLineToSpline(RRctrl.getCurrentPos(),Positions.fourRingSecondWobbleDropPosition);
                break;
        }
    }

    private void storeEndPosInFile() {
        // todo store end position in file for teleOp
        FileWriter myWriter = null;
        try {
            myWriter = new FileWriter("../PosForTeleOp.txt"); // use ../ even if programming on windows because Android is Linux.
        } catch (IOException e) {
            e.printStackTrace();
        }
        double x = RRctrl.getCurrentPos().getX();
        double y = RRctrl.getCurrentPos().getY();
        double h = RRctrl.getCurrentPos().getHeading();
        try {
            myWriter.write(x + "\n" + y + "\n" + h); // will be three lines with x, y, heading on each line.
        } catch (IOException e) {
            e.printStackTrace();
        }
        try {
            myWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void moveToEndPos() {
        RRctrl.doLineToSpline(RRctrl.getCurrentPos(),Positions.endPosition);
    }

    private void dropWobbleTwo() {
        PIDarm.interrupt();
        armCtrl.openClaw();
        RRctrl.doBackward(RRctrl.getCurrentPos(),5);
        armCtrl.shutDown();

    }

    private void moveToPickupWobbleTwo() {
        RRctrl.doLineToSpline(RRctrl.getCurrentPos(),Positions.secondWobblePosition);
    }

    private void dropWobbleOne() {
        armCtrl.goToPosArm(130,true);
    }

    private void moveToDropWobbleOne() {
        switch(numRings) {
            case 0:
                RRctrl.doLineToSpline(RRctrl.getCurrentPos(),Positions.zeroRingsWobblePosition);
                break;
            case 1:
                RRctrl.doLineToSpline(RRctrl.getCurrentPos(),Positions.oneRingWobbleDropPosition);
                break;

            case 2:
                RRctrl.doLineToSpline(RRctrl.getCurrentPos(),Positions.fourRingWobbleDropPosition);
                break;
        }
    }



    public void brainSleep(long milisecs)
    {
        try {
            Thread.sleep(milisecs);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }





}
