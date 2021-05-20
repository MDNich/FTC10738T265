package org.sbs.bears.ftc.robot.controller;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.ftc.robot.lib.ShootingModes;
import org.sbs.bears.ftc.util.RobotSubsytemManager;

public class RingSubsytemController extends RobotSubsytemManager {


    public enum ANGLE_STATE {
        PSHOT,
        GSHOT;
    }

    public LaunchController launchCtrl;
    public IntakeController intakeCtrl;
    public ShootingModes ringManagementMode;
    public ANGLE_STATE angleState;
    public RingSubsytemController(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
        launchCtrl = new LaunchController(hwMap,telemetry);
        intakeCtrl = new IntakeController(hwMap,telemetry);
        angleState = ANGLE_STATE.GSHOT;
        applyRingManagementMode(ShootingModes.IDLE);

    }
    public void shootOneRing() {
        launchCtrl.shootOneRing();
    }
    public void shootManyRings(int num)
    {
        ShootingModes prevMode = ringManagementMode;
        applyRingManagementMode(ShootingModes.SHOOTING);
        launchCtrl.shootManyRings(num);
        applyRingManagementMode(prevMode);
    }
    public void setRingManagementMode(ShootingModes ringManagementMode)
    {
        applyRingManagementMode(ringManagementMode);
    }

    public ShootingModes getRingManagementMode()
    {
        return this.ringManagementMode;
    }

    public void applyAngleStateChange(ANGLE_STATE angle_state)
    {
        this.angleState = angle_state;
        switch (angleState)
        {
            case GSHOT:
                launchCtrl.setAngleToTeleOpGSHOT();
                return;
            case PSHOT:
                launchCtrl.setAngleToTeleOpPSHOT();
                return;
        }
    }
    public void applyAngleStateChange()
    {
        if(angleState == ANGLE_STATE.PSHOT)
        {
            applyAngleStateChange(ANGLE_STATE.GSHOT);
        }
        else
            applyAngleStateChange(ANGLE_STATE.PSHOT);
    }

    private void applyRingManagementMode(ShootingModes ringManagementMode)
    {
        this.ringManagementMode = ringManagementMode;
        switch (ringManagementMode)
        {
            case IDLE:
                launchCtrl.stopShooter();
                intakeCtrl.shutDown();
                return;
            case INTAKING:
                launchCtrl.setShootPower(-0.3);
                intakeCtrl.setIntakePower();
                return;
            case SHOOTING:
                intakeCtrl.shutDown();
                launchCtrl.prepShooter();
                return;
            case REVERSE_INTAKE:
                intakeCtrl.setIntakePower(-1);
                launchCtrl.shutDown();
                return;
        }
    }



    @Override
    public void shutDown() {
        applyRingManagementMode(ShootingModes.IDLE);
    }

    public void setAngleToTeleOpPSHOT()
    {
        launchCtrl.setAngleToTeleOpPSHOT();
    }
    public void setAngleToTeleOpGSHOT()
    {
        launchCtrl.setAngleToTeleOpGSHOT();
    }
    public void incrementAngle()
    {
        launchCtrl.incrementAngle();
    }
    public void decrementAngle()
    {
        launchCtrl.decrementAngle();
    }

    public double s1Power()
    {
        return launchCtrl.s1Power();
    }

    public double s2Power()
    {
        return launchCtrl.s2Power();
    }

    public double getAngle()
    {
        return launchCtrl.getAngle();
    }

}

