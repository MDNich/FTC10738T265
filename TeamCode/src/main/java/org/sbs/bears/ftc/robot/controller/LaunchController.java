package org.sbs.bears.ftc.robot.controller;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.ftc.constants.RingLaunchConstants;
import org.sbs.bears.ftc.constants.RingLaunchSetting;
import org.sbs.bears.ftc.util.RobotSubsytemManager;
import org.sbs.bears.ftc.exceptions.InvalidNumberOfRingsToShootException;


/**
 * This class will control the launcher for the Ultimate Goal Season.
 */
public class LaunchController extends RobotSubsytemManager {
    private BaseMotorManager baseMotorController;
    private BaseServoManager baseServoController;


    public LaunchController(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
        baseMotorController = new BaseMotorManager(hwMap,telemetry);
        baseServoController = new BaseServoManager(hwMap,telemetry);
    }
    public void prepAndShootRings(int num) throws InvalidNumberOfRingsToShootException {
        if (num <= 0)
            throw new InvalidNumberOfRingsToShootException();
        prepShooter();
        baseServoController.pushManyRings(num);
        stopShooter();
    }
    public void prepShooter()
    {
        baseMotorController.startLauncher();
        baseServoController.liftBayUp();
    }
    public void prepShooter(RingLaunchSetting setting)
    {
        baseMotorController.startLauncher();
        baseServoController.liftBayUp();
        baseServoController.setLaunchAngle(new RingLaunchConstants().getValue(setting));
    }
    public void stopShooter()
    {
        baseServoController.shutDown();
        baseMotorController.stopLauncher();
    }
    public void shootOneRing()
    {
        baseServoController.pushOneRing();
    }
    public void shootManyRings(int num)
    {
        baseServoController.liftBayUp();
        baseServoController.pushManyRings(num);
        baseServoController.putBayDown();
    }
    public void setShootPower(double power)
    {
        baseMotorController.setLauncherPower(power);
    }

    public void setAngleToTeleOpPSHOT()
    {
        baseServoController.setAngleToTeleOpPSHOT();
    }
    public void setAngleToTeleOpGSHOT()
    {
        baseServoController.setAngleToTeleOpGSHOT();
    }
    public double getAngle()
    {
        return baseServoController.shooterServo.getPosition();
    }

    public void incrementAngle()
    {
        baseServoController.incrementAngle();
    }
    public void decrementAngle()
    {
        baseServoController.decrementAngle();
    }

    public double s1Power()
    {
        return baseMotorController.s1Power();
    }

    public double s2Power()
    {
        return baseMotorController.s2Power();
    }

    @Override
    public void shutDown() {
        stopShooter();
    }
}
