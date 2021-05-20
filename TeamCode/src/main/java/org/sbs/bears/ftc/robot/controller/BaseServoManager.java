package org.sbs.bears.ftc.robot.controller;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Sleep;
import org.sbs.bears.ftc.constants.RingLaunchConstants;
import org.sbs.bears.ftc.constants.RingLaunchSetting;
import org.sbs.bears.ftc.util.RobotSubsytemManager;

import static java.lang.Thread.sleep;

public class BaseServoManager extends RobotSubsytemManager {

    public Servo shooterServo;
    public Servo liftServo;
    public Servo pusherServo;
    int timing1 = 200;
    int timing2 = 700;

    public BaseServoManager(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        shooterServo = hardwareMap.get(Servo.class, "angle");
        pusherServo = hardwareMap.servo.get("pusherServo");
        liftServo = hardwareMap.get(Servo.class, "lift4");

        shooterServo.setDirection(Servo.Direction.REVERSE);
        liftServo.setDirection(Servo.Direction.FORWARD);
        liftServo.setPosition(new RingLaunchConstants().getValue(RingLaunchSetting.LAUNCH_BAY_INACTIVE));
        pusherServo.setPosition(new RingLaunchConstants().getValue(RingLaunchSetting.PUSHER_INACTIVE));
    }

    @Override
    public void shutDown() {
        putBayDown();
    }

    public void setLaunchAngle(double servoValue)
    {
        shooterServo.setPosition(servoValue);
    }

    public void pushOneRing() {
        pusherServo.setPosition(new RingLaunchConstants().getValue(RingLaunchSetting.PUSHER_ACTIVE));
        Sleep.sleep(timing1); // variablized
        pusherServo.setPosition(new RingLaunchConstants().getValue(RingLaunchSetting.PUSHER_INACTIVE));
        Sleep.sleep(timing2); // variablized

    }
    public void pushManyRings(int numRings) {
        for(int i = 0; i < numRings; i++) {
           pushOneRing();
           Sleep.sleep(300);
        }
    }
    public void liftBayUp()
    {
        liftServo.setPosition(new RingLaunchConstants().getValue(RingLaunchSetting.LAUNCH_BAY_ACTIVE));
    }
    public void putBayDown()
    {
        liftServo.setPosition(new RingLaunchConstants().getValue(RingLaunchSetting.LAUNCH_BAY_INACTIVE)); // its ok even if it is there already
    }

    public void setAngleToTeleOpPSHOT()
    {
        setLaunchAngle(new RingLaunchConstants().getValue(RingLaunchSetting.TELEOP_PSHOT));
    }
    public void setAngleToTeleOpGSHOT()
    {
        setLaunchAngle(new RingLaunchConstants().getValue(RingLaunchSetting.TELEOP_GOALSHOT));
    }

    public void incrementAngle()
    {
        shooterServo.setPosition(shooterServo.getPosition() + 0.01);
    }
    public void decrementAngle()
    {
        shooterServo.setPosition(shooterServo.getPosition() - 0.01);
    }


}
