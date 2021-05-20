package org.sbs.bears.ftc.robot.controller;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.ftc.util.RobotSubsytemManager;

public class DriveAngleController extends RobotSubsytemManager {

    RRDriveController driveCtrl;
    BaseMotorManager motorCtrl;

    public DriveAngleController(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
        driveCtrl = new RRDriveController(hwMap,telemetry);
        motorCtrl = new BaseMotorManager(hwMap,telemetry);
    }

    @Override
    public void shutDown() {
        // nothing
    }

    public void turn(double deg)
    {
        double iniHeading = driveCtrl.getCurrentPos().getHeading();
        double finHeading = iniHeading + Math.toRadians(deg);
        boolean isRightTurn = finHeading < iniHeading;
       /* if(isRightTurn)
        {
            while(driveCtrl.getCurrentPos().getHeading() > finHeading)
            {
                motorCtrl.turnRPow(0.3);
            }
        }
        else {
            while(driveCtrl.getCurrentPos().getHeading() < finHeading)
            {
                motorCtrl.turnRPow(-0.3);
            }
        }*/
        //motorCtrl.stopMotors();
    }
}
