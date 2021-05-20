package org.sbs.bears.ftc.robot.controller;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.ftc.util.RobotSubsytemManager;

public class PathPlanner extends RobotSubsytemManager {
    public PathPlanner(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
    }

    @Override
    public void shutDown() {

    }

}
