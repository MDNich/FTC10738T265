package org.sbs.bears.ftc.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class RobotSubsytemManager {

    public HardwareMap hwMap;
    public Telemetry telemetry;

    public RobotSubsytemManager(HardwareMap hwMap, Telemetry telemetry)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }

    public abstract void shutDown();

}
