package org.sbs.bears.ftc.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotUtil {

    protected HardwareMap hwMap;
    protected Telemetry telemetry;
    public RobotUtil(HardwareMap hwMap, Telemetry telemetry)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }
}
