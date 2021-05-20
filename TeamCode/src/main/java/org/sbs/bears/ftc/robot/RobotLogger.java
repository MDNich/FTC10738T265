package org.sbs.bears.ftc.robot;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotUtil;

public class RobotLogger extends RobotUtil {

    private static Telemetry telemetry;
    private static HardwareMap hwMap;

    public RobotLogger(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap,telemetry);
    }

    public static void logToTelemetry(String messageContent, Object messageValue) {
        telemetry.addData(messageContent, messageValue);
    }

    public static void logToDebugger(String tag, String message) {
        Log.w(tag, message);
    }

    public static void flush() {
        telemetry.update();
    }

}
