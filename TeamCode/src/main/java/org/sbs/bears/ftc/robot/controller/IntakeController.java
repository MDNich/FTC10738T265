package org.sbs.bears.ftc.robot.controller;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.ftc.util.RobotSubsytemManager;

public class IntakeController extends RobotSubsytemManager {
    DcMotor intake;
    public IntakeController(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
        intake = hwMap.get(DcMotor.class, "intake");
    }

    public void setIntakePower()
    {
        intake.setPower(-1);
    }

    public void setIntakePower(double power)
    {
        intake.setPower(-power);
    }

    public double getIntakePower()
    {
        return intake.getPower();
    }


    @Override
    public void shutDown() {
        intake.setPower(0);
    }
}
