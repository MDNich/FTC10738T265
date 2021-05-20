package org.sbs.bears.ftc.robot.controller;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.ftc.util.RobotSubsytemManager;


public class BaseMotorManager extends RobotSubsytemManager {

    // Motors + Servo Declarations
    //public DcMotor lf;
    //public DcMotor rf;
    //public DcMotor lb;
    //public DcMotor rb;
    public DcMotor s1;
    public DcMotor s2;

    public BaseMotorManager(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        //lf = hardwareMap.get(DcMotor.class, "lf");
        // rf = hardwareMap.get(DcMotor.class, "rf");
        //lb = hardwareMap.get(DcMotor.class, "lb");
        //rb = hardwareMap.get(DcMotor.class, "rb");
        s1 = hardwareMap.get(DcMotor.class, "s1");
        s2 = hardwareMap.get(DcMotor.class, "s2");
        // Motor Setup
        //lf.setDirection(DcMotorSimple.Direction.REVERSE);
        //rb.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void shutDown() {
        stopLauncher();
    }

    /*public void turnRPow(double pow)
    {
        rf.setPower(-pow);
        rb.setPower(-pow);
        lf.setPower(pow);
        lb.setPower(pow);
    }
    public void turnLPow(double pow)
    {
        rf.setPower(pow);
        rb.setPower(pow);
        lf.setPower(-pow);
        lb.setPower(-pow);
    }
    public void stopMotors()
    {
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
    }
*/
    public void stopLauncher()
    {
        s1.setPower(0);
        s2.setPower(0);
    }

    public void startLauncher()
    {
        s1.setPower(-1);
        s2.setPower(-1);
    }
    public void setLauncherPower(double power)
    {
        s1.setPower(power);
        s2.setPower(power);
    }

    public void reverseLauncher()
    {
        s1.setPower(1);
        s2.setPower(1);
    }
    public double s1Power()
    {
        return s1.getPower();
    }
    public double s2Power()
    {
        return s2.getPower();
    }




}
