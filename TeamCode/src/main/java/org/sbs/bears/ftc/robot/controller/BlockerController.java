package org.sbs.bears.ftc.robot.controller;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.ftc.util.RobotSubsytemManager;

public class BlockerController extends RobotSubsytemManager {

    private Servo lJeff;
    private Servo rJeff;

    public BlockerController(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
        lJeff = hwMap.servo.get("lJeff");
        rJeff = hwMap.servo.get("rJeff");
        lJeffState = lJeffStates.UP;
        rJeffState = rJeffStates.IN;
        armState = armStates.OUT;
    }

    @Override
    public void shutDown() {

    }


    public enum lJeffStates {
        UP, DOWN
    }

    public enum rJeffStates {
        UP, DOWN, IN
    }

    public enum armStates {
        IN, OUT
    }

    public lJeffStates lJeffState;
    public rJeffStates rJeffState;
    public armStates armState;

    public void setLBlockerState(lJeffStates lJeffState)
    {
        this.lJeffState = lJeffState;
    }
    public void setRBlockerState(rJeffStates rJeffState)
    {
        this.rJeffState = rJeffState;
    }
    public void setArmState(armStates armState)
    {
        this.armState = armState;
    }

    public lJeffStates getLBlockerState()
    {
        return this.lJeffState;
    }
    public rJeffStates getRBlockerState()
    {
        return this.rJeffState;
    }
    public armStates getArmState()
    {
        return this.armState;
    }


    public double lJeffUpPos = .49;
    public double lJeffDownPos = .149;
    public void lJeffUp() {
        lJeff.setPosition(lJeffUpPos);
        lJeffState = lJeffStates.UP;
    }

    public void lJeffDown() {
        lJeff.setPosition(lJeffDownPos);
        lJeffState = lJeffStates.DOWN;
    }

    public double rJeffUpPos = .3; // was .49
    public double rJeffDownPos = .66; // was 0.149
    public double rJeffInPos = .059; // was .219
    public void rJeffUp() {
        rJeff.setPosition(rJeffUpPos);
        rJeffState = rJeffStates.UP;
    }

    public void rJeffDown() {
        rJeff.setPosition(rJeffDownPos);
        rJeffState = rJeffStates.DOWN;
    }

    public void rJeffIn() {
        rJeff.setPosition(rJeffInPos);
        rJeffState = rJeffStates.IN;
    }
    public void checkRJeff()
    {
        if(rJeffState.equals(rJeffStates.UP) || rJeffState.equals(rJeffStates.IN))
        {
            rJeffDown();
        }
    }

    public void toggleJeffs()
    {
        if(rJeffState.equals(rJeffStates.IN) || rJeffState.equals(rJeffStates.UP) || lJeffState.equals(lJeffStates.UP))
        {
            rJeffDown();
            lJeffDown();
        }
        else
        {
            rJeffUp();
            lJeffUp();
        }
    }




}
