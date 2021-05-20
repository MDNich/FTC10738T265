package org.sbs.bears.ftc.util;

import com.qualcomm.robotcore.hardware.VoltageSensor;

public class VoltageCalculator {
    VoltageSensor batteryMeter;
    public static double k = -0.0715;
    public static double b = 1.61;

    public VoltageCalculator(VoltageSensor batteryMeter){
        this.batteryMeter = batteryMeter;
    }
    public double calculatePowerShooter()
    {
        return k*batteryMeter.getVoltage()+b;
    }
    public void setK(double k)
    {
        this.k = k;

    }
    public void setB(double b)
    {
        this.b = b;
    }
    public double getK() {return this.k;}
    public double getB() {return this.b;}

}


/*


P = k*V + b










*/