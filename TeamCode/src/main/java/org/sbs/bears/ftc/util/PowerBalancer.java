package org.sbs.bears.ftc.util;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;

import static java.lang.Thread.sleep;

@RequiresApi(api = Build.VERSION_CODES.N)
public class PowerBalancer {

    // Status Initialized
    static boolean initializedBalancer = false;

    // Hardware Map
    static HardwareMap hardwareMap;

    // Voltage Battery Meter
    static VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

    // Booleans
    static boolean startingValues = true;
    static boolean recyclingValues = false;

    // Value Being Returned
    public static double proficentPower;


    // Averaged Current Battery Voltage Reference ArrayList
    static ArrayList<Double> averagedVoltageValues = new ArrayList<Double>();
    // Corrected Current Battery Voltage Reference ArrayList
    static ArrayList<Double> correctedAverageVoltageValues = new ArrayList<Double>();


    // Main Balancer
    static Thread balancerMain = new Thread(() -> {
        // Check for Balance Mode | Recycle or Sample
        averagedValuesModeCheck();

        // Main
        if(startingValues == true) {
            // Get Samples
            // Try Catch Sleep
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (recyclingValues == true) {
            recycleLists(voltageSensor.getVoltage());
            getCorrectedValue();
        }
    });

    // Allows for other classes to retrieve power
    public static double getPower() throws VoltageBalancerNotInitializedException {
        if(initializedBalancer != false) {
            if(startingValues == true) {
                return voltageSensor.getVoltage();
            } else {
                return proficentPower;
            }
        } else {
            throw new VoltageBalancerNotInitializedException("Voltage Balancer Was Not Initialized", null, false, true);
        }
    }

    // Starts the balancer and returns the status of the request
    public static boolean startBalancer() {
        boolean status = false;
        if(!initializedBalancer) {
            try {
                balancerMain.start();
                status = true;
            } catch(Exception e) {
                status = false;
            }
        } else {
            status = false;
        }
        return status;
    }

    // Stops the balancer and returns the status of the request
    public static boolean stopBalancer() {
        boolean status = false;
        if(initializedBalancer) {
            try {
                balancerMain.stop();
                purgeVoltageTables();
                status = true;
            } catch (Exception e) {
                status = false;
            }
        } else {
            status = false;
        }
        return status;
    }

    // Purges all voltage tables
    public static void purgeVoltageTables() {
        correctedAverageVoltageValues.clear();
        averagedVoltageValues.clear();
    }

    // Checks if we have enough values in our averageVoltageValues
    static void averagedValuesModeCheck() {
        if(averagedVoltageValues.size() != 10) {
            startingValues = true;
        } else if (averagedVoltageValues.size() == 10) {
            startingValues = false;
            recyclingValues = true;
        }
    }

    // Gets the average of the correctedAverageVoltageValues
    static double getCorrectedValue() {
        double correctedAverageVoltage = 0;
        for(double correctedVolage : correctedAverageVoltageValues) {
            correctedAverageVoltage = correctedAverageVoltage + correctedVolage;
        }
        correctedAverageVoltage = Math.abs(correctedAverageVoltage / correctedAverageVoltageValues.size());
        return correctedAverageVoltage;
    }

    // Gets the average of the averageVoltageValues
    static double getRawAverageValue() {
        double averageVoltage = 0;
        for(double voltage : averagedVoltageValues) {
            averageVoltage = averageVoltage + voltage;
        }
        averageVoltage = Math.abs(averageVoltage / averagedVoltageValues.size());
        return averageVoltage;
    }


    // Moves the values of the ArrayLists to the right, and then updates the values with new ones
    static void recycleLists(double tempPower) {
        if(averagedVoltageValues.size() == 10) {
            if(averagedVoltageValues.size() == 10) {
                double temp = averagedVoltageValues.get(averagedVoltageValues.size()-1);
                for(int i = averagedVoltageValues.size()-1; i > 0; i--) {
                    averagedVoltageValues.set(i, averagedVoltageValues.get(i-1));
                }
                averagedVoltageValues.set(0, tempPower);
            }
            if(correctedAverageVoltageValues.size() == 10) {
                double temp = correctedAverageVoltageValues.get(correctedAverageVoltageValues.size()-1);
                for(int i = correctedAverageVoltageValues.size()-1; i > 0; i--) {
                    correctedAverageVoltageValues.set(i, correctedAverageVoltageValues.get(i-1));
                }
                correctedAverageVoltageValues.set(0, getRawAverageValue());
            }
        }
    }


}

// Exceptions that can be thrown
class VoltageBalancerNotInitializedException extends Exception {
        @RequiresApi(api = Build.VERSION_CODES.N)
        public VoltageBalancerNotInitializedException(String message, Throwable cause, boolean EnableSupression, boolean writeableStackTrace) {
            super(message, cause, EnableSupression, writeableStackTrace);
        }
}
