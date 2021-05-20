package org.sbs.bears.ftc.robot.controller;

import com.qualcomm.robotcore.hardware.HardwareMap;


import org.sbs.bears.ftc.robot.RobotLogger;
import org.sbs.bears.ftc.util.RobotSubsytemManager;

import java.util.ArrayList;

public class ThreadController extends RobotSubsytemManager {

    private static HardwareMap hwMap;
    private static int ThreadCount = 0;

    private ArrayList<Thread> pendingThreads = new ArrayList<Thread>();
    private ArrayList<Thread> AsyncThreads = new ArrayList<Thread>();
    private ArrayList<Thread> currentThreads = new ArrayList<Thread>();


    public ThreadController(HardwareMap hwMap) {
        super(hwMap,null);
    }

    private static void createTask(Thread task) {
        RobotLogger.logToDebugger("Thread Manager", "Recieved Task");
        if(task != null) {


        }
    }


    @Override
    public void shutDown() {

    }
}
