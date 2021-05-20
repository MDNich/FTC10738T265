package org.sbs.bears.ftc.robot.controller;

import java.util.concurrent.atomic.AtomicBoolean;

public class PIDarmController implements Runnable {

    private Thread worker;
    private final AtomicBoolean running = new AtomicBoolean(false);
    private ArmAndGrabberManager armCtrl;
    private int target;

    public PIDarmController(ArmAndGrabberManager armCtrl, int target) {
        this.armCtrl = armCtrl;
        this.target = target;
    }

    public void start() {
        worker = new Thread(this);
        worker.start();
    }
    public void interrupt() {
        running.set(false);
        worker.interrupt();
    }

    public void stop() {
        running.set(false);
    }

    @Override
    public void run() {
        running.set(true);
        while (running.get()) {
            // do something here
            armCtrl.posController(target);
        }
        armCtrl.shutDown();
    }
}
