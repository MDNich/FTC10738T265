package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmAndGrabberController {

    public DcMotor armMotor;
    private Servo claw;
    public static final int EPSILON = 10000;
    public double kPp;
    private double kPv;
    private int target;
    int iniPos, finPos;
    long iniTime, finTime;
    public double vel;

    boolean isClawOpen;




    public ArmAndGrabberController(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, "transfer");
        //arm = hardwareMap.get(Encoder.class, "transfer");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setTargetPosition(0);
        claw = hardwareMap.get(Servo.class, "Grabber");
        claw.setDirection(Servo.Direction.REVERSE);
        closeClaw();
        isClawOpen= false;
        kPp = 0.15;
        kPv = 0.07;
        target  = 0;
        iniPos = 0;
        finPos = 0;
        iniTime = 0;
        finTime = 0;
        vel = 0;

    }


    public void goToPosArm(int position, boolean qOpenClaw) {

        //400 ticks per 3 sec.

        // for now it is only a P
        target = position;
        double iniPos = armMotor.getCurrentPosition();
        iniTime = System.nanoTime();
        if (armMotor.getCurrentPosition() < position) {
            while (armMotor.getCurrentPosition() + 15 < position) {
                calculateV();
                veloController((iniPos > position) ? -40 : 40);
                ///iniPos = armMotor.getCurrentPosition();

            }
        } else if (armMotor.getCurrentPosition() > position) {
            while (armMotor.getCurrentPosition() + 15 > position) {
                calculateV();
                veloController((iniPos > position) ? 400 : -400);
                ////iniPos = armMotor.getCurrentPosition();

            }
        }

        if (qOpenClaw) {
            armMotor.setPower(0.1);
            try {
                Thread.sleep(1000);
            } catch (Exception e) {

            }
            openClaw();
        }
        armMotor.setPower(0);

    }
    public void goToPosArmBack(int position) {

        //400 ticks per 3 sec.

        // for now it is only a P
        target = position;
        double iniPos = armMotor.getCurrentPosition();
        iniTime = System.nanoTime();
        if (armMotor.getCurrentPosition() > position) {
            while (armMotor.getCurrentPosition() + 15 > position) {
                calculateV();
                veloController(-40);
                ///iniPos = armMotor.getCurrentPosition();

            }
            armMotor.setPower(0);
        } else if (armMotor.getCurrentPosition() < position) {
            while (armMotor.getCurrentPosition() + 15 < position) {
                calculateV();
                veloController(400);
                ////iniPos = armMotor.getCurrentPosition();

            }
        }



        armMotor.setPower(0);

    }
    public void posController(int target) {

        // for now it is only a P

        double delta = target - armMotor.getCurrentPosition();
        if (delta > 20) {
            armMotor.setPower(0.4);
        } else if (delta > 0) {
            armMotor.setPower((kPp) * delta);
        } else if (delta > -20) {
            armMotor.setPower((kPp) * delta);
        }
        else {
            armMotor.setPower(-0.4);
        }
    }
    public void posController(int target, double numSecs) {

        // for now it is only a P
        long iniTime = System.nanoTime();
        while(System.nanoTime() < iniTime + numSecs*Math.pow(10,9)) {
            double delta = target - armMotor.getCurrentPosition();

            if (delta > 20) {
                armMotor.setPower(1);
            } else if (delta > 0) {
                armMotor.setPower((kPp) * delta);
            } else if (delta > -20) {
                armMotor.setPower((kPp) * delta);
            }
            else {
                armMotor.setPower(-1);
            }
        }
        armMotor.setPower(0);
    }
    public void posControllerNoWobble(int target, double numSecs) {

        // for now it is only a P
        long iniTime = System.nanoTime();
        while(System.nanoTime() < iniTime + numSecs*Math.pow(10,9)) {
            double delta = target - armMotor.getCurrentPosition();

            if (delta > 20) {
                armMotor.setPower(0.2);
            } else if (delta > 0) {
                armMotor.setPower((kPp) * delta);
            } else if (delta > -20) {
                armMotor.setPower((kPp) * delta);
            }
            else {
                armMotor.setPower(-0.2);
            }
        }
        armMotor.setPower(0);
    }

    public void calculateV()
    {
        finTime = System.nanoTime();
        long deltaT = finTime - iniTime;
        if(deltaT <= EPSILON) {
            return;
        }
        finPos = armMotor.getCurrentPosition();
        vel = (double) (finPos - iniPos)/(finTime - iniTime)*10E8;
        iniTime = finTime;
        iniPos = finPos;

    }


    public void veloController(int targetVel) {

        // for now it is only a P


        double delta = Math.abs(targetVel - vel);
        if (delta > 20) {
            armMotor.setPower(armMotor.getPower() + 0.2);
        } else if (delta > 0) {
            armMotor.setPower((kPv) * delta + armMotor.getPower());
        } else if (delta > -20) {
            armMotor.setPower(-(kPv) * delta + armMotor.getPower());
        } else {
            armMotor.setPower(armMotor.getPower() - 0.4);
        }
    }

    public void openClaw()
    {
        claw.setPosition(0.5);
        isClawOpen = true;
    }
    public void closeClaw()
    {
        claw.setPosition(0);
        isClawOpen = false;
    }

    public void toggleClaw()
    {
        if (isClawOpen) {
            closeClaw();
        } else {
            openClaw();
        }
    }

}