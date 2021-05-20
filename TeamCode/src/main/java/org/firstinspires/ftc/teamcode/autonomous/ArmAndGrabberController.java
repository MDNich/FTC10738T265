package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.sbs.bears.ftc.util.RobotSubsytemManager;

/**
 * This class controls the Arm and Grabber on our robot.
 * @author Marc D Nichitiu
 */
public class ArmAndGrabberController extends RobotSubsytemManager {


    /** This is the armMotor object. */
    public DcMotor armMotor;

    /** This is the claw servo object. */
    private Servo claw;

    /** Epsilon represents the amount of time that is necessary for calculating velocity. */
    public static final int EPSILON = 10000;

    /** This is the proportional constant for the arm position controller. */
    public final double kPp;

    /** This is the proportional constant for the arm velocity controller. */
    public final double kPv;
    private int target;
    int iniPos, finPos;
    long iniTime, finTime;

    /** This is the velocity of the arm, updated by the function <a href="#calculateV--">calculateV().</a> */
    protected double vel;


    /** This is the boolean that states whether the claw is open or not. */
    protected boolean isClawOpen;



    /**
     * This constructor initializes the arm and grabber by starting the position of the arm and closing the claw.
     * @param hardwareMap the hardware map which supplies the information to initialize the arm motor and servo objects.
     */
    public ArmAndGrabberController(HardwareMap hardwareMap) {
        super(hardwareMap,null);
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

    /**
     * This function uses a velocity P controller to move the arm to a position that is in front of it.
     * @param position the target position of the arm.
     * @param qOpenClaw a boolean that determines whether to open the claw at the end or not.
     * Precondition: position must be larger than the arm's current position.
     */
    public void goToPosArm(int position, boolean qOpenClaw) {

        //400 ticks per 3 sec.
        if(armMotor.getCurrentPosition() >= position)
            return; // exits.
        // for now it is only a P
        target = position;
        iniTime = System.nanoTime();
        if (armMotor.getCurrentPosition() < position) {
            while (armMotor.getCurrentPosition() + 15 < position) {
                calculateV();
                veloController(40);
                ///iniPos = armMotor.getCurrentPosition();

            }
        } else if (armMotor.getCurrentPosition() > position) {
            while (armMotor.getCurrentPosition() + 15 > position) {
                calculateV();
                veloController(-300);
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


    /**
     * This method will set the armMotor to have a power that will compensate for a difference in position.
     * If called in a loop, it will effectively act as a position P controller.
     * <strong>Precondition: </strong>the target param can be any int between 0 and 450,
     * though for best performance only use positions in the range of 400 to 200: in front of the robot up to ~45deg from the vertical.
     * @param target the target position to hold the arm at.
     */
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


    /**
     * This method returns the velocity.
     * @return the velocity of the arm.
     */
    public double getVel()
    {
        calculateV();
        return vel;
    }

    /**
     * This is another position controller, but it is encapsulated inside a loop to hold for an amount of time.
     * <strong>Preconditions:</strong> the target param can be any int between 0 and 450,
     * though for best performance only use positions in the range of 400 to 200:
     * in front of the robot up to ~45deg from the vertical.
     * @param target the target position.
     * @param numSecs the amount of seconds to hold the position at.
     */
    public void posController(int target, double numSecs) {

        // for now it is only a P
        long iniTime = System.nanoTime();
        while(System.nanoTime() < iniTime + numSecs*Math.pow(10,9)) {
            posController(target);
        }
        armMotor.setPower(0);
    }


    /**
     * This method calculates velocity of the arm over a period of EPSILON.
     * It stores the velocity in the <strong>vel</strong> variable.
     */
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

    /**
     * This method holds the arm at a given position, however it is tuned for use without a wobble.
     * @param target the target position to hold the arm at.
     * @param numSecs the number of seconds to hold the arm at.
     */
    public void posControllerNoWobble(int target, double numSecs) {

        // for now it is only a P
        long iniTime = System.nanoTime();
        while(System.nanoTime() < iniTime + numSecs*Math.pow(10,9)) {
            posControllerNoWobble(target);
        }
        armMotor.setPower(0);
    }
    /**
     * This method holds the arm at a given position, however it is tuned for use without a wobble.
     * @param target the target position to hold the arm at.
     */
    public void posControllerNoWobble(int target) {

        // for now it is only a P
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


    /**
     * This method moves the arm to the specified position.
     * @deprecated This method should not be used.
     * @param position the position to which the arm will move back to.
     */
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

    /**
     * Similar to the posController method with only one parameter,
     * this method sets the motor power to a power that will compensate for a difference
     * in measured and target velocity using a P controller.
     * @param targetVel the target velocity of the velocity controller.
     */
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
            armMotor.setPower(armMotor.getPower() - 0.2);
        }
    }

    /** This method opens the claw. It does not matter where the claw is. */
    public void openClaw()
    {
        claw.setPosition(0.5);
        isClawOpen = true;
    }

    /** This method closes the claw. It does not matter where the claw is. */
    public void closeClaw()
    {
        claw.setPosition(0);
        isClawOpen = false;
    }

    /** This method toggles the claw's state: it opens the claw if it is closed, and vice versa. */
    public void toggleClaw()
    {
        if (isClawOpen) {
            closeClaw();
        } else {
            openClaw();
        }
    }

    @Override
    public void shutDown() {

    }
}