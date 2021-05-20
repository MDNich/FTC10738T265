package org.sbs.bears.ftc.robot.controller;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.autonomous.ArmAndGrabberController.EPSILON;


@Config
/**
 * This class is written to control the shooter PID.
 * @author Marc Nichitiu
 */
public class PIDshooterController {


    VoltageSensor batteryMeter;
    private DcMotor s1 = null;
    private DcMotor s2 = null;
    private Servo liftServo;
    private Servo pusherServo;
    private Servo shooterServo;

    private DcMotor shooter1Encoder = null;
    double deriv;
    double powToMotor;
    public static double kP = 0.001;
    public static double kI = 0;
    public static double kD = 0.00005;
    public static double cap = 0;
    public static double error;
    public static double targetVel = 2200;
    public static double rampAngle = 0.395;
    private long deltaT = 0;
    public static double motorPow = 0.2;
    private long iniTime, finTime;
    private long iniPos, finPos;
    protected double vel;
    private double totalError;
    private double errorLast;
    private double numErrors;

    public static double bayAngle = 0.865;

    private FtcDashboard dashboard;

    public double avgError;
    public static double kF;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;



    /**
     * @author Marc D Nichitiu
     * This is the constructor. It takes hardwaremap and telemetry.
     * @param hwMap the hardware map.
     * @param telmetry the telemetry opmode object.
     *
     */
    public PIDshooterController(HardwareMap hwMap, Telemetry telmetry) {
        this.hardwareMap = hwMap;
        this.telemetry = telmetry;
        batteryMeter = hardwareMap.voltageSensor.iterator().next();
        shooter1Encoder  = hardwareMap.get(DcMotor.class, "s1");
        shooter1Encoder.setDirection(DcMotorSimple.Direction.REVERSE);

        s1 = hardwareMap.get(DcMotor.class, "s1");
        s2 = hardwareMap.get(DcMotor.class, "s2");
        s1.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setDirection(DcMotorSimple.Direction.REVERSE);
        s1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        s2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        numErrors = 0;

        shooterServo = hardwareMap.get(Servo.class, "angle");
        shooterServo.setDirection(Servo.Direction.REVERSE);
        pusherServo = hardwareMap.get(Servo.class, "pusherServo");
        pusherServo = hardwareMap.servo.get("pusherServo");
        liftServo = hardwareMap.get(Servo.class, "lift4");

        liftServo.setDirection(Servo.Direction.FORWARD);
        liftServo.setPosition(bayAngle);

        //motor2Controller.start();


        shooterServo.setPosition(rampAngle); //POSITION 3 .609 POSITION 2 .6643 POSITION 1 .60838
        s1.setPower(0); //power 85, pos 1 9
        s2.setPower(0); //power 85, p
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(1);
        shooterServo.setPosition(rampAngle);
    }
    /** @author Marc D Nichitiu
     * @version 1.2.0
     * This method prints telemetry to the phone and dashboard. This will squash the current telemetry.
     */
    public void printTelemetry() {

        telemetry.addData("Shooter Power", s1.getPower());
        telemetry.addData("vel", vel);
        telemetry.addData("velLowLevel", ((DcMotorEx) (s1)).getVelocity());
        telemetry.addData("avg+velLowLevel", ((DcMotorEx) (s1)).getVelocity() + error);
        telemetry.addData("targetVel", targetVel);
        telemetry.addData("currentError", error);
        telemetry.addData("avgError", avgError);
        telemetry.addData("vel+avgError", targetVel + avgError);
        telemetry.addData("Voltage", batteryMeter.getVoltage()); // added this morning
        telemetry.addData("shooter angle", shooterServo.getPosition()); // added this morning
        telemetry.addData("shooter pos", shooter1Encoder.getCurrentPosition()); // added this morning
        telemetry.update();
    }

    /**
     * @author Marc D Nichitiu
     * @version 1.2.0
     * This method calculates the velocity of the shooter 1.
     */
    public void calculateV() {
        finTime = System.nanoTime();
        finPos = shooter1Encoder.getCurrentPosition();
        deltaT = finTime - iniTime;
        if (deltaT <= EPSILON) {
            return;
        }
        vel = (double) (finPos - iniPos) / deltaT * 1E9;

        iniTime = finTime;
        iniPos = finPos;
    }


    /**
     * @author Marc D Nichitiu
     * This method should be called in the loop.
     * Preconditions: must be called in a loop.
     * @param targetVel the target vel of the shot
     */
    public void doPIDLoop(int targetVel)
    {
        calculateV();
        vController(targetVel);
    }

    /**
     * This method does a PID velocity controller for the shooter wheel.
     * @param target the target velocity of the shooter in clicks/sec.
     * @author Marc Nichitiu
     * @version 1.0.0
     */
    public void vController(double target) {
        //vel = s1.getVelocity();
        error = target - vel;
        totalError += error;
        deriv = (double) (error - errorLast) / deltaT * 1E9;
        powToMotor = Math.max((s1.getPower() + (kP * error) + kI * totalError + (kD * deriv)), cap);
        setPowShooter(powToMotor);
        errorLast = error;
        numErrors++;
        if (numErrors >= 50) {
            avgError = totalError / 50.0;
            numErrors = 0;
            totalError = 0;
        }
    }

    /**
     * This method sets the specified power to the shooter motors.
     * @param pow the power to which the motors will be set.
     * Preconditions: pow must be from -1 to 1 inclusive.
     * @author Marc Nichitiu
     * @version 1.0.0
     */
    public void setPowShooter(double pow) {
        s1.setPower(pow);
        s2.setPower(pow);
    }

}