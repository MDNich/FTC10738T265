package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.intakeMotor;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.liftServo;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.pusherServo;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.s1;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.s2;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.shooterAngleServo;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousV2.ringCase;
import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

//TODO: awooooga woah mama humna humna eyes pop out of skull jaw drops to floor toungue rolls out what a dame
public class AutonomousHandle {

    private static SampleMecanumDrive drive;
    private static HardwareMap hardwareMap;
    private static Telemetry telemetry;

    // Constructor
    public AutonomousHandle(SampleMecanumDrive drive, HardwareMap hardwareMap, Telemetry telemetry) {
        this.drive = drive;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public static void displayHardwareReport() {
        telemetry.addData("Shooter One Info", s1.getConnectionInfo());
        telemetry.addData("Shooter Two Info", s2.getConnectionInfo());
        telemetry.addData("Lift Info", liftServo.getConnectionInfo());
        telemetry.addData("Pusher Info", pusherServo.getConnectionInfo());
        telemetry.addData("Shooter Angle Info", shooterAngleServo.getConnectionInfo());
        telemetry.addData("Intake Motor Info", intakeMotor.getConnectionInfo());
        telemetry.addData("OpenCV Ring Report", ringCase);
        telemetry.addData("Robot Status", "Initialized");
        telemetry.update();
    }






    public static Thread shootOneRing = new Thread(() -> {
        sleep(400);
        pusherServo.setPosition(0.05);
        sleep(400);
        pusherServo.setPosition(0.24);
    });

    public static  Thread doPusher = new Thread(() -> {
        sleep(400);
        pusherServo.setPosition(0.059);
        sleep(400);
        pusherServo.setPosition(0.24);
    });



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                   Publicly Availible References                                       //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    static ArmAndGrabberController armAndGrabberController;
    public void dropArmAndOpenClaw() {
        armAndGrabberController = new ArmAndGrabberController(hardwareMap);
        armAndGrabberController.goToPosArm(130, false);
    }
    // Starts Intake
    public void startIntake() {
        intakeMotor.setPower(-1);
    }
    // Stops Intake
    public void stopIntake() {
        intakeMotor.setPower(0);
    }
    // Puts Lift Up
    public static Thread liftUp = new Thread(() -> {
        liftServo.setPosition(0.85);
    });

    // Puts Lift Down
    public static Thread liftDown = new Thread(() -> {
        liftServo.setPosition(1);
    });

    // Stops FlyWheel Independently
    public static Thread stopFlywheel = new Thread(() -> {
        // 0 Stop
        s1.setPower(0);
        s2.setPower(0);
    });

    // Starts FlyWheel Independently
    public static Thread startFlywheel = new Thread(() -> {
        // -1 Start
        s1.setPower(-1);
        s2.setPower(-1);
    });
    // Does the Three Powershots
    public static Thread doPowershots = new Thread(() -> {
        liftUp.run();
        try {
            liftUp.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        shootOneRing.run();
        drive.turn(10);
        shootOneRing.run();
        drive.turn(10);
        shootOneRing.run();
        liftDown.run();
    });

    // Return Servo and Motors Objects
    public static Servo returnPusherServo() {
        return pusherServo;
    }



}