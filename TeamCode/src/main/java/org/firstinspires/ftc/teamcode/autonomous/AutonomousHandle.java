package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.intakeMotor;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.liftServo;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.pusherServo;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.pusherServoInPos;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.pusherServoOutPos;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.s1;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.s2;
import static org.firstinspires.ftc.teamcode.autonomous.AutonomousReferences.shooterAngleServo;
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
        AutonomousReferences.initializeRobot(hardwareMap);
    }

    public static void displayHardwareReport() {
        telemetry.addData("Shooter One Info", s1.getConnectionInfo());
        telemetry.addData("Shooter Two Info", s2.getConnectionInfo());
        telemetry.addData("Lift Info", liftServo.getConnectionInfo());
        telemetry.addData("Pusher Info", pusherServo.getConnectionInfo());
        telemetry.addData("Shooter Angle Info", shooterAngleServo.getConnectionInfo());
        telemetry.addData("Intake Motor Info", intakeMotor.getConnectionInfo());
        telemetry.addData("Robot Status", "Initialized");
        telemetry.update();
    }


    public static  Thread doPusher = new Thread(() -> {
        sleep(400);
        pusherServo.setPosition(0.05);
        sleep(400);
        pusherServo.setPosition(0.24);
    });





}