package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubEx;

public class AutonomousReferences
{
    // Servos
    public static Servo shooterAngleServo;
    public static Servo liftServo;
    public static Servo pusherServo;

    // Servo Positions
    public static double liftServoDownPos = 1.0;
    public static double liftServoUpPos = 0;

    public static double pusherServoInPos = 0.05;
    public static double pusherServoOutPos = 0.24;

    public static double beginningShooterAngle = 0.59;

    // Motors
    public static DcMotor s1;
    public static DcMotor s2;
    public static DcMotor intakeMotor;

    public static void initializeRobot(@NotNull HardwareMap hardwareMap) {
        shooterAngleServo = hardwareMap.get(Servo.class, "angle");
        pusherServo = hardwareMap.servo.get("pusherServo");
        liftServo = hardwareMap.get(Servo.class, "lift4");
        s1 = hardwareMap.get(DcMotor.class, "s1");
        s2 = hardwareMap.get(DcMotor.class, "s2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        // Sets Initial Directions For Servos
        shooterAngleServo.setDirection(Servo.Direction.REVERSE);
        liftServo.setDirection(Servo.Direction.FORWARD);

        // Sets Initial Positions For Servos
        shooterAngleServo.setPosition(beginningShooterAngle);
        liftServo.setPosition(liftServoDownPos);
        pusherServo.setPosition(pusherServoInPos);
    }
}
