package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubEx;

public class RobotConstants {

    // Control Hub and Expansion Hub
    public static ExpansionHubEx expansionHub;
    public static ExpansionHubEx controlHub;

    // Servos
    public static Servo shooterAngleServo;
    public static Servo liftServo;
    public static Servo pusherServo;

    // Servo Positions
    public static double liftServoStartPosition = 1.0;
    public static double pusherServoStartPosition = 0.35;

    // Motors
    public static DcMotor s1;
    public static DcMotor s2;
    public static DcMotor intakeMotor;

    public static void initializeRobotConstants(@NotNull HardwareMap hardwareMap) {
        // Gives The Objects Values
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        controlHub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
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
        liftServo.setPosition(liftServoStartPosition);
        pusherServo.setPosition(pusherServoStartPosition);



    }

}
