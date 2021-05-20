package org.firstinspires.ftc.teamcode.newArchitecture.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Positions {

    public static Pose2d iniPos = new Pose2d(-61, -27, 0);
    public static Pose2d teleOpShootPos = new Pose2d(0,-36,0);

    public static Pose2d startingPosition = new Pose2d(-61, -27, 0);
    public static Pose2d startShootingPosition = new Pose2d(-45, -31, 0);
    public static Pose2d readRingsPosition = new Pose2d(-56, -32.5, 6.26);
    public static Pose2d secondWobblePosition = new Pose2d(-35.8, -40, 3.08);
    public static Pose2d endPosition = new Pose2d(13, -34.7, 6.2);
    public static Pose2d powerShotOne = new Pose2d(-45, -31, 6.24);

    // Zero Rings
    public static Pose2d zeroRingsWobblePosition = new Pose2d(10.5, -50, 4.7);
    public static Pose2d zeroRingsSecondWobbleDropPosition = new Pose2d(1.5, -46, 4.7);

    // One Ring
    public static Pose2d oneRingPickupPosition = new Pose2d(-25.2,-37.7, 6.3);
    public static Pose2d oneRingShootPosition = new Pose2d(-24.8, -36.3, 0);
    public static Pose2d oneRingWobbleDropPosition = new Pose2d(26.8, -47.8, 0); // new Pose2d(26.8, -37.8, 0);
    public static Pose2d oneRingSecondWobbleDropPosition = new Pose2d(17.2, -56, 0);

    // Four Rings
    public static Pose2d fourRingPrepPosition = new Pose2d(-36.8,-34.2, 6.26);
    public static Pose2d fourRingWobbleDropPosition = new Pose2d(45.4, -59.3, 0);
    public static Pose2d fourRingSecondWobbleDropPosition = new Pose2d(38.98, -55, 5.4);
}
