package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class TurnTesting extends OpMode {
    SampleMecanumDrive drive;
    private boolean qRB;
    private boolean qLB;
    private boolean qRD;
    private boolean qLD;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry = new MultipleTelemetry(telemetry);
    }

    @Override
    public void loop() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );
        if(gamepad1.right_bumper && !qRB)
        {
            drive.turn(Math.toRadians(20));
        }
        else if(!gamepad1.right_bumper && qRB) {
            qRB = false;
        }
        if(gamepad1.left_bumper && !qLB)
        {
            drive.turn(Math.toRadians(45));
        }
        else if(!gamepad1.left_bumper && qLB) {
            qRB = false;
        }
        if(gamepad1.dpad_right && !qRD)
        {
            drive.turn(Math.toRadians(90));
        }
        else if(!gamepad1.dpad_right && qRD) {
            qRB = false;
        }
        if(gamepad1.dpad_left && !qLD)
        {
            drive.turn(Math.toRadians(180));
        }
        else if(!gamepad1.dpad_left && qLD) {
            qRB = false;
        }
        telemetry.addData("heading",drive.getPoseEstimate().getHeading());
        telemetry.update();
    }
}
