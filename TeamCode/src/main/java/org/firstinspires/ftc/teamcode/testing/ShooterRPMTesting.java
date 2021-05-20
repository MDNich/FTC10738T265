package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ShooterRPMTesting extends LinearOpMode {
    private DcMotor shooterEncoder;
    private DcMotor shooter;
    boolean pressingUp;
    boolean pressingDown;

    double rate;

    int t1 = 0;
    int t2 = 0;
    double time1 = -1;
    double time2;

    void getRate() {
        time2 = getRuntime();
        t2 = shooterEncoder.getCurrentPosition();

        rate = (t2 - t1) / (time2 - time1);


        time1 = time2;
        t1 = t2;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        shooterEncoder = hardwareMap.get(DcMotor.class, "transfer"); //-28 is full revolution
        shooter = hardwareMap.get(DcMotor.class, "s1");
        shooterEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();



        while (opModeIsActive()){

            getRate();



            if(gamepad1.dpad_up && !pressingUp) {
                pressingUp = true;
                shooter.setPower(shooter.getPower()+.1);
            } else if(!gamepad1.dpad_up && pressingUp) {
                pressingUp = false;
            }

            // DPAD DOWN
            if(gamepad1.dpad_down && !pressingDown) {
                pressingDown = true;
                shooter.setPower(shooter.getPower()-.1);
            } else if(!gamepad1.dpad_down && pressingDown) {
                pressingDown = false;
            }

            telemetry.addData("Rate", rate);
            telemetry.update();

        }

    }
}
