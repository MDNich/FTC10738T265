package org.firstinspires.ftc.teamcode.testing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class BlockerTesting extends LinearOpMode {
    private Servo lJeff;
    private Servo rJeff;
    boolean pressingUp = false;
    boolean pressingDown = false;
    @Override
    public void runOpMode() throws InterruptedException {

        lJeff = hardwareMap.servo.get("lJeff");
        rJeff = hardwareMap.servo.get("rJeff");
        waitForStart();
        while (!isStopRequested()) {
            if(gamepad2.dpad_up && !pressingUp) {
                pressingUp = true;
            } else if(!gamepad2.dpad_up && pressingUp) {
                lJeff.setPosition(0.5);
                rJeff.setPosition(0.139);
                pressingUp = false;
            }
            // DPAD DOWN
            if(gamepad2.dpad_down && !pressingDown) {
                pressingDown = true;
            } else if(!gamepad2.dpad_down && pressingDown) {
                lJeff.setPosition(.15);
                rJeff.setPosition(.5);
                pressingDown = false;
            }
            telemetry.addData("lJeff", lJeff.getPosition());
            telemetry.addData("rJeff", rJeff.getPosition());
            telemetry.update();
        }
    }
}