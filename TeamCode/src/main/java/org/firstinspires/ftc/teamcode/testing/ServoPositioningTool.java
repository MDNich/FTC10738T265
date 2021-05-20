package org.firstinspires.ftc.teamcode.testing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class ServoPositioningTool extends LinearOpMode {

    private Servo lJeff;
    private Servo rJeff;
    boolean pressingUp = false;
    boolean pressingDown = false;
    boolean pressingY = false;
    boolean leftSelected = true;
    private Servo editingServo;
    double settingPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        lJeff = hardwareMap.servo.get("lJeff");
        rJeff = hardwareMap.servo.get("rJeff");
        editingServo = lJeff;
        settingPosition = 0;
        waitForStart();
        while (!isStopRequested()) {
            if(gamepad1.dpad_up && !pressingUp) {
                pressingUp = true;
            } else if(!gamepad1.dpad_up && pressingUp) {
                settingPosition = settingPosition + 0.01;
                pressingUp = false;
            }

            if(gamepad1.y && !pressingY) {
                pressingY = true;
            } else if(!gamepad1.y && pressingY) {
                if(leftSelected) {
                    leftSelected = !leftSelected;
                    editingServo = rJeff;
                } else if(!leftSelected) {
                    leftSelected = !leftSelected;
                    editingServo = lJeff;
                }
                pressingY = false;
            }


            // DPAD DOWN
            if(gamepad1.dpad_down && !pressingDown) {
                pressingDown = true;
            } else if(!gamepad1.dpad_down && pressingDown) {
                settingPosition = settingPosition - 0.01;
                pressingDown = false;
            }

            editingServo.setPosition(settingPosition);
            telemetry.addData("lJeff", lJeff.getPosition());
            telemetry.addData("rJeff", rJeff.getPosition());
            telemetry.update();
        }
    }
}