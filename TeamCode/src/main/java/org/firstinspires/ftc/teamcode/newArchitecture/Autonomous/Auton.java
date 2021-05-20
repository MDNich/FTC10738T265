package org.firstinspires.ftc.teamcode.newArchitecture.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Auton Beta")
public class Auton extends OpMode {
    AutonBrain brain;
    @Override
    public void init() {
       brain = new AutonBrain(hardwareMap,telemetry);
    }

    @Override
    public void loop() {
        brain.doBrainTask();
        if(AutonBrain.brainState == AutonBrain.CURRENT_COMMAND.DONE)
            requestOpModeStop();
    }
}
