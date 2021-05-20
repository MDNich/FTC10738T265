package org.sbs.bears.ftc.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.ftc.robot.controller.ArmAndGrabberManager;
import org.sbs.bears.ftc.robot.controller.BlockerController;
import org.sbs.bears.ftc.robot.controller.DriveAngleController;
import org.sbs.bears.ftc.robot.controller.LaunchController;
import org.sbs.bears.ftc.robot.controller.PIDshooterController;
import org.sbs.bears.ftc.robot.controller.RRDriveController;
import org.sbs.bears.ftc.robot.controller.RRDriveControllerNoOdom;
import org.sbs.bears.ftc.robot.controller.RingSubsytemController;
import org.sbs.bears.ftc.robot.controller.ThreadController;
import org.sbs.bears.ftc.util.Beta;

/**
 * This class models the Robot and is in a has-a relationship with the robot controllers.
 * <p>
 * Controllers in the Robot:
 * <ul>
    * <li>Arm and Grabber Controller</li>
    * <li>Drive Angle Controller <strong>(Beta)</strong></li>
    * <li>Ring Subsytem Controller</li>
    * <li>RoadRunner Drive Controller (Drive Encoders)</li>
    * <li>Thread Controller <strong>(Beta)</strong></li>
    * <li>Blocker Controller</li>
 * </ul></p>
 * @author Marc D Nichitiu
 * @version 2.0
 */
public class Robot {

    /**
     * This is the controller for the Arm and Grabber. See <a href="controller/ArmAndGrabberManager.html">here</a> for more information.
     */
    public ArmAndGrabberManager armCtrl;

    /**
     * This is the controller for finely changing the angle of the robot.
     * Note that this is in beta.
     * @deprecated this is in beta Do Not Use
     */
    @Beta
    public DriveAngleController driveAngleCtrl;

    /**
     * This is the controller for the entire Ring Subsystem.
     * It contains a Launch Controller and Intake Controller.
     */
    public RingSubsytemController ringCtrl;

    /**
     * This is the RoadRunner wrapper. It allows client classes
     * to very simply use the main functions of RoadRunner without meddling with the code.
     */
    public RRDriveControllerNoOdom rrCtrlNoOdom;



    /**
     * This is the Thread Controller.
     * It is in Beta.
     * @deprecated This is in Beta. Do Not Use.
     */
    @Beta
    public ThreadController threadCtrl;


    /**
     * This is the Blocker Controller.
     * It controls the blockers in front of the robot.
     */
    public BlockerController blockerCtrl;


    /**
     * This is the constructor for the Robot object.
     * One must provide it with the hardwareMap of the robot and the telemetry object, which will notify the client
     * the opmode state.
     * @param hardwareMap the hardwareMap, which is necessary for initialization of hardware devices.
     * @param telemetry the telemetry object, which is necessary for notification of termination of initialization as well as other operations.
     */
    public Robot(HardwareMap hardwareMap, Telemetry telemetry)
    {
        armCtrl = new ArmAndGrabberManager(hardwareMap);
        driveAngleCtrl = new DriveAngleController(hardwareMap,telemetry);
        ringCtrl = new RingSubsytemController(hardwareMap,telemetry);
        rrCtrlNoOdom = new RRDriveControllerNoOdom(hardwareMap,telemetry);
        threadCtrl = new ThreadController(hardwareMap);
        blockerCtrl = new BlockerController(hardwareMap,telemetry);
    }

}
