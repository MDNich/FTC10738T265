package org.sbs.bears.ftc.constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.ConstantFile;

import java.util.HashMap;

@Config
public class RingLaunchConstants implements ConstantFile {

    public static HashMap<RingLaunchSetting, Object> cts1;
    static {
        cts1 = new HashMap<>();
        cts1.put(RingLaunchSetting.LAUNCH_BAY_INACTIVE,Double.valueOf(1.0));
        cts1.put(RingLaunchSetting.LAUNCH_BAY_ACTIVE,Double.valueOf(0.86));
        cts1.put(RingLaunchSetting.PUSHER_ACTIVE,Double.valueOf(0.3));
        cts1.put(RingLaunchSetting.PUSHER_INACTIVE,Double.valueOf(0.05));
        cts1.put(RingLaunchSetting.AUTO_INITIAL_SHOT_GOAL,Double.valueOf(0.58));
        cts1.put(RingLaunchSetting.AUTO_1_AND_4_RING_INTAKE_SHOT,Double.valueOf(0.58));
        cts1.put(RingLaunchSetting.AUTO_4_SECOND_RING_SHOT,Double.valueOf(0.58));
        cts1.put(RingLaunchSetting.TELEOP_GOALSHOT,Double.valueOf(0.3));
        cts1.put(RingLaunchSetting.TELEOP_PSHOT,Double.valueOf(0.5));
    }
    private HashMap<RingLaunchSetting, Object> cts;

    public RingLaunchConstants() {
        cts = new HashMap<>();
        cts.put(RingLaunchSetting.LAUNCH_BAY_INACTIVE,Double.valueOf(1.0));
        cts.put(RingLaunchSetting.LAUNCH_BAY_ACTIVE,Double.valueOf(0.86));
        cts.put(RingLaunchSetting.PUSHER_ACTIVE,Double.valueOf(0.3));
        cts.put(RingLaunchSetting.PUSHER_INACTIVE,Double.valueOf(0.05));
        cts.put(RingLaunchSetting.AUTO_INITIAL_SHOT_GOAL,Double.valueOf(0.58));
        cts.put(RingLaunchSetting.AUTO_1_AND_4_RING_INTAKE_SHOT,Double.valueOf(0.58));
        cts.put(RingLaunchSetting.AUTO_4_SECOND_RING_SHOT,Double.valueOf(0.58));
        cts.put(RingLaunchSetting.TELEOP_GOALSHOT,Double.valueOf(0.3));
        cts.put(RingLaunchSetting.TELEOP_PSHOT,Double.valueOf(0.5));

    }

    @Override
    public HashMap<RingLaunchSetting, Object> getAllConstants() {
        return cts1;
    }

    @Override
    public void setAllConstants(HashMap<? extends Enum, Object> newData) {
        cts1 = (HashMap<RingLaunchSetting, Object>) newData;
    }

    public double getValue(RingLaunchSetting key)
    {
        return (double) cts1.get(key);
    }


}

