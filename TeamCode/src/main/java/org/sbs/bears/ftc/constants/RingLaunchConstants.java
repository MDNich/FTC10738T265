package org.sbs.bears.ftc.constants;

import org.firstinspires.ftc.teamcode.util.ConstantFile;

import java.util.HashMap;

public class RingLaunchConstants implements ConstantFile {

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
        cts.put(RingLaunchSetting.TELEOP_GOALSHOT,Double.valueOf(0.455));
        cts.put(RingLaunchSetting.TELEOP_PSHOT,Double.valueOf(0.5));

    }

    @Override
    public HashMap<RingLaunchSetting, Object> getAllConstants() {
        return cts;
    }

    @Override
    public void setAllConstants(HashMap<? extends Enum, Object> newData) {
        cts = (HashMap<RingLaunchSetting, Object>) newData;
    }

    public double getValue(RingLaunchSetting key)
    {
        return (double) cts.get(key);
    }


}

