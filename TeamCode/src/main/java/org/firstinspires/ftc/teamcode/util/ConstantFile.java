package org.firstinspires.ftc.teamcode.util;

import org.sbs.bears.ftc.constants.RingLaunchSetting;

import java.util.HashMap;

public interface ConstantFile {
    public HashMap<RingLaunchSetting, Object> getAllConstants();
    public void setAllConstants(HashMap<? extends Enum, Object> newData);
}
