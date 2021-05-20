package org.sbs.bears.ftc.util;

import java.util.HashMap;
import java.util.Map;

public interface ConstantFile {
    public HashMap<? extends Enum, Object>  getAllConstants();
    public void setAllConstants(HashMap<? extends Enum, Object> newData);
}
