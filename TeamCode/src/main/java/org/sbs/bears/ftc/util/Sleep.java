package org.sbs.bears.ftc.util;

public class Sleep {
    public static void sleep(long secs)
    {
        try {
            Thread.sleep((long) (secs*Math.pow(10,3)));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}
