package org.sbs.bears.ftc.exceptions;

import android.os.Build;

import androidx.annotation.RequiresApi;

// Exceptions that can be thrown
public class VoltageBalancerNotInitializedException extends Exception {
        @RequiresApi(api = Build.VERSION_CODES.N)
        public VoltageBalancerNotInitializedException(String message, Throwable cause, boolean EnableSupression, boolean writeableStackTrace) {
            super(message, cause, EnableSupression, writeableStackTrace);
        }
}
