package org.firstinspires.ftc.teamcode.util.control;

import java.lang.Math;

public class Control {
    static public double trapMotionP(double maxV, double maxA, double startD, double endD, double currentT) {
        if (startD == endD) {
            return endD;
        }
        int dir = 1;
        if (endD < startD) {
            dir = -1;
        }

        double t_accel = maxV/maxA;
        double d_half = Math.abs(endD - startD) / 2;
        double d_accel = (1/2.0)*maxA*Math.pow(t_accel, 2);

        if (d_accel > d_half) {
            t_accel = Math.sqrt(d_half / (0.5*maxA));
            d_accel = (1/2.0)*maxA*Math.pow(t_accel, 2);
        }


        maxV = maxA*t_accel;

        double d_cruise = Math.abs(endD - startD) - (2*d_accel);
        double t_cruise = d_cruise / maxV;

        double t_entire = t_accel + t_cruise + t_accel;
        if (currentT > t_entire) {
            return endD;
        } else if (currentT < t_accel) {
            return startD + dir*(1/2.0)*maxA*Math.pow(currentT, 2);
        } else if (currentT < t_accel + t_cruise) {
            double t_current_cruise = currentT - t_accel;
            return startD + dir*(d_accel + maxV*t_current_cruise);
        } else {
            double t_current_decel = currentT - (t_accel + t_cruise);
            return startD + dir*(d_accel + d_cruise + maxV*t_current_decel - (1/2.0)*maxA*Math.pow(t_current_decel, 2));
        }
    }
    static public double trapMotionV(double maxV, double maxA, double startD, double endD, double currentT) {
        if (startD == endD) {
            return 0.0;
        }
        int dir = 1;
        if (endD < startD) {
            dir = -1;
        }

        double t_accel = maxV/maxA;
        double d_half = Math.abs(endD - startD) / 2;
        double d_accel = (1/2.0)*maxA*Math.pow(t_accel, 2);

        if (d_accel > d_half) {
            t_accel = Math.sqrt(d_half / (0.5*maxA));
            d_accel = (1/2.0)*maxA*Math.pow(t_accel, 2);
        }

        maxV = maxA*t_accel;

        double d_cruise = Math.abs(endD - startD) - (2*d_accel);
        double t_cruise = d_cruise / maxV;

        double t_entire = t_accel + t_cruise + t_accel;
        if (currentT > t_entire) {
            return 0.0;
        } else if (currentT < t_accel) {
            return dir*maxA*currentT;
        } else if (currentT < t_accel + t_cruise) {
            return dir*maxV;
        } else {
            double t_current_decel = currentT - (t_accel + t_cruise);
            return dir*(maxV-maxA*t_current_decel);
        }
    }
    static public double trapMotionA(double maxV, double maxA, double startD, double endD, double currentT) {
        if (startD == endD) {
            return 0.0;
        }
        int dir = 1;
        if (endD < startD) {
            dir = -1;
        }

        double t_accel = maxV/maxA;
        double d_half = Math.abs(endD - startD) / 2;
        double d_accel = (1/2.0)*maxA*Math.pow(t_accel, 2);

        if (d_accel > d_half) {
            t_accel = Math.sqrt(d_half / (0.5*maxA));
            d_accel = (1/2.0)*maxA*Math.pow(t_accel, 2);
        }


        maxV = maxA*t_accel;

        double d_cruise = Math.abs(endD - startD) - (2*d_accel);
        double t_cruise = d_cruise / maxV;

        double t_entire = t_accel + t_cruise + t_accel;
        if (currentT > t_entire) {
            return 0.0;
        } else if (currentT < t_accel) {
            return dir*maxA;
        } else if (currentT < t_accel + t_cruise) {
            return 0.0;
        } else {
            double t_current_decel = currentT - (t_accel + t_cruise);
            return dir*(-maxA);
        }
    }
}
