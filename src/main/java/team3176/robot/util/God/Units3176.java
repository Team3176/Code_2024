package team3176.robot.util.God;

import team3176.robot.*;

public class Units3176 {

  public double rads2Tics(double rads, double EncoderUnitsPerRevolution) {
    // rads = rads * (2 * Math.PI);
    double rads_clamped = (Math.min((Math.max(rads, (-Math.PI))), (Math.PI)));
    double tics = ((rads_clamped / (2.0 * Math.PI)) * EncoderUnitsPerRevolution);
    return tics;
  }

  public int rads2Tics(double rads, double EncoderUnitsPerRevolution, boolean wantItInInts) {
    // rads = rads * (2 * Math.PI);
    double rads_clamped = (Math.min((Math.max(rads, (-Math.PI))), (Math.PI)));
    double tics = ((rads_clamped / (2.0 * Math.PI)) * EncoderUnitsPerRevolution);
    return (int) tics;
  }

  public double tics2Rads(double tics, double EncoderUnitsPerRevolution) {
    tics = tics % EncoderUnitsPerRevolution;
    if (tics < 0) {
      tics += EncoderUnitsPerRevolution;
    }
    tics -= (EncoderUnitsPerRevolution / 2);
    return ((tics / EncoderUnitsPerRevolution) * (2 * Math.PI));
  }

  public static double feetPerSecond2metersPerSecond(double i) {
    return i / 3.2808;
  }

  public static double metersPerSecond2feetPerSecond(double i) {
    return i * 3.2808;
  }

  public static double revolutionsPerMinute2ticsPer100MS(double maxRPM, double ticsPerRev) {
    return (maxRPM * ticsPerRev) / 600;
  }
}
