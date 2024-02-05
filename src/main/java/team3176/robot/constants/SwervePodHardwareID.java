// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package team3176.robot.constants;

public class SwervePodHardwareID {
  public int SERIAL;
  public int THRUST_CID;
  public String THRUST_CBN;
  public int CANCODER_CID;
  public String CANCODER_CBN;
  public double OFFSET;

  SwervePodHardwareID() {}

  SwervePodHardwareID(
      int thrust_cid, String thrust_cbn, int cancoder_cid, String cancoder_cbn, double offset) {
    this.THRUST_CID = thrust_cid;
    this.THRUST_CBN = thrust_cbn;
    this.CANCODER_CID = cancoder_cid;
    this.CANCODER_CBN = cancoder_cbn;
    this.OFFSET = offset;
  }

  SwervePodHardwareID(
      int serial,
      int thrust_cid,
      String thrust_cbn,
      int cancoder_cid,
      String cancoder_cbn,
      double offset) {
    this.THRUST_CID = thrust_cid;
    this.THRUST_CBN = thrust_cbn;
    this.CANCODER_CID = cancoder_cid;
    this.CANCODER_CBN = cancoder_cbn;
    this.SERIAL = serial;
    this.OFFSET = offset;
  }

  public static boolean check_duplicates(SwervePodHardwareID a, SwervePodHardwareID b) {
    boolean are_duplicates = (a.CANCODER_CID == b.CANCODER_CID) | (a.THRUST_CID == b.THRUST_CID);
    return are_duplicates;
  }

  public static boolean check_duplicates_all(
      SwervePodHardwareID a, SwervePodHardwareID b, SwervePodHardwareID c, SwervePodHardwareID d) {
    return check_duplicates(a, b)
        | check_duplicates(a, c)
        | check_duplicates(a, d)
        | check_duplicates(b, c)
        | check_duplicates(b, d)
        | check_duplicates(c, d);
  }
}
