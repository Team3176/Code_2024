package team3176.robot.constants;

public class SuperStructureConstants {
  
    /**
   * How many amps the arm motor can use.
   */
  public static final int ARM_CURRENT_LIMIT_A = 15;

  /**
   * Percent output to run the arm up/down at
   */
  public static final double ARM_OUTPUT_POWER = 1;

  /**
   * How many amps the claw can use while picking up
   */
  public static final int CLAW_CURRENT_LIMIT_A = 30;
  public static final int CLAW_AMPS = 30;

  /**
   * How many amps the claw can use while holding
   */
  public static final int CLAW_HOLD_CURRENT_LIMIT_A = 5;

  /**
   * Percent output for inhaling
   */
  public static final double CLAW_OUTPUT_POWER_CUBE = .8;
  public static final double CLAW_OUTPUT_POWER_CONE = 1.0;

  /**
   * Percent output for holding
   */
  public static final double CLAW_HOLD_POWER = 0.14;
  public static final double CLAW_HOLD_CONE_FACTOR = 2;

  public static final int ARM_ENCODER_OFFSET = 50; 

  public static final double ARM_kP = 0.006;
  public static final double ARM_kI = 0; //.0025
  public static final double ARM_kD = 0; //.001
  public static final double ARM_kg = 0.2;
  public static final double ARM_TOLERANCE = 3;

  public static final double ARM_ZERO_POS = 150;
  public static final double ARM_CARRY_POS = ARM_ZERO_POS;
  public static final double ARM_CATCH_POS = 45 + ARM_ZERO_POS;
  public static final double ARM_MID_POS =  100 + ARM_ZERO_POS;
  public static final double ARM_HIGH_POS = 210 + ARM_ZERO_POS;
  public static final double ARM_SIM_OFFSET = 70 + ARM_ZERO_POS;
}
  