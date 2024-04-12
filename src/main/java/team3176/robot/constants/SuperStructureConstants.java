package team3176.robot.constants;

public class SuperStructureConstants {

  /** How many amps the arm motor can use. */
  public static final int ARM_CURRENT_LIMIT_A = 15;

  /** Percent output to run the arm up/down at */
  public static final double ARM_OUTPUT_POWER = 1;

  /** How many amps the claw can use while picking up */
  public static final int CLAW_CURRENT_LIMIT_A = 30;

  public static final int CLAW_AMPS = 30;

  /** How many amps the claw can use while holding */
  public static final int CLAW_HOLD_CURRENT_LIMIT_A = 5;

  /** Percent output for inhaling */
  public static final double CLAW_OUTPUT_POWER_CUBE = .8;

  public static final double CLAW_OUTPUT_POWER_CONE = 1.0;

  /** Percent output for holding */
  public static final double CLAW_HOLD_POWER = 0.14;

  public static final double CLAW_HOLD_CONE_FACTOR = 2;

  public static final int ARM_ENCODER_OFFSET = 50;

  public static final double CLIMBLEFTj_kP = 0.006;
  public static final double CLIMBLEFT_kI = 0; // .0025
  public static final double CLIMBLEFT_kD = 0; // .001
  public static final double CLIMBLEFT_kg = 0.2;
  public static final double CLIMBLEFT_TOLERANCE = 3;
  public static final double CLIMBLEFT_ZERO_POS = 0.39;
  public static final double CLIMBLEFT_TOP_POS = 75 + CLIMBLEFT_ZERO_POS;
  public static final double CLIMBLEFT_SIM_OFFSET = 63 + CLIMBLEFT_ZERO_POS;

  public static final double CLIMBRIGHT_kP = 0.006;
  public static final double CLIMBRIGHT_kI = 0; // .0025
  public static final double CLIMBRIGHT_kD = 0; // .001
  public static final double CLIMBRIGHT_kg = 0.2;
  public static final double CLIMBRIGHT_TOLERANCE = 3;
  public static final double CLIMBRIGHT_ZERO_POS = 0.39;
  public static final double CLIMBRIGHT_TOP_POS = 75 + CLIMBRIGHT_ZERO_POS;
  public static final double CLIMBRIGHT_SIM_OFFSET = 63 + CLIMBRIGHT_ZERO_POS;

  public static final double INTAKE_PIVOT_kP = 0.006;
  public static final double INTAKE_PIVOT_kI = 0; // .0025
  public static final double INTAKE_PIVOT_kD = 0; // .001
  public static final double INTAKE_PIVOT_kg = 0.2;
  public static final double INTAKE_PIVOT_TOLERANCE = 3;
  public static final double INTAKE_PIVOT_ZERO_POS = 0.39;
  public static final double INTAKE_PIVOT_PICKUP_POS = 70 + INTAKE_PIVOT_ZERO_POS;
  public static final double INTAKE_PIVOT_CARRY_POS = INTAKE_PIVOT_ZERO_POS;
  public static final double INTAKE_PIVOT_SIM_OFFSET = 75 + INTAKE_PIVOT_ZERO_POS;
  /*   public static final int INTAKE_LASERCAN_DIST_TO_NOTE = 200;
   */
  public static final double INTAKE_ROLLER_kP = 0.006;
  public static final double INTAKE_ROLLER_kI = 0; // .0025
  public static final double INTAKE_ROLLER_kD = 0; // .001
  public static final double INTAKE_ROLLER_kg = 0.2;
  public static final double INTAKE_ROLLER_TOLERANCE = 3;

  public static final double TRANSFER_kP = 0.006;
  public static final double TRANSFER_kI = 0; // .0025
  public static final double TRANSFER_kD = 0; // .001
  public static final double TRANSFER_kg = 0.2;
  public static final double TRANSFER_TOLERANCE = 3;

  public static final double ANGLER_kP = 0.006;
  public static final double ANGLER_kI = 0; // .0025
  public static final double ANGLER_kD = 0; // .001
  public static final double ANGLER_kg = 0.2;
  public static final double ANGLER_TOLERANCE = 3;
  public static final double ANGLER_ZERO_POS = 70;
  public static final double ANGLER_MID_POS = ANGLER_ZERO_POS;
  public static final double ANGLER_HIGH_POS = 70 + ANGLER_ZERO_POS;
  public static final double ANGLER_SIM_OFFSET = 70 + ANGLER_ZERO_POS;

  public static final double SHOOTER_kP = 0.006;
  public static final double SHOOTER_kI = 0; // .0025
  public static final double SHOOTER_kD = 0; // .001
  public static final double SHOOTER_kg = 0.2;
  public static final double SHOOTER_TOLERANCE = 3;
  public static final double SHOOTER_ZERO_POS = 0;
  public static final double SHOOTER_PIVOT_CARRY_POS = SHOOTER_ZERO_POS;
  public static final double SHOOTER_PIVOT_TOP_POS = 30 + SHOOTER_ZERO_POS;
  public static final double SHOOTER_PIVOT_SIM_OFFSET = 70 + SHOOTER_ZERO_POS;

  public static final double SHOOTER_UPPERWHEEL_kP = 0.006;
  public static final double SHOOTER_UPPERWHEEL_kI = 0; // .0025
  public static final double SHOOTER_UPPERWHEEL_kD = 0; // .001
  public static final double SHOOTER_UPPERWHEEL_kg = 0.2;
  public static final double SHOOTER_UPPERWHEEL_TOLERANCE = 3;

  public static final double SHOOTER_LOWERWHEEL_kP = 0.006;
  public static final double SHOOTER_LOWERWHEEL_kI = 0; // .0025
  public static final double SHOOTER_LOWERWHEEL_kD = 0; // .001
  public static final double SHOOTER_LOWERWHEEL_kg = 0.2;
  public static final double SHOOTER_LOWERWHEEL_TOLERANCE = 3;
}
