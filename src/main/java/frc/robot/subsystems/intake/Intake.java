package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** AdvantageKit-style Intake subsystem. Delegates hardware to an IntakeIO implementation. */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // Moving-average filters for current sensing (uses same pattern as Hood)
  private final LinearFilter pivotCurrentFilter = LinearFilter.movingAverage(5);
  private final LinearFilter rollerCurrentFilter = LinearFilter.movingAverage(5);

  @AutoLogOutput(key = "Intake/Pivot Current Filter Value")
  private double pivotCurrentFilterValue = 0.0;

  @AutoLogOutput(key = "Intake/Roller Current Filter Value")
  private double rollerCurrentFilterValue = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public static final double ZEROING_CURRENT_THRESHOLD_AMPS = 15.9;

  /** Extend pivot to an absolute distance in inches. */
  public void extendToDistance(double inches) {
    io.setPivotMotionMagicPosition(inches);
  }

  public Command goToDeployAndThenToUndeployCommand() {
    return runEnd(this::goToDeployedPosition, this::goToFramePerimeterPosition);
  }

  public void goToDeployedPosition() {
    io.setPivotMotionMagicPosition(IntakeConstants.deployDistance);
  }

  public void goToFramePerimeterPosition() {
    io.setPivotMotionMagicPosition(IntakeConstants.framePerimeterDistance);
    setRollerMotorPercentOutput(0);
  }

  public Command CommandGoToDistance(double inches) {
    return runOnce(() -> extendToDistance(inches));
  }

  public void setRollerMotorPercentOutput(double outputPercent) {
    io.setRollerOpenLoop(outputPercent);
  }

  public Command setRollerMotorPercentOutputAndThenTo0Command(double power) {
    return runEnd(() -> setRollerMotorPercentOutput(power), () -> setRollerMotorPercentOutput(0));
  }

  public Command setRollerMotorPercentOutputCommand(double power) {
    return runOnce(() -> setRollerMotorPercentOutput(power));
  }

  public Command goToDeployedPositionCommand() {
    return runOnce(this::goToDeployedPosition);
  }

  public Command goToFramePerimeterPositionCommand() {
    return runOnce(this::goToFramePerimeterPosition);
  }

  public Command goToMidPointPositionCommand() {
    return runOnce(() -> extendToDistance(IntakeConstants.midPoint));
  }

  /** Rezero the pivot encoder to zero. */
  public Command rezero() {
    return runOnce(() -> io.resetEncoder(0.0));
  }

  /** Drive toward the front hardstop with +2V until a current spike is detected, then rezero. */
  public Command currentZeroFrontHardstop() {
    return this.run(() -> io.setVoltage(+2.0))
        .until(() -> Math.abs(pivotCurrentFilterValue) > ZEROING_CURRENT_THRESHOLD_AMPS)
    .andThen(Commands.parallel(Commands.print("Intake Front Zeroed"), Commands.runOnce(() -> io.resetEncoder(4.197998))))
        .andThen(() -> io.setVoltage(0.0));
  }

  /** Drive toward the back hardstop with -2V until a current spike is detected, then rezero. */
  public Command currentZeroBackHardstop() {
    return this.run(() -> io.setVoltage(-2.0))
        .until(() -> Math.abs(pivotCurrentFilterValue) > ZEROING_CURRENT_THRESHOLD_AMPS)
    .andThen(Commands.parallel(Commands.print("Intake Back Zeroed"), Commands.runOnce(() -> io.resetEncoder(0.014404))))
        .andThen(() -> io.setVoltage(0.0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    // Update filters and log filtered current values
    pivotCurrentFilterValue = pivotCurrentFilter.calculate(inputs.pivotCurrentAmps);
    rollerCurrentFilterValue = rollerCurrentFilter.calculate(inputs.rollerCurrentAmps);
  }

  /** Returns the latest raw pivot stator current in amps from IO. */
  public double getPivotCurrentAmps() {
    return inputs.pivotCurrentAmps;
  }

  /** Returns the moving-average filtered pivot current (amps). */
  public double getPivotFilteredCurrentAmps() {
    return pivotCurrentFilterValue;
  }

  /** Returns the latest raw roller current (amps) from IO. */
  public double getRollerCurrentAmps() {
    return inputs.rollerCurrentAmps;
  }

  /** Returns the moving-average filtered roller current (amps). */
  public double getRollerFilteredCurrentAmps() {
    return rollerCurrentFilterValue;
  }
}
