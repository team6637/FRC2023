package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  private final Spark led = new Spark(1);
  private double power;
  private final double green = 0.77;
  private final double yellow = 0.69;
  private final double purple = 0.91;

  public LEDSubsystem() {
    power = yellow;
  }

  public void turnLEDgreen() {
    power = green;
  }

  public void turnLEDyellow() {
    power = yellow;
  }

  public void turnLEDpurple() {
    power = purple;
  }

  @Override
  public void periodic() {
    led.set(power);
  }
}