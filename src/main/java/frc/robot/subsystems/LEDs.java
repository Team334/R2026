package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Superstructure;
import frc.robot.utils.ShotParameters;

import java.util.Map;

import static edu.wpi.first.units.Units.Microseconds;

public class LEDs extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  private final Superstructure m_superstructure;
  private final ShotParameters m_shotParameters;

  public LEDs(Superstructure superstructure, ShotParameters shotParameters) {
    m_superstructure = superstructure;
    m_shotParameters = shotParameters;

    m_led = new AddressableLED(9);
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
  }

  @Override
  public void periodic() {
    LEDPattern currentPattern = selectPattern();

    currentPattern.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
  }

  private LEDPattern selectPattern() {
    if (m_superstructure.isClimbing()) {
      return LEDPattern.solid(Color.kPurple);
    }

    if (m_superstructure.isShooting() && m_shotParameters.isErrorSensitive) {
      return alternate(
        LEDPattern.solid(Color.kGreen), Units.Seconds.of(0.5),
        LEDPattern.solid(Color.kYellow), Units.Seconds.of(0.5)
      );
    }

    if (m_superstructure.isShooting()) {
      return LEDPattern.solid(Color.kYellow);
    }

    if (m_shotParameters.isErrorSensitive) {
      return LEDPattern.solid(Color.kGreen).blink(Units.Seconds.of(0.5));
    }

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red
          ? LEDPattern.solid(Color.kRed)
          : LEDPattern.solid(Color.kBlue);
    }

    return LEDPattern.solid(Color.kWhite);
  }

  public static LEDPattern alternate(LEDPattern patternA, Time timeA, LEDPattern patternB, Time timeB) {
    final long totalTimeMicros = (long) (timeA.in(Microseconds) + timeB.in(Microseconds));
    final long timeAMicros = (long) timeA.in(Microseconds);

    return (reader, writer) -> {
      if (RobotController.getTime() % totalTimeMicros < timeAMicros) {
        patternA.applyTo(reader, writer);
      } else {
        patternB.applyTo(reader, writer);
      }
    };
  }
}
