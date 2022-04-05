package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;

public class TwoBallAuto implements AutoInterface {
  private SequentialCommandGroup auto;

  public TwoBallAuto() {
    auto = new SequentialCommandGroup(
        new ParallelCommandGroup(
          new IntakeAndHold().withTimeout(3.5),
          new DriveStraightOpenLoop().withTimeout(3.5)
        ),
        new TurnToAngle(()->0 , ()->0, 180).withTimeout(1),
        new Shoot().withTimeout(6)
    );
  }

  public SequentialCommandGroup getAuto() {
    return auto;
  }
}
