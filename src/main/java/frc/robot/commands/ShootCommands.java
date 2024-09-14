package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public final class ShootCommands {
    /**
     * Sends commands to the shooter and intake to start shooting.
     * @param shooter The shooter to start shooting
     * @param intake The intake to start shooting
     * @return A command to trigger shooting at the same time.
     */
    public static Command getShootCommand(Shooter shooter, Intake intake) {
        return shooter.getShootCommand().alongWith(intake.getShootCommand());
    }

    /**
     * Sends commands to the shooter and intake to start intaking
     * @param shooter The shooter to start intaking
     * @param intake The intake to start intaking
     * @return A command to trigger intaking at the same time
     */
    public static Command getIntakeCommand(Shooter shooter, Intake intake) {
        return shooter.getIntakeCommand().alongWith(intake.getIntakeCommand());
    }

    /**
     * Sends commands to the shooter and intake to stop moving.
     * @param shooter The shooter to stop moving
     * @param intake The intake to stop moving
     * @return A command to stop both at the same time.
     */
    public static Command getStopCommand(Shooter shooter, Intake intake) {
        return shooter.getStopCommand().alongWith(intake.getStopCommand());
    }
}
