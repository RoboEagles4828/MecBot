package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWMIDs;
import frc.robot.subsystems.shooter.ShooterConstants;

public class Intake extends SubsystemBase {

    private final VictorSP m_intake = new VictorSP(PWMIDs.kIntake);

    public Intake() {}

    /**
     * Moves the motors at the speed specified by {@link ShooterConstants.shootSpeed}
     * @return The command to move the motors
     */
    public Command getShootCommand() {
        return run(() -> setMotorSpeed(IntakeConstants.shootSpeed));
    }

    /**
     * Moves the motors at the speed specified by {@link ShooterConstants.intakeSpeed}
     * @return The command to move the motors
     */
    public Command getIntakeCommand() {
        return run(() -> setMotorSpeed(IntakeConstants.intakeSpeed));
    }

    /**
     * Stops the motors and sets them to neutral mode
     * @return The command to stop the motors
     */
    public Command getStopCommand() {
        return runOnce(() -> stopMotor());
    }

    /**
     * Sets the speed of the motors.
     * Should be used for both intaking and shooting.
     * @param speed The speed of the motor
     */
    void setMotorSpeed(final double speed) {
        m_intake.set(speed);
    }

    /**
     * Stops the motor by calling {@link WPI_TalonSRX.stopMotor}
     */
    void stopMotor() {
        m_intake.stopMotor();
    }
    
}
