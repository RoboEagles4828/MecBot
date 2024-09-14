package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RioCANIDs;

public class Shooter extends SubsystemBase {

    private final WPI_TalonSRX m_shooterLeft = new WPI_TalonSRX(RioCANIDs.kShooterLeft);
    private final WPI_TalonSRX m_shooterRight = new WPI_TalonSRX(RioCANIDs.kShooterRight);

    public Shooter() {
        // TODO: Configure m_shooter settings / directions here
        m_shooterLeft.setInverted(false); // These may need to be swapped
        m_shooterRight.setInverted(true);
    }

    /**
     * Moves the motors at the speed specified by {@link ShooterConstants.shootSpeed}
     * @return The command to move the motors
     */
    public Command getShootCommand() {
        return run(() -> setMotorSpeeds(ShooterConstants.shootSpeed));
    }

    /**
     * Moves the motors at the speed specified by {@link ShooterConstants.intakeSpeed}
     * @return The command to move the motors
     */
    public Command getIntakeCommand() {
        return run(() -> setMotorSpeeds(ShooterConstants.intakeSpeed));
    }

    /**
     * Stops the motors and sets them to neutral mode
     * @return The command to stop the motors
     */
    public Command getStopCommand() {
        return runOnce(() -> stopMotors());
    }

    /**
     * Sets the speeds of the motors.
     * Should be used for both intaking and shooting.
     * @param speed The speed of the motors
     */
    void setMotorSpeeds(final double speed) {
        m_shooterLeft.set(speed);
        m_shooterRight.set(-speed);
    }

    /**
     * Stops the motors by calling {@link WPI_TalonSRX.stopMotor}
     */
    void stopMotors() {
        m_shooterLeft.stopMotor();
        m_shooterRight.stopMotor();
    }
    
}
