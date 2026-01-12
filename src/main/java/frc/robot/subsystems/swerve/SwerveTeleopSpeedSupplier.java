package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.Util;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Config.SwerveTeleop.applySlew;
import static frc.robot.Config.SwerveTeleop.applySniperToRotate;
import static frc.robot.Config.SwerveTeleop.deadband;
import static frc.robot.Config.SwerveTeleop.exponent;
import static frc.robot.Config.SwerveSubsystem.maxTeleopRotate;
import static frc.robot.Config.SwerveSubsystem.maxTeleopTranslate;
import static frc.robot.Config.SwerveTeleop.slewRate;
import static frc.robot.Config.SwerveTeleop.sniperFactor;
import static frc.robot.Config.SwerveTeleop.turboFactor;

/**
 * Supplier for ChassisSpeeds that processes stick input and provides
 * several useful options:
 * <ul>
 *
 *     <li>Max translate/rotate speed (to convert 0.0 - 1.0 values to
 *     a valid field speed)</li>
 *
 *     <li>Deadband (highly recommended for joysticks that don't
 *     report 0.0 values when centered)</li>
 *
 *     <li>Exponent (lore has it that squaring or cubing small
 *     input values gives better control)</li>
 *
 *     <li>"Turbo" and "Sniper" Factors for translation (allows you to
 *     implement stuff like a simple "double my top speed" control)</li>
 *
 *     <li>One-sided slew rate limiting for translation (i.e.
 *     don't accelerate from 0 instantaneously, but "snap back"
 *     to 0 if they release the joystick)</li>
 *
 * </ul>
 *
 * <p>For all configuration properties, translation units in feet and rotation
 * units are degrees. The resulting output is in meters and radians, in
 * accordance with prophecy.</p>
 *
 * <p>This class is implemented so it doesn't depend on a specific swerve
 * drive implementation; if it proves useful, and we can figure out a better
 * way to do configuration, we might later move it into a library.</p>
 */
public class SwerveTeleopSpeedSupplier implements Supplier<ChassisSpeeds> {

    /**
     * Setting this to true will make this command publish a bunch of info
     * to the dashboard that might be helpful for debugging
     */
    static final boolean enableLogging = false;

    /**
     * Represents the current mode
     */
    public enum Mode {
        TURBO,
        SNIPER,
        NONE
    }

    DoubleSupplier x;
    DoubleSupplier y;
    DoubleSupplier omega;
    BooleanSupplier turboTrigger;
    BooleanSupplier sniperTrigger;
    SlewRateLimiter limiterX;
    SlewRateLimiter limiterY;
    double inX;
    double inY;
    double inO;
    double lastX;
    double lastY;
    double lastOmega;

    /**
     * Creates a {@link SwerveTeleopSpeedSupplier}
     * @param x supplier for x speed (-1.0 - 1.0) (required)
     * @param y supplier for y speed (-1.0 - 1.0) (required)
     * @param omega supplier for rotation speed (-1.0 - 1.0) (required)
     * @param turboTrigger trigger for turbo mode (required)
     * @param sniperTrigger trigger for sniper mode (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public SwerveTeleopSpeedSupplier(DoubleSupplier x,
                                     DoubleSupplier y,
                                     DoubleSupplier omega,
                                     BooleanSupplier turboTrigger,
                                     BooleanSupplier sniperTrigger) {

        this.x = Objects.requireNonNull(x);
        this.y = Objects.requireNonNull(y);
        this.omega = Objects.requireNonNull(omega);
        this.turboTrigger = Objects.requireNonNull(turboTrigger);
        this.sniperTrigger = Objects.requireNonNull(sniperTrigger);
        this.lastX = Double.NaN;
        this.lastY = Double.NaN;
        this.lastOmega = Double.NaN;

        if (enableLogging) {
            SmartDashboard.putData("SwerveSpeedSupplier", builder -> {
                builder.addStringProperty("Mode", () -> getMode().toString(), null);
                builder.addDoubleProperty("InputX", () -> inX, null);
                builder.addDoubleProperty("InputY", () -> inY, null);
                builder.addDoubleProperty("InputOmega", () -> inO, null);
                builder.addDoubleProperty("SpeedX", () -> lastX, null);
                builder.addDoubleProperty("SpeedY", () -> lastY, null);
                builder.addDoubleProperty("SpeedOmega", () -> lastOmega, null);
            });
        }
    }

    /*
     * Calculates the current mode based on trigger positions; sniper mode
     * will override turbo mode if both are enabled
     */
    private Mode getMode() {
        if (sniperTrigger.getAsBoolean()) {
            return Mode.SNIPER;
        } else if (turboTrigger.getAsBoolean()) {
            return Mode.TURBO;
        } else {
            return Mode.NONE;
        }
    }

    /**
     * @return drive speeds based on current inputs
     */
    @Override
    public ChassisSpeeds get() {

        // get input from the joystick
        inX = x.getAsDouble();
        inY = y.getAsDouble();
        inO = omega.getAsDouble();

        // "condition" the values with deadband etc.
        lastX = conditionInput(inX);
        lastY = conditionInput(inY);
        lastOmega = conditionInput(inO);

        // ensure that the point defined by (x, y) lies on the unit
        // circle - when we scale them by the maximum translate speed
        // this will prevent us from shooting off too fast at an angle
        double d = Math.hypot(lastX, lastY);
        if (d > 1.0) {
            lastX /= d;
            lastY /= d;
        }

        // convert to speeds
        double mt = maxTeleopTranslate.getAsDouble();
        lastX *= mt;
        lastY *= mt;
        lastOmega *= maxTeleopRotate.getAsDouble();

        // update slew settings and apply slew rate limiting if necessary
        checkSlew();
        if (limiterX != null) {
            lastX = slewLimit(lastX, limiterX);
        }
        if (limiterY != null) {
            lastY = slewLimit(lastY, limiterY);
        }

        switch (getMode()) {

            // if we're in sniper mode, we might want to slow down rotation too
            case SNIPER:
                double sf = sniperFactor.getAsDouble();
                lastX *= sf;
                lastY *= sf;
                if (applySniperToRotate.getAsBoolean()) {
                    lastOmega *= sf;
                }
                break;

            // if we're in turbo mode, we only change translation
            case TURBO:
                double tf = turboFactor.getAsDouble();
                lastX *= tf;
                lastY *= tf;
                break;

            // nothing to do here
            case NONE:
                break;

        }

        return new ChassisSpeeds(
                Units.feetToMeters(lastX),
                Units.feetToMeters(lastY),
                Math.toRadians(lastOmega));
    }

    /*
     * Makes sure that we're either applying or not applying slew rate limiting
     * depending on the configuration property, and logs changes
     */
    private void checkSlew() {
        if (applySlew.getAsBoolean()) {
            if (limiterX == null) {
                double slew = slewRate.getAsDouble();
                limiterX = new SlewRateLimiter(slew);
                limiterY = new SlewRateLimiter(slew);
                Util.log("[swerve-teleop] enabling slew rate limiting @ %.2f", slew);
            }
        } else if (limiterX != null) {
            limiterX = null;
            limiterY = null;
            Util.log("[swerve-teleop] disabling slew rate limiting");
        }
    }

    /*
     * This applies slew limiting, but only when we're moving. If the desired
     * speed is 0, we will simply stop and reset the slew limit. This is to
     * keep manual targeting accurate.
     */
    private double slewLimit(double value, SlewRateLimiter limiter) {
        if (value == 0.0) {
            limiter.reset(0.0);
        } else {
            value = limiter.calculate(value);
        }
        return value;
    }

    /*
     * Applies deadband and exponent to input
     */
    private double conditionInput(double input) {
        input = MathUtil.clamp(input, -1.0, 1.0);
        input = MathUtil.applyDeadband(input, deadband.getAsDouble());
        input = Math.copySign(Math.pow(input, exponent.getAsDouble()), input);
        return input;
    }

    /**
     * @return a speed supplier using our "standard" controls (left stick
     * controls strafing, right stick controls turning, left trigger is sniper,
     * right trigger is turbo)
     */
    public static SwerveTeleopSpeedSupplier create(
            CommandXboxController controller) {

        // pushing right or forward on the joystick results in negative values, so
        // we invert them before using them
        DoubleSupplier leftX = () -> -controller.getLeftX();
        DoubleSupplier leftY = () -> -controller.getLeftY();
        DoubleSupplier rightX = () -> -controller.getRightX();

        // triggers controller sniper/turbo behavior
        BooleanSupplier sniperTrigger = () -> controller.getLeftTriggerAxis() > 0.5;
        BooleanSupplier turboTrigger = () -> controller.getRightTriggerAxis() > 0.5;

        return new SwerveTeleopSpeedSupplier(
                leftX,
                leftY,
                rightX,
                turboTrigger,
                sniperTrigger);
    }
}
