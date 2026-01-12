package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

import java.util.function.DoubleSupplier;

/**
 * Adds the following useful functionality to a normal {@link PIDController}:
 * <ul>
 *
 *     <li>It doesn't use I - we generally stay away from it because of
 *     <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/common-control-issues.html#integral-term-windup">integral
 *     windup</a></li>
 *
 *     <li>Parameters are received from {@link DoubleSupplier} instances,
 *     so they can be managed via {@link edu.wpi.first.wpilibj.Preferences}.
 *     Updated values are read on {@link #reset()}.
 *     </li>
 *
 * </ul>
 */
public class PDController extends PIDController {

    final DoubleSupplier p;
    final DoubleSupplier d;
    final DoubleSupplier tolerance;

    /**
     * Creates a {@link PDController} with no max feedback
     * @param p supplier for the p parameter
     * @param d supplier for the d parameter
     * @param tolerance supplier for the tolerance parameter
     */
    public PDController(DoubleSupplier p,
                        DoubleSupplier d,
                        DoubleSupplier tolerance) {
        super(p.getAsDouble(), 0.0, d.getAsDouble());
        this.p = p;
        this.d = d;
        this.tolerance = tolerance;
        if (tolerance != null) {
            setTolerance(tolerance.getAsDouble());
        }
    }

    /**
     * Resets all parameters from their associated {@link DoubleSupplier} and
     * accumulated error
     */
    @Override
    public void reset() {
        setP(p.getAsDouble());
        setD(d.getAsDouble());
        if (tolerance != null) {
            setTolerance(tolerance.getAsDouble());
        }
        super.reset();
    }
}
