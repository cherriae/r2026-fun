package frc.robot.utils;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

@Logged(name = "Shooting")
public class Shooting {
    @Logged(name = "Shooting/Target")
    private static Pose2d target = Pose2d.kZero;

    @Logged(name = "Shooting/VirtualTarget")
    private static Pose2d virtualTarget = Pose2d.kZero;

    @Logged(name = "Shooting/Heading")
    private static Rotation2d shotHeading = Rotation2d.kZero;

    @Logged(name = "Shooting/Newton Iterations")
    private static int newtonIterations = 0;

    @Logged(name = "Shooting/isValid")
    public static BooleanSupplier isValid = () -> false;

    @Logged(name = "Shooting/Converged")
    private static boolean converged = false;

    private static final int maxIterations = 10;

    public static Matrix<N3, N1> vec3(double a, double b, double c) {
        return new Matrix<N3, N1>(N3(), N1(), new double[] {a, b, c});
    }

    public Shooting() {
        isValid = () -> true;
        converged = true;
    }

    public Shooting(
        boolean shooterReady,
        boolean headingWithinTolerance, // fix later with actual heading check
        Pose2d robotPose
    ) {
        isValid = () -> {
            return 
                shooterReady && 
                headingWithinTolerance && 
                FieldUtil.inAllianceZone(robotPose) && 
                converged; 
        };
    }

    // newton's method to find the heading to shoot at to hit the target
    public static void calculateShotHeading(Pose2d robotPose) {
    }

    public Rotation2d getShotHeading() {
        return shotHeading;
    }

    public Pose2d getTarget() {
        return target;
    }

    public Pose2d getVirtualTarget() {
        return virtualTarget;
    }

    public void setShotHeading(Rotation2d shotHeading) {
        Shooting.shotHeading = shotHeading;
    }

    public void setTarget(Translation2d target) {
        Shooting.target = new Pose2d(target, Rotation2d.kZero);
    }

    public void setVirtualTarget(Translation2d virtualTarget) {
        Shooting.virtualTarget = new Pose2d(virtualTarget, Rotation2d.kZero);
    }
}
