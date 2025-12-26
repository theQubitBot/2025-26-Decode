package org.firstinspires.ftc.teamcode.qubit.core.Field;

import com.pedropathing.geometry.Pose;

public class Triangle {
  public final Pose vertexA;
  public final Pose vertexB;
  public final Pose vertexC;
  public final double angleA;
  public final double angleB;
  public final double angleC;
  public final double sideAB;
  public final double sideBC;
  public final double sideCA;
  public final double area;
  public final double perimeter;
  private final double invertedDeterminant;

  public Triangle(Pose a, Pose b, Pose c) {
    vertexA = new Pose(a.getX(), a.getY(), a.getHeading());
    vertexB = new Pose(b.getX(), b.getY(), b.getHeading());
    vertexC = new Pose(c.getX(), c.getY(), c.getHeading());

    sideAB = vertexA.distanceFrom(vertexB);
    sideBC = vertexB.distanceFrom(vertexC);
    sideCA = vertexC.distanceFrom(vertexA);

    perimeter = (sideAB + sideBC + sideCA);
    double s = perimeter / 2.0;
    area = Math.sqrt(s * (s - sideAB) * (s - sideBC) * (s - sideCA));

    angleA = getAngle(sideBC, sideCA, sideAB);
    angleB = getAngle(sideCA, sideAB, sideBC);
    angleC = getAngle(sideAB, sideBC, sideCA);

    // Determinant used for normalization (denominator)
    // This is an optimization for fixed static triangles.
    invertedDeterminant = ((vertexB.getY() - vertexC.getY()) * (vertexA.getX() - vertexC.getX()) +
        (vertexC.getX() - vertexB.getX()) * (vertexA.getY() - vertexC.getY()));
  }

  private double getAngle(double sideA, double sideB, double sideC) {
    // Law of Cosines: cos(Angle) = (b^2 + c^2 - a^2) / (2bc)
    double cosAngleA = (sideB * sideB + sideC * sideC - sideA * sideA) / (2 * sideB * sideC);

    // Handle potential floating point inaccuracies by clamping value
    cosAngleA = Math.max(-1.0, Math.min(1.0, cosAngleA));
    return Math.acos(cosAngleA);
  }

  /**
   * Determines if the given point is inside the triangle
   * using Barycentric coordinates.
   *
   * @param p The given point.
   * @return True, if the given point is inside the triangle, false otherwise.
   */
  public boolean isInside(Pose p) {
    // Calculate Barycentric coordinates
    double w1 = ((vertexB.getY() - vertexC.getY()) * (p.getX() - vertexC.getX()) +
        (vertexC.getX() - vertexB.getX()) * (p.getY() - vertexC.getY())) / invertedDeterminant;
    double w2 = ((vertexC.getY() - vertexA.getY()) * (p.getX() - vertexC.getX()) +
        (vertexA.getX() - vertexC.getX()) * (p.getY() - vertexC.getY())) / invertedDeterminant;
    double w3 = 1.0 - w1 - w2;

    // Point is inside if all Barycentric coordinates are between 0 and 1
    return (w1 >= 0) && (w2 >= 0) && (w3 >= 0);
  }
}
