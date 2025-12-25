package org.firstinspires.ftc.teamcode.qubit.core;

public class FtcField {

}

class Point {
  public double x, y;

  public Point(double x, double y) {
    this.x = x;
    this.y = y;
  }

  public double distanceFrom(Point z) {
    return Math.sqrt(Math.pow(x - z.x, 2) + Math.pow(y - z.y, 2));
  }
}

class Triangle {
  public final Point A;
  public final Point B;
  public final Point C;
  public final double angleA;
  public final double angleB;
  public final double angleC;
  public final double sideAB;
  public final double sideBC;
  public final double sideCA;
  public final double area;
  public final double perimeter;

  public Triangle(Point a, Point b, Point c) {
    A = new Point(a.x, a.y);
    B = new Point(b.x, b.y);
    C = new Point(c.x, c.y);

    sideAB = A.distanceFrom(B);
    sideBC = B.distanceFrom(C);
    sideCA = C.distanceFrom(A);

    perimeter = (sideAB + sideBC + sideCA);
    double s = perimeter / 2.0;
    area = Math.sqrt(s * (s - sideAB) * (s - sideBC) * (s - sideCA));

    angleA = calculateAngle(sideBC, sideCA, sideAB);
    angleB = calculateAngle(sideCA, sideAB, sideBC);
    angleC = calculateAngle(sideAB, sideBC, sideCA);
  }

  private double calculateAngle(double sideA, double sideB, double sideC) {
    // Law of Cosines: cos(Angle) = (b^2 + c^2 - a^2) / (2bc)
    double cosAngleA = (sideB * sideB + sideC * sideC - sideA * sideA) / (2 * sideB * sideC);

    // Handle potential floating point inaccuracies by clamping value
    cosAngleA = Math.max(-1.0, Math.min(1.0, cosAngleA));
    return Math.acos(cosAngleA);
  }
}
