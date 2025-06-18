package org.firstinspires.ftc.teamcode.qubit.core.enumerations;

/**
 * Geometric shapes are:
 * TARGET_UNKNOWN = 0 sides
 * TRIANGLE = 3 sides
 * SQUARE = 4 sides, aspect ratio = 1
 * RECTANGLE = 4 sides, aspect ratio != 1
 * PENTAGON = 5 sides
 * HEXAGON = 6 sides
 * CIRCLE >= 7 sides
 */
public enum GeometricShapeEnum {
    UNKNOWN(0, "Unknown"),
    TRIANGLE(3, "Triangle"),
    SQUARE(4, "Square"),
    RECTANGLE(4, "Rectangle"),
    PENTAGON(5, "Pentagon"),
    HEXAGON(6, "Hexagon"),
    CIRCLE(7, "Circle");

    public final int sides;
    public final String name;

    GeometricShapeEnum(int sides, String name) {
        this.sides = sides;
        this.name = name;
    }

    public boolean match(long sides) {
        if (this.sides == 6) return sides >= this.sides;
        return sides == this.sides;
    }
}
