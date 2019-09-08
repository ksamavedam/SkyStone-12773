package org.firstinspires.ftc.teamcode;

public class Motion {
    private double[] values = new double[4];

    final int LENGTH = 4;

    public Motion() {
    }
//         parameters.vuforiaLicenseKey = "AQapiaj/////AAABmR9MFwGXtUXlokDNoVqBfPgVJQUtQjEGM5ThSHmsuy4picaSUk8W+xn1vM+EV1DbJfrr58EOVEJdMfLFvG4An8oN8YDvHB44IGFPAmQBdHv3RkhbYEgWU/guwcEjwIXtcRTRt/J0PmTZG2xnyDxFfAk+AOUVtLE/Ze481z/We0oTolHGpStuPrUhGKGQcY7noVFb2q2LU/3DRoUKg7R1P7y93lluKthHr2BXZSFuHqN4CmEzXdeJKQ+vZrJxorguqiIdyiNvJ1fjXAtCATQeLAOwGQWp7HlRb/T9UUf/KXLcYxKKQV3NPtvE3iPuRkDceF3IvbPbX3i32+MKXZqKFHhpZwomR9PpqEzaxgG8pJ1I";
    Motion(double[] values) {
        this.values = values;
    }

    Motion add(Motion other) {
        double[] out = new double[4];
        for (int i = 0; i != LENGTH; ++i) {
            out[i] = values[i] + other.getValue(i);
        }
        return new Motion(out);
    }

    public Motion scale(double x) {
        double[] out = new double[4];
        for (int i = 0; i != LENGTH; ++i) {
            out[i] = values[i] * x;
        }
        return new Motion(out);
    }

    double getValue(int i) {
        return values[i];
    }
}
