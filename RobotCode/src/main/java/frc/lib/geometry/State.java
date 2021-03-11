package frc.lib.geometry;

import frc.lib.util.CSVWritable;
import frc.lib.math.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();
}
