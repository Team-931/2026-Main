package frc.robot;

public class MyMath {
    /** Finds the maximum of an arbtirary number of
     * arguments.
     */
static double max(double ... xs) {
    double mx = Double.NEGATIVE_INFINITY;
    for (double x : xs) mx = Math.max(mx, x);
    return mx;}

    /** Finds the minimum of an arbtirary number of
     * arguments.
     */
static double min(double ... xs) {
    double mx = Double.POSITIVE_INFINITY;
    for (double x : xs) mx = Math.min(mx, x);
    return mx;}
}
