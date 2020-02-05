package frc.robot.math;

public class Coordinates2D extends Coordinates3D {

	public Coordinates2D(double x, double y) {
		super(x, y, 0);
		// TODO Auto-generated constructor stub
	}

	
	public static Coordinates2D fromRadial(double r, double phi) {
		return new Coordinates2D(r * Math.cos(phi), r * Math.sin(phi));
	}

	public double direction() {
		return Math.atan2(y, x);
	}

	public boolean isZeroVector() {
		return size() < 1E-6;
	}

	@Override
	public String toString() {
		return " x: " + x + "; y: " + y;
	}

	public Coordinates2D clone() {
		return new Coordinates2D(x, y);
	}
}
