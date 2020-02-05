package frc.robot.math;

public class Coordinates3D extends Vector {
	public double x, y, z;

	public Coordinates3D(double x, double y, double z) {
		super(x, y, z);
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public double squaredSize() {
		return x * x + y * y + z * z;
	}

	public double size() {
		return Math.sqrt(squaredSize());
	}

	public Coordinates3D scale(double scale) {
		x *= scale;
		y *= scale;
		z *= scale;
		
		return this;
	}
	
	public double squaredDistanceTo(Coordinates3D a) {
		return (a.x - x) * (a.x - x) + (a.y - y) * (a.y - y) + (a.z - z) * (a.z - z);
	}

	public double distanceTo(Coordinates3D a) {
		return Math.sqrt(squaredDistanceTo(a));
	}

	public static Coordinates3D sum(Coordinates3D a, Coordinates3D b) {
		return new Coordinates3D(a.x + b.x, a.y + b.y, a.z + b.z);
	}

	public Coordinates3D getVectorTo(Coordinates3D a) {
		return new Coordinates3D(a.x - x, a.y - y, a.z - z);
	}

	public Coordinates3D add(Coordinates3D a) {
		x += a.x;
		y += a.y;
		z += a.z;
		return this;
	}

	public Coordinates3D normalize() {
		double size = size();
		x /= size;
		y /= size;
		z /= size;
		return this;
	}

	public static Coordinates3D crossProduct(Coordinates3D a, Coordinates3D b) {
		return new Coordinates3D(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
	}
}
