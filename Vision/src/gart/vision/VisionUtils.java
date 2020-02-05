package gart.vision;

import gart.math.Matrix;
import gart.math.SquareMatrix;
import gart.math.Vector;

public class VisionUtils {

	public static Vector solveCamera(double px0, double py0, double px1, double py1, double px2, double py2, double px3,
			double py3, double qx0, double qy0, double qx1, double qy1, double qx2, double qy2, double qx3, double qy3,
			double focalLength) {
		Vector p0 = new Vector(new double[] { px0, py0, 1 });
		Vector p1 = new Vector(new double[] { px1, py1, 1 });
		Vector p2 = new Vector(new double[] { px2, py2, 1 });
		Vector p3 = new Vector(new double[] { px3, py3, 1 });
		Vector q0 = new Vector(new double[] { qx0, qy0, 1 });
		Vector q1 = new Vector(new double[] { qx1, qy1, 1 });
		Vector q2 = new Vector(new double[] { qx2, qy2, 1 });
		Vector q3 = new Vector(new double[] { qx3, qy3, 1 });
		Vector q4 = new Vector(new double[] { 3.1304, -1.2404, 1 });
		Vector idealLine = new Vector(new double[] { 0, 0, 1 });
		SquareMatrix T = getTransformMat(p0, p1, p2, p3, q0, q1, q2, q3);
		SquareMatrix invT = T.clone().invert();
		System.out.println(Matrix.multiply(T, q4));
		System.out.println(Matrix.multiply(invT, q4));
		Vector VLI = Vector.multiply(invT.transpose(), idealLine);

//		System.out.println(VLI);
		double DI = Math.abs(VLI.get(2)) / Math.sqrt(VLI.get(0) * VLI.get(0) + VLI.get(1) * VLI.get(1));
//		Vector PPI = Vector.multiply(T, idealLine);
//		System.out.println(PPI.scale(1/PPI.get(2)));
		double theta = Math.atan(focalLength / DI);
		Vector VLO = Vector.multiply(T.transpose(), idealLine);
//		System.out.println(VLO);
		Vector PPO = Vector.multiply(invT, idealLine);
//		System.out.println(PPO);
		double DO = Math.abs((VLO.get(0) * PPO.get(0) + VLO.get(1) * PPO.get(1) + VLO.get(2) * PPO.get(2)) / PPO.get(2))
				/ Math.sqrt(VLO.get(0) * VLO.get(0) + VLO.get(1) * VLO.get(1));
		double phi = Math.atan(-VLO.get(1) / VLO.get(0));
		double XSGN = -Math.signum((VLO.get(0) * PPO.get(0) + VLO.get(1) * PPO.get(1) + VLO.get(2) * PPO.get(2))
				* VLO.get(0) * PPO.get(2));
		double YSGN = -Math.signum((VLO.get(0) * PPO.get(0) + VLO.get(1) * PPO.get(1) + VLO.get(2) * PPO.get(2))
				* VLO.get(1) * PPO.get(2));

		double DCP = DO * Math.sin(theta);
//		System.out.println(DCP);
		double XCP = XSGN * Math.abs(DCP * Math.sin(theta) * Math.cos(phi)) + PPO.get(0) / PPO.get(2);
		double YCP = YSGN * Math.abs(DCP * Math.sin(theta) * Math.sin(phi)) + PPO.get(1) / PPO.get(2);
		double ZCP = DCP * Math.cos(theta);
		return new Vector(new double[] { XCP, YCP, ZCP });
	}

	public static SquareMatrix getTransformMat(Vector p0, Vector p1, Vector p2, Vector p3, Vector q0, Vector q1,
			Vector q2, Vector q3) {
		SquareMatrix Q = new SquareMatrix(3);
		Q.setRow(q0, 0);
		Q.setRow(q1, 1);
		Q.setRow(q2, 2);
		SquareMatrix P = new SquareMatrix(3);
		P.setRow(p0, 0);
		P.setRow(p1, 1);
		P.setRow(p2, 2);
		Vector V = Vector.multiply(P.clone().invert().transpose(), p3);
		Vector R = Vector.multiply(Q.clone().invert().transpose(), q3);
		double w1 = (V.get(0) / V.get(2)) * (R.get(2) / R.get(0));
		double w2 = (V.get(1) / V.get(2)) * (R.get(2) / R.get(1));
		SquareMatrix w = SquareMatrix.identity(3);
		w.getMat()[0][0] = w1;
		w.getMat()[1][1] = w2;
		SquareMatrix T = SquareMatrix.multiply(w, P);
		T = SquareMatrix.multiply(Q.invert(), T).transpose();
		return T;
	}

	double[] getPictureCoords(double[] object_coords, double[] sensor) {
		// sensor[0,1,2]: normal unity vector of sensor, sensor[3]: distance from origin
		// to sensor. Sensor equation: ax+by+cz=d
		// output: x', y' with new origin on sensor, O'x' || xOy
		double a = sensor[0], b = sensor[1], c = sensor[2], d = sensor[3];
		double k = d / (a * object_coords[0] + b * object_coords[1] + c * object_coords[2]);
		double[] picture_coords_old = new double[3]; // position vector from new origin in old coordinates
		for (int i = 0; i < 3; i++) {
			picture_coords_old[i] = object_coords[i] * k - sensor[i] * d;
		}
		double[] picture_coords = new double[2];
		picture_coords[0] = (-b * picture_coords_old[0] + a * picture_coords_old[1]) / Math.sqrt(a * a + b * b);
		picture_coords[1] = (a * picture_coords_old[0] + b * picture_coords_old[1]
				- (a * a + b * b) / c * picture_coords_old[2]) / Math.sqrt((a * a + b * b) / (c * c));
		return picture_coords;
	}

}
