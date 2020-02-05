package frc.robot.vision;

import java.util.Arrays;
import java.util.LinkedList;

import frc.robot.math.*;

public class CameraSolver {
	final double A[];
	final Coordinates3D[] P;
	final double[][] squared = new double[4][4];
	static final LinkedList<int[]> perm = heapPermutation(new int[] { 0, 1, 2, 3 });

	public CameraSolver(Coordinates3D... irlPoints) {
		P = irlPoints;
		A = getAreaList(irlPoints);
//		System.out.println(Arrays.toString(A));
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				if (i == j)
					continue;
				squared[i][j] = P[i].squaredDistanceTo(P[j]);
			}
		}
	}

	public void solve(Coordinates2D... q) {
		double[] B = getAreaList(q);
		double[][] C = new double[4][4];

		double[][] squaredH = new double[4][4];
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				if (i == j)
					continue;
				C[i][j] = B[j] / B[i] * A[i] / A[j];
				squaredH[i][j] = Math.pow((q[i].x - C[i][j] * q[j].x), 2) + Math.pow((q[i].y - C[i][j] * q[j].y), 2);
			}
		}
		getFocal(C, squaredH);
		double f = 790;
		System.out.println(f);
		double[][] R = new double[4][4];
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				if (i == j)
					continue;
				R[i][j] = Math.sqrt(squaredH[i][j] + f * f * (1 - C[i][j]) * (1 - C[i][j]));
			}
		}
		Coordinates3D[] Q = getCamCoordPoints(q, R, f);
//		System.out.println(Arrays.toString(Q));
		Matrix cal = new Matrix(3,24);
		int index = 0;
		for (int[] perm : CameraSolver.perm) {
			cal.setCol(getCamPos(Q, perm[0], perm[1], perm[2]), index);
			index++;
		}		
		System.out.println(getMedian(cal.getMat()[0]));
		System.out.println(getMedian(cal.getMat()[1]));
		System.out.println(getMedian(cal.getMat()[2]));
	}

//	public Vector getCamPos(Coordinates3D[] Q) {
//		Vector[] solutions = new Vector[12];
//
//	}

	public Vector getCamPos(Coordinates3D[] Q, int i, int j, int k) {
		Coordinates3D Qij = Q[i].getVectorTo(Q[j]);
		Coordinates3D Qik = Q[i].getVectorTo(Q[k]);
		SquareMatrix q = new SquareMatrix(3);
		q.setCol(Qij, 0);
		q.setCol(Qik, 1);
		q.setCol(Coordinates3D.crossProduct(Qij, Qik), 2);

		Coordinates3D Pij = P[i].getVectorTo(P[j]);
		Coordinates3D Pik = P[i].getVectorTo(P[k]);
		SquareMatrix p = new SquareMatrix(3);
		p.setCol(Pij, 0);
		p.setCol(Pik, 1);
		p.setCol(Coordinates3D.crossProduct(Pij, Pik), 2);

		SquareMatrix T = SquareMatrix.multiply(p, q.invert());

		return Vector.getVectorTo(Vector.multiply(T, Q[i]), P[i]);
	}

	public Coordinates3D[] getCamCoordPoints(Coordinates2D[] q, double[][] R, double f) {
		Coordinates3D[] Q = new Coordinates3D[4];
		for (int i = 0; i < 4; i++) {
			Q[i] = getCamCoordPoint(q, R, i, f);
		}
		return Q;
	}

	public Coordinates3D getCamCoordPoint(Coordinates2D[] q, double[][] R, int i, double f) {
		double[] x = new double[3];
		double[] y = new double[3];
		double[] z = new double[3];

		for (int j = 0; j < 4; j++) {
			if (j == i)
				continue;
			Coordinates3D camCoord = getCamCoordPoint(q, R, i, j, f);
			x[j > i ? j - 1 : j] = camCoord.x;
			y[j > i ? j - 1 : j] = camCoord.y;
			z[j > i ? j - 1 : j] = camCoord.z;

		}
		return new Coordinates3D(getMedian(x), getMedian(y), getMedian(z));
	}

	public double getMedian(double... arr) {
		Arrays.sort(arr);
		return (arr[arr.length / 2] + arr[(arr.length - 1) / 2]) / 2;
	}

	public double getAverage(double[] arr) {
		double avg = 0;
		for (int i = 0; i < arr.length; i++) {
			avg += arr[i];
		}
		return avg / arr.length;
	}

	public Coordinates3D getCamCoordPoint(Coordinates2D[] q, double[][] R, int i, int j, double f) {
		return new Coordinates3D(q[i].x * Math.sqrt(squared[i][j]) / R[i][j],
				q[i].y * Math.sqrt(squared[i][j]) / R[i][j], f * Math.sqrt(squared[i][j]) / R[i][j]);
	}

	public double getFocal(double[][] C, double[][] H) {
		double[] f = new double[24];
		int index = 0;
		for (int[] perm : CameraSolver.perm) {
			f[index] = getFocal(C, H, perm[0], perm[1], perm[2]);
			index++;
		}
		System.out.println(Arrays.toString(f));
		return getMedian(f);
	}

	public static LinkedList<int[]> heapPermutation(int a[]) {
		LinkedList<int[]> perm = new LinkedList<int[]>();
		heapPermutation(a, a.length, a.length, perm);
		return perm;
	}

	static void heapPermutation(int a[], int size, int n, LinkedList<int[]> perm) {
		// if size becomes 1 then prints the obtained
		// permutation
		if (size == 1) {
			perm.add(a.clone());
		}
		for (int i = 0; i < size; i++) {
			heapPermutation(a, size - 1, n, perm);

			// if size is odd, swap first and last
			// element
			if (size % 2 == 1) {
				int temp = a[0];
				a[0] = a[size - 1];
				a[size - 1] = temp;
			}

			// If size is even, swap ith and last
			// element
			else {
				int temp = a[i];
				a[i] = a[size - 1];
				a[size - 1] = temp;
			}
		}
	}

	public double getFocal(double[][] C, double[][] H, int i, int j, int k) {
//		System.out.println("s: "
//				+ (squared[i][j] * (1 - C[i][k]) * (1 - C[i][k]) - squared[i][k] * (1 - C[i][j]) * (1 - C[i][j])));
//		System.out.println("H: " + (squared[i][k] * H[i][j] - squared[i][j] * H[i][k]));
		return Math.sqrt((squared[i][k] * H[i][j] - squared[i][j] * H[i][k])
				/ (squared[i][j] * (1 - C[i][k]) * (1 - C[i][k]) - squared[i][k] * (1 - C[i][j]) * (1 - C[i][j])));
	}

	public static double[] getAreaList(Coordinates3D[] p) {
		return new double[] { triangleArea(1, 2, 3, p), triangleArea(0, 2, 3, p), triangleArea(0, 1, 3, p),
				triangleArea(0, 1, 2, p) };
	}

	public static double triangleArea(int a, int b, int c, Coordinates3D[] p) {
		return triangleArea(p[a], p[b], p[c]);
	}

	public static double triangleArea(Coordinates3D a, Coordinates3D b, Coordinates3D c) {
		return Math.abs(Coordinates3D.crossProduct(c.getVectorTo(a), c.getVectorTo(b)).size());
	}
}
