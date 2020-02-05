package frc.robot.math;

public class Matrix {
	private final double[][] mat;
	final int xSize, ySize;

	public Matrix(int xSize, int ySize) {
		this.xSize = xSize;
		this.ySize = ySize;
		mat = new double[xSize][ySize];
	}

	public void setMat(double[][] matrix) {
		if (matrix.length != xSize || mat[0].length != ySize) {
			return;
		}
		for (int x = 0; x < xSize; x++) {
			for (int y = 0; y < ySize; y++) {
				mat[x][y] = matrix[x][y];
			}
		}
	}

	public void setRow(Vector vector, int row) {
		if (vector.xSize == ySize || row < 0 || row > xSize) {
			for (int y = 0; y < ySize; y++) {
				getMat()[row][y] = vector.getMat()[y][0];
			}

		}
	}

	public void setCol(Vector vector, int col) {
		if (vector.xSize == xSize || col < 0 || col > ySize) {
			for (int x = 0; x < xSize; x++) {
				getMat()[x][col] = vector.getMat()[x][0];
			}

		}
	}

	public Matrix scale(double factor) {
		for (int x = 0; x < xSize; x++) {
			for (int y = 0; y < ySize; y++) {
				getMat()[x][y] *= factor;
			}
		}
		return this;
	}

	public Matrix transpose() {
		Matrix transposed = new Matrix(ySize, xSize);
		for (int x = 0; x < xSize; x++) {
			for (int y = 0; y < ySize; y++) {
				transposed.getMat()[y][x] = getMat()[x][y];
			}
		}
		return transposed;
	}

	public static Matrix add(Matrix a, Matrix b) {
		if (a.xSize != b.xSize || a.ySize != b.ySize) {
			return null;
		}
		Matrix result = new Matrix(a.xSize, b.ySize);
		for (int x = 0; x < result.xSize; x++) {
			for (int y = 0; y < result.ySize; y++) {
				result.getMat()[x][y] = a.getMat()[x][y] + b.getMat()[x][y];
			}
		}
		return result;
	}

	public static Matrix multiply(Matrix a, Matrix b) {
		if (a.ySize != b.xSize) {
			return null;
		}
		Matrix result = new Matrix(a.xSize, b.ySize);
		for (int x = 0; x < result.xSize; x++) {
			for (int y = 0; y < result.ySize; y++) {
				for (int z = 0; z < a.ySize; z++) {
					result.getMat()[x][y] += a.getMat()[x][z] * b.getMat()[z][y];
				}
			}
		}
		return result;
	}

	public Matrix clone() {
		Matrix clone = new Matrix(xSize, ySize);
		for (int x = 0; x < xSize; x++) {
			for (int y = 0; y < ySize; y++) {
				clone.getMat()[x][y] = getMat()[x][y];
			}
		}
		return clone;
	}

	public String toString() {
		StringBuilder a = new StringBuilder();
		for (int x = 0; x < xSize; x++) {
			a.append("{");
			for (int y = 0; y < ySize; y++) {
				a.append(getMat()[x][y]);
				if (y == ySize - 1) {
					a.append("}");
				}
				a.append(", ");
			}
			a.append("\r\n");
		}
		return a.toString();
	}

	public double[][] getMat() {
		return mat;
	}

}
