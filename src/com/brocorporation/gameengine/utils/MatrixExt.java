package com.brocorporation.gameengine.utils;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

public class MatrixExt extends Matrix {

	protected final static FloatBuffer matrixBuffer = ByteBuffer
			.allocateDirect(64).order(ByteOrder.nativeOrder()).asFloatBuffer();

	public static FloatBuffer getMatrix(final float[] matrix) {
		matrixBuffer.clear();
		matrixBuffer.put(matrix);
		matrixBuffer.flip();
		return matrixBuffer;
	}

	public static void logM(final String name, final float[] matrix) {
		final StringBuilder builder = new StringBuilder();
		int c = 0;
		final int colums = (int) Math.sqrt(matrix.length);
		for (int i = 0; i < matrix.length; i++) {
			builder.append(matrix[i] + "_");
			c++;
			if (c == colums) {
				System.out.println(name + "__" + new String(builder));
				builder.setLength(0);
				c = 0;
			}
		}
		System.out.println();
	}

	/**
	 * ab = b*a <br />
	 */
	public static void multiplyM3M3(float[] result, int resultOffset,
			float[] lhs, int lhsOffset, float[] rhs, int rhsOffset) {
		final float l0 = lhs[lhsOffset];
		final float l1 = lhs[lhsOffset + 1];
		final float l2 = lhs[lhsOffset + 2];
		final float l3 = lhs[lhsOffset + 3];
		final float l4 = lhs[lhsOffset + 4];
		final float l5 = lhs[lhsOffset + 5];
		final float l6 = lhs[lhsOffset + 6];
		final float l7 = lhs[lhsOffset + 7];
		final float l8 = lhs[lhsOffset + 8];
		final float r0 = rhs[rhsOffset];
		final float r1 = rhs[rhsOffset + 1];
		final float r2 = rhs[rhsOffset + 2];
		final float r3 = rhs[rhsOffset + 3];
		final float r4 = rhs[rhsOffset + 4];
		final float r5 = rhs[rhsOffset + 5];
		final float r6 = rhs[rhsOffset + 6];
		final float r7 = rhs[rhsOffset + 7];
		final float r8 = rhs[rhsOffset + 8];
		result[resultOffset] = r0 * l0 + r1 * l3 + r2 * l6;
		result[resultOffset + 1] = r0 * l1 + r1 * l4 + r2 * l7;
		result[resultOffset + 2] = r0 * l2 + r1 * l5 + r2 * l8;
		result[resultOffset + 3] = r3 * l0 + r4 * l3 + r5 * l6;
		result[resultOffset + 4] = r3 * l1 + r4 * l4 + r5 * l7;
		result[resultOffset + 5] = r3 * l2 + r4 * l5 + r5 * l8;
		result[resultOffset + 6] = r6 * l0 + r7 * l3 + r8 * l6;
		result[resultOffset + 7] = r6 * l1 + r7 * l4 + r8 * l7;
		result[resultOffset + 8] = r6 * l2 + r7 * l5 + r8 * l8;
	}

	public static void setM4(final float[] result, final float[] matrix) {
		System.arraycopy(matrix, 0, result, 0, matrix.length);
	}

	public static void castM3(final float result[], final float[] matrix) {
		result[0] = matrix[0];
		result[1] = matrix[1];
		result[2] = matrix[2];
		result[3] = matrix[4];
		result[4] = matrix[5];
		result[5] = matrix[6];
		result[6] = matrix[8];
		result[7] = matrix[9];
		result[8] = matrix[10];
	}

	public static boolean equalsM(final float[] matrix1, final float[] matrix2) {
		final int length = matrix1.length;
		if (length != matrix2.length)
			return false;
		for (int i = 0; i < length; i++) {
			if (matrix1[i] != matrix2[i])
				return false;
		}
		return true;
	}

	public static boolean invertM3(final float[] result, final float[] matrix) {
		final float m0 = matrix[0];
		final float m1 = matrix[1];
		final float m2 = matrix[2];
		final float m3 = matrix[3];
		final float m4 = matrix[4];
		final float m5 = matrix[5];
		final float m6 = matrix[6];
		final float m7 = matrix[7];
		final float m8 = matrix[8];

		final float m4x8subm7x5 = m4 * m8 - m7 * m5;
		final float m7x2submm1x8 = m7 * m2 - m1 * m8;
		final float m1x5submm4x2 = m1 * m5 - m4 * m2;

		float det = (m0 * (m4x8subm7x5) + m3 * (m7x2submm1x8) + m6
				* (m1x5submm4x2));
		if (det == 0) {
			return false;
		}
		det = 1F / det;
		result[0] = (m4x8subm7x5) * det;
		result[1] = (m7x2submm1x8) * det;
		result[2] = (m1x5submm4x2) * det;
		result[3] = (m6 * m5 - m3 * m8) * det;
		result[4] = (m0 * m8 - m6 * m2) * det;
		result[5] = (m3 * m2 - m0 * m5) * det;
		result[6] = (m3 * m7 - m6 * m4) * det;
		result[7] = (m6 * m1 - m0 * m7) * det;
		result[8] = (m0 * m4 - m3 * m1) * det;
		return true;
	}

	public static void transposeM3(final float[] result, final float[] matrix) {
		final float tmp1 = matrix[1];
		final float tmp2 = matrix[2];
		final float tmp5 = matrix[5];
		result[0] = matrix[0];
		result[1] = matrix[3];
		result[2] = matrix[6];
		result[3] = tmp1;
		result[4] = matrix[4];
		result[5] = matrix[7];
		result[6] = tmp2;
		result[7] = tmp5;
		result[8] = matrix[8];
	}

	public static void setIdentityM3(float[] result) {
		result[0] = result[4] = result[8] = 1;
		result[1] = result[2] = result[3] = result[5] = result[6] = result[7] = 0;
	}

	public static void setNullM(float[] result) {
		for (int i = result.length - 1; i >= 0; i--) {
			result[i] = 0;
		}
	}

	public static void setLookAtPosM(float[] rm, int rmOffset, float eyeX,
			float eyeY, float eyeZ, float centerX, float centerY,
			float centerZ, float upX, float upY, float upZ) {
		setLookAtDirM(rm, rmOffset, eyeX, eyeY, eyeZ, centerX - eyeX, centerY
				- eyeY, centerZ - eyeZ, upX, upY, upZ);
	}

	public static void setLookAtDirM(float[] rm, int rmOffset, float eyeX,
			float eyeY, float eyeZ, float dirX, float dirY, float dirZ,
			float upX, float upY, float upZ) {
		final float rlf = 1.0f / Matrix.length(dirX, dirY, dirZ);
		dirX *= rlf;
		dirY *= rlf;
		dirZ *= rlf;

		float sx = dirY * upZ - dirZ * upY;
		float sy = dirZ * upX - dirX * upZ;
		float sz = dirX * upY - dirY * upX;

		final float rls = 1.0f / Matrix.length(sx, sy, sz);
		sx *= rls;
		sy *= rls;
		sz *= rls;

		final float ux = sy * dirZ - sz * dirY;
		final float uy = sz * dirX - sx * dirZ;
		final float uz = sx * dirY - sy * dirX;

		rm[rmOffset] = sx;
		rm[rmOffset + 1] = ux;
		rm[rmOffset + 2] = -dirX;
		rm[rmOffset + 3] = 0;
		rm[rmOffset + 4] = sy;
		rm[rmOffset + 5] = uy;
		rm[rmOffset + 6] = -dirY;
		rm[rmOffset + 7] = 0;
		rm[rmOffset + 8] = sz;
		rm[rmOffset + 9] = uz;
		rm[rmOffset + 10] = -dirZ;
		rm[rmOffset + 11] = 0;

		rm[rmOffset + 12] = -sx * eyeX - sy * eyeY - sz * eyeZ;
		rm[rmOffset + 13] = -ux * eyeX - uy * eyeY - uz * eyeZ;
		rm[rmOffset + 14] = dirX * eyeX + dirY * eyeY + dirZ * eyeZ;
		rm[rmOffset + 15] = 1;
	}

	public static void setLookAtDirM(float[] rm, int rmOffset, Vector3f eye,
			Vector3f dir, Vector3f side, Vector3f up) {
		rm[rmOffset] = side.x;
		rm[rmOffset + 1] = up.x;
		rm[rmOffset + 2] = -dir.x;
		rm[rmOffset + 3] = 0;
		rm[rmOffset + 4] = side.y;
		rm[rmOffset + 5] = up.y;
		rm[rmOffset + 6] = -dir.y;
		rm[rmOffset + 7] = 0;
		rm[rmOffset + 8] = side.z;
		rm[rmOffset + 9] = up.z;
		rm[rmOffset + 10] = -dir.z;
		rm[rmOffset + 11] = 0;
		rm[rmOffset + 12] = -side.x * eye.x - side.y * eye.y - side.z * eye.z;
		rm[rmOffset + 13] = -up.x * eye.x - up.y * eye.y - up.z * eye.z;
		rm[rmOffset + 14] = dir.x * eye.x + dir.y * eye.y + dir.z * eye.z;
		rm[rmOffset + 15] = 1;
	}

	// 4x4 determinate
	// a*(f*k*p+g*l*n+h*j*o-n*k*h-o*l*f-p*j*g)
	// -e*(b*k*p+c*l*n+d*j*o-n*k*d-o*l*b-p*j*c)
	// +i*(b*g*p+c*h*n+d*b*o-n*g*d-o*h*b-p*f*c)
	// -m*(b*g*l+c*h*j+d*f*k-j*g*d-k*h*b-l*f*c)
}
