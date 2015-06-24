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
			builder.append(matrix[i] + "\t");
			c++;
			if (c == colums) {
				System.out.println(name + "\t" + builder.toString());
				builder.setLength(0);
				c = 0;
			}
		}
	}

	public static void multiplyMM(float[] result, int resultOffset,
			float[] lhs, int lhsOffset, float[] rhs, int rhsOffset) {
		Matrix.multiplyMM(result, resultOffset, rhs, rhsOffset, lhs, lhsOffset);
	}

	public static void multiplyVM(float[] resultVec, int resultVecOffset,
			float[] lhsVec, int lhsVecOffset, float[] rhsMat, int rhsMatOffset) {
		Matrix.multiplyMV(resultVec, resultVecOffset, rhsMat, rhsMatOffset,
				lhsVec, lhsVecOffset);
	}

	public static void multiplyMV(float[] resultVec, int resultVecOffset,
			float[] lhsMat, int lhsMatOffset, float[] rhsVec, int rhsVecOffset) {
		Matrix.multiplyVM(resultVec, resultVecOffset, rhsVec, rhsVecOffset,
				lhsMat, lhsMatOffset);
	}

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
		result[resultOffset] = l0 * r0 + l1 * r3 + l2 * r6;
		result[resultOffset + 1] = l0 * r1 + l1 * r4 + l2 * r7;
		result[resultOffset + 2] = l0 * r2 + l1 * r5 + l2 * r8;
		result[resultOffset + 3] = l3 * r0 + l4 * r3 + l5 * r6;
		result[resultOffset + 4] = l3 * r1 + l4 * r4 + l5 * r7;
		result[resultOffset + 5] = l3 * r2 + l4 * r5 + l5 * r8;
		result[resultOffset + 6] = l6 * r0 + l7 * r3 + l8 * r6;
		result[resultOffset + 7] = l6 * r1 + l7 * r4 + l8 * r7;
		result[resultOffset + 8] = l6 * r2 + l7 * r5 + l8 * r8;
	}

	public static void multiplyD3M3(float[] result, int resultOffset,
			float[] lhs, int lhsOffset, Vector3f diagonale) {
		result[resultOffset] = diagonale.x * lhs[lhsOffset];
		result[resultOffset + 1] = diagonale.x * lhs[lhsOffset + 1];
		result[resultOffset + 2] = diagonale.x * lhs[lhsOffset + 2];
		result[resultOffset + 3] = diagonale.y * lhs[lhsOffset + 3];
		result[resultOffset + 4] = diagonale.y * lhs[lhsOffset + 4];
		result[resultOffset + 5] = diagonale.y * lhs[lhsOffset + 5];
		result[resultOffset + 6] = diagonale.z * lhs[lhsOffset + 6];
		result[resultOffset + 7] = diagonale.z * lhs[lhsOffset + 7];
		result[resultOffset + 8] = diagonale.z * lhs[lhsOffset + 8];
	}

	public static void multiplyM3D3(float[] result, int resultOffset,
			Vector3f diagonale, float[] rhs, int rhsOffset) {
		result[resultOffset] = rhs[rhsOffset] * diagonale.x;
		result[resultOffset + 1] = rhs[rhsOffset + 1] * diagonale.y;
		result[resultOffset + 2] = rhs[rhsOffset + 2] * diagonale.z;
		result[resultOffset + 3] = rhs[rhsOffset + 3] * diagonale.x;
		result[resultOffset + 4] = rhs[rhsOffset + 4] * diagonale.y;
		result[resultOffset + 5] = rhs[rhsOffset + 5] * diagonale.z;
		result[resultOffset + 6] = rhs[rhsOffset + 6] * diagonale.x;
		result[resultOffset + 7] = rhs[rhsOffset + 7] * diagonale.y;
		result[resultOffset + 8] = rhs[rhsOffset + 8] * diagonale.z;
	}

	public static void multiplyM3D3MT3(float[] result, float[] rot,
			Vector3f diag) {
		final float ax = rot[0] * diag.x;
		final float by = rot[1] * diag.y;
		final float cz = rot[2] * diag.z;
		final float dx = rot[3] * diag.x;
		final float ey = rot[4] * diag.y;
		final float fz = rot[5] * diag.z;
		result[0] = rot[0] * ax + rot[1] * by + rot[2] * cz;
		result[1] = ax * rot[3] + by * rot[4] + cz * rot[5];
		result[2] = ax * rot[6] + by * rot[7] + cz * rot[8];
		result[4] = rot[3] * dx + rot[4] * ey + rot[5] * fz;
		result[5] = dx * rot[6] + ey * rot[7] + fz * rot[8];
		result[8] = rot[6] * rot[6] * diag.x + rot[7] * rot[7] * diag.y
				+ rot[8] * rot[8] * diag.z;
		result[3] = result[1];
		result[6] = result[2];
		result[7] = result[5];
	}

	public static void multiplyMT3D3M3(float[] result, float[] rot,
			Vector3f diag) {
		final float ax = rot[0] * diag.x;
		final float by = rot[3] * diag.y;
		final float cz = rot[6] * diag.z;
		final float dx = rot[1] * diag.x;
		final float ey = rot[4] * diag.y;
		final float fz = rot[7] * diag.z;
		result[0] = rot[0] * ax + rot[3] * by + rot[6] * cz;
		result[3] = ax * rot[1] + by * rot[4] + cz * rot[7];
		result[6] = ax * rot[2] + by * rot[5] + cz * rot[8];
		result[4] = rot[1] * dx + rot[4] * ey + rot[7] * fz;
		result[7] = dx * rot[2] + ey * rot[5] + fz * rot[8];
		result[8] = rot[2] * rot[2] * diag.x + rot[5] * rot[5] * diag.y
				+ rot[8] * rot[8] * diag.z;
		result[1] = result[3];
		result[2] = result[6];
		result[5] = result[7];
	}

	public static void setM(final float[] result, final float[] matrix) {
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

		final float m4x8subm7x5 = m4 * m8 - m5 * m7;
		final float m7x2submm1x8 = m2 * m7 - m1 * m8;
		final float m1x5submm4x2 = m1 * m5 - m2 * m4;

		float det = (m0 * (m4x8subm7x5) + m3 * (m7x2submm1x8) + m6
				* (m1x5submm4x2));
		if (det == 0) {
			return false;
		}
		det = 1F / det;
		result[0] = (m4x8subm7x5) * det;
		result[1] = (m7x2submm1x8) * det;
		result[2] = (m1x5submm4x2) * det;
		result[3] = (m5 * m6 - m3 * m8) * det;
		result[4] = (m0 * m8 - m2 * m6) * det;
		result[5] = (m2 * m3 - m0 * m5) * det;
		result[6] = (m3 * m7 - m4 * m6) * det;
		result[7] = (m1 * m6 - m0 * m7) * det;
		result[8] = (m0 * m4 - m1 * m3) * det;
		return true;
	}

	public static void transposeM3(final float[] result, final float[] matrix) {
		result[0] = matrix[0];
		result[4] = matrix[4];
		result[8] = matrix[8];
		float tmp = matrix[1];
		result[1] = matrix[3];
		result[3] = tmp;
		tmp = matrix[2];
		result[2] = matrix[6];
		result[6] = tmp;
		tmp = matrix[5];
		result[5] = matrix[7];
		result[7] = tmp;

	}

	public static void transposeM4(final float[] result, final float[] matrix) {
		result[0] = matrix[0];
		result[5] = matrix[5];
		result[10] = matrix[10];
		result[15] = matrix[15];
		float tmp = matrix[1];
		result[1] = matrix[4];
		result[4] = tmp;
		tmp = matrix[2];
		result[2] = matrix[8];
		result[8] = tmp;
		tmp = matrix[3];
		result[3] = matrix[12];
		result[12] = tmp;
		tmp = matrix[6];
		result[6] = matrix[9];
		result[9] = tmp;
		tmp = matrix[7];
		result[7] = matrix[13];
		result[13] = tmp;
		tmp = matrix[11];
		result[11] = matrix[14];
		result[14] = tmp;
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
		final float rlf = 1.0f / length(dirX, dirY, dirZ);
		dirX *= rlf;
		dirY *= rlf;
		dirZ *= rlf;

		float sx = dirY * upZ - dirZ * upY;
		float sy = dirZ * upX - dirX * upZ;
		float sz = dirX * upY - dirY * upX;

		final float rls = 1.0f / length(sx, sy, sz);
		sx *= rls;
		sy *= rls;
		sz *= rls;

		final float ux = sy * dirZ - sz * dirY;
		final float uy = sz * dirX - sx * dirZ;
		final float uz = sx * dirY - sy * dirX;

		rm[rmOffset] = sx;
		rm[rmOffset + 1] = sy;
		rm[rmOffset + 2] = sz;
		rm[rmOffset + 3] = -sx * eyeX - sy * eyeY - sz * eyeZ;
		rm[rmOffset + 4] = ux;
		rm[rmOffset + 5] = uy;
		rm[rmOffset + 6] = uz;
		rm[rmOffset + 7] = -ux * eyeX - uy * eyeY - uz * eyeZ;
		rm[rmOffset + 8] = -dirX;
		rm[rmOffset + 9] = -dirY;
		rm[rmOffset + 10] = -dirZ;
		rm[rmOffset + 11] = dirX * eyeX + dirY * eyeY + dirZ * eyeZ;
		rm[rmOffset + 12] = 0.0f;
		rm[rmOffset + 13] = 0.0f;
		rm[rmOffset + 14] = 0.0f;
		rm[rmOffset + 15] = 1.0f;
	}

	public static void setLookAtDirM(float[] rm, int rmOffset, Vector3f eye,
			Vector3f dir, Vector3f side, Vector3f up) {
		rm[rmOffset] = side.x;
		rm[rmOffset + 1] = side.y;
		rm[rmOffset + 2] = side.z;
		rm[rmOffset + 3] = -side.x * eye.x - side.y * eye.y - side.z * eye.z;
		rm[rmOffset + 4] = up.x;
		rm[rmOffset + 5] = up.y;
		rm[rmOffset + 6] = up.z;
		rm[rmOffset + 7] = -up.x * eye.x - up.y * eye.y - up.z * eye.z;
		rm[rmOffset + 8] = -dir.x;
		rm[rmOffset + 9] = -dir.y;
		rm[rmOffset + 10] = -dir.z;
		rm[rmOffset + 11] = dir.x * eye.x + dir.y * eye.y + dir.z * eye.z;
		rm[rmOffset + 12] = 0.0f;
		rm[rmOffset + 13] = 0.0f;
		rm[rmOffset + 14] = 0.0f;
		rm[rmOffset + 15] = 1.0f;
	}

	// 4x4 determinate
	// a*(f*k*p+g*l*n+h*j*o-n*k*h-o*l*f-p*j*g)
	// -e*(b*k*p+c*l*n+d*j*o-n*k*d-o*l*b-p*j*c)
	// +i*(b*g*p+c*h*n+d*b*o-n*g*d-o*h*b-p*f*c)
	// -m*(b*g*l+c*h*j+d*f*k-j*g*d-k*h*b-l*f*c)

	private final static float[] sTemp = new float[32];

	/**
	 * Computes an orthographic projection matrix.
	 * 
	 * @param m
	 *            returns the result
	 * @param mOffset
	 * @param left
	 * @param right
	 * @param bottom
	 * @param top
	 * @param near
	 * @param far
	 */
	public static void orthoM(float[] m, int mOffset, float left, float right,
			float bottom, float top, float near, float far) {
		if (left == right) {
			throw new IllegalArgumentException("left == right");
		}
		if (bottom == top) {
			throw new IllegalArgumentException("bottom == top");
		}
		if (near == far) {
			throw new IllegalArgumentException("near == far");
		}

		final float r_width = 1.0f / (right - left);
		final float r_height = 1.0f / (top - bottom);
		final float r_depth = 1.0f / (far - near);
		final float x = 2.0f * (r_width);
		final float y = 2.0f * (r_height);
		final float z = -2.0f * (r_depth);
		final float tx = -(right + left) * r_width;
		final float ty = -(top + bottom) * r_height;
		final float tz = -(far + near) * r_depth;
		m[mOffset + 0] = x;
		m[mOffset + 1] = 0.0f;
		m[mOffset + 2] = 0.0f;
		m[mOffset + 3] = tx;
		m[mOffset + 4] = 0.0f;
		m[mOffset + 5] = y;
		m[mOffset + 6] = 0.0f;
		m[mOffset + 7] = ty;
		m[mOffset + 8] = 0.0f;
		m[mOffset + 9] = 0.0f;
		m[mOffset + 10] = z;
		m[mOffset + 11] = tz;
		m[mOffset + 12] = 0.0f;
		m[mOffset + 13] = 0.0f;
		m[mOffset + 14] = 0.0f;
		m[mOffset + 15] = 1.0f;
	}

	/**
	 * Defines a projection matrix in terms of six clip planes.
	 * 
	 * @param m
	 *            the float array that holds the output perspective matrix
	 * @param offset
	 *            the offset into float array m where the perspective matrix
	 *            data is written
	 * @param left
	 * @param right
	 * @param bottom
	 * @param top
	 * @param near
	 * @param far
	 */
	public static void frustumM(float[] m, int offset, float left, float right,
			float bottom, float top, float near, float far) {
		if (left == right) {
			throw new IllegalArgumentException("left == right");
		}
		if (top == bottom) {
			throw new IllegalArgumentException("top == bottom");
		}
		if (near == far) {
			throw new IllegalArgumentException("near == far");
		}
		if (near <= 0.0f) {
			throw new IllegalArgumentException("near <= 0.0f");
		}
		if (far <= 0.0f) {
			throw new IllegalArgumentException("far <= 0.0f");
		}
		final float r_width = 1.0f / (right - left);
		final float r_height = 1.0f / (top - bottom);
		final float r_depth = 1.0f / (near - far);
		final float x = 2.0f * (near * r_width);
		final float y = 2.0f * (near * r_height);
		final float A = (right + left) * r_width;
		final float B = (top + bottom) * r_height;
		final float C = (far + near) * r_depth;
		final float D = 2.0f * (far * near * r_depth);
		m[offset + 0] = x;
		m[offset + 1] = 0.0f;
		m[offset + 2] = A;
		m[offset + 3] = 0.0f;
		m[offset + 4] = 0.0f;
		m[offset + 5] = y;
		m[offset + 6] = B;
		m[offset + 7] = 0.0f;
		m[offset + 8] = 0.0f;
		m[offset + 9] = 0.0f;
		m[offset + 10] = C;
		m[offset + 11] = D;
		m[offset + 12] = 0.0f;
		m[offset + 13] = 0.0f;
		m[offset + 14] = -1.0f;
		m[offset + 15] = 0.0f;
	}

	/**
	 * Defines a projection matrix in terms of a field of view angle, an aspect
	 * ratio, and z clip planes.
	 * 
	 * @param m
	 *            the float array that holds the perspective matrix
	 * @param offset
	 *            the offset into float array m where the perspective matrix
	 *            data is written
	 * @param fovy
	 *            field of view in y direction, in degrees
	 * @param aspect
	 *            width to height aspect ratio of the viewport
	 * @param zNear
	 * @param zFar
	 */
	public static void perspectiveM(float[] m, int offset, float fovy,
			float aspect, float zNear, float zFar) {
		float f = 1.0f / (float) Math.tan(fovy * (Math.PI / 360.0));
		float rangeReciprocal = 1.0f / (zNear - zFar);
		m[offset + 0] = f / aspect;
		m[offset + 1] = 0.0f;
		m[offset + 2] = 0.0f;
		m[offset + 3] = 0.0f;
		m[offset + 4] = 0.0f;
		m[offset + 5] = f;
		m[offset + 6] = 0.0f;
		m[offset + 7] = 0.0f;
		m[offset + 8] = 0.0f;
		m[offset + 9] = 0.0f;
		m[offset + 10] = (zFar + zNear) * rangeReciprocal;
		m[offset + 11] = 2.0f * zFar * zNear * rangeReciprocal;
		m[offset + 12] = 0.0f;
		m[offset + 13] = 0.0f;
		m[offset + 14] = -1.0f;
		m[offset + 15] = 0.0f;
	}

	/**
	 * Computes the length of a vector.
	 * 
	 * @param x
	 *            x coordinate of a vector
	 * @param y
	 *            y coordinate of a vector
	 * @param z
	 *            z coordinate of a vector
	 * @return the length of a vector
	 */
	public static float length(float x, float y, float z) {
		return (float) Math.sqrt(x * x + y * y + z * z);
	}

	/**
	 * Sets matrix m to the identity matrix.
	 * 
	 * @param sm
	 *            returns the result
	 * @param smOffset
	 *            index into sm where the result matrix starts
	 */
	public static void setIdentityM(float[] sm, int smOffset) {
		for (int i = 0; i < 16; i++) {
			sm[smOffset + i] = 0;
		}
		for (int i = 0; i < 16; i += 5) {
			sm[smOffset + i] = 1.0f;
		}
	}

	/**
	 * Scales matrix m by x, y, and z, putting the result in sm.
	 * <p>
	 * m and sm must not overlap.
	 * 
	 * @param sm
	 *            returns the result
	 * @param smOffset
	 *            index into sm where the result matrix starts
	 * @param m
	 *            source matrix
	 * @param mOffset
	 *            index into m where the source matrix starts
	 * @param x
	 *            scale factor x
	 * @param y
	 *            scale factor y
	 * @param z
	 *            scale factor z
	 */
	public static void scaleM(float[] sm, int smOffset, float[] m, int mOffset,
			float x, float y, float z) {
		for (int i = 0; i < 16; i += 4) {
			int smi = smOffset + i;
			int mi = mOffset + i;
			sm[smi] = m[mi] * x;
			sm[1 + smi] = m[1 + mi] * y;
			sm[2 + smi] = m[2 + mi] * z;
			sm[3 + smi] = m[3 + mi];
		}
	}

	/**
	 * Scales matrix m in place by sx, sy, and sz.
	 * 
	 * @param m
	 *            matrix to scale
	 * @param mOffset
	 *            index into m where the matrix starts
	 * @param x
	 *            scale factor x
	 * @param y
	 *            scale factor y
	 * @param z
	 *            scale factor z
	 */
	public static void scaleM(float[] m, int mOffset, float x, float y, float z) {
		for (int i = 0; i < 16; i += 4) {
			int mi = mOffset + i;
			m[mi] *= x;
			m[1 + mi] *= y;
			m[2 + mi] *= z;
		}
	}

	/**
	 * Translates matrix m by x, y, and z, putting the result in tm.
	 * <p>
	 * m and tm must not overlap.
	 * 
	 * @param tm
	 *            returns the result
	 * @param tmOffset
	 *            index into sm where the result matrix starts
	 * @param m
	 *            source matrix
	 * @param mOffset
	 *            index into m where the source matrix starts
	 * @param x
	 *            translation factor x
	 * @param y
	 *            translation factor y
	 * @param z
	 *            translation factor z
	 */
	public static void translateM(float[] tm, int tmOffset, float[] m,
			int mOffset, float x, float y, float z) {
		for (int i = 0; i < 12; i += 4) {
			tm[tmOffset + i] = m[mOffset + i];
			tm[tmOffset + i + 1] = m[mOffset + i + 1];
			tm[tmOffset + i + 2] = m[mOffset + i + 2];
		}
		for (int i = 0; i < 16; i += 4) {
			int tmi = tmOffset + i;
			int mi = mOffset + i;
			tm[3 + tmi] = m[mi] * x + m[1 + mi] * y + m[2 + mi] * z + m[3 + mi];
		}
	}

	/**
	 * Translates matrix m by x, y, and z in place.
	 * 
	 * @param m
	 *            matrix
	 * @param mOffset
	 *            index into m where the matrix starts
	 * @param x
	 *            translation factor x
	 * @param y
	 *            translation factor y
	 * @param z
	 *            translation factor z
	 */
	public static void translateM(float[] m, int mOffset, float x, float y,
			float z) {
		for (int i = 0; i < 16; i += 4) {
			int mi = mOffset + i;
			m[3 + mi] += m[mi] * x + m[1 + mi] * y + m[2 + mi] * z;
		}
	}

	/**
	 * Rotates matrix m by angle a (in degrees) around the axis (x, y, z).
	 * <p>
	 * m and rm must not overlap.
	 * 
	 * @param rm
	 *            returns the result
	 * @param rmOffset
	 *            index into rm where the result matrix starts
	 * @param m
	 *            source matrix
	 * @param mOffset
	 *            index into m where the source matrix starts
	 * @param a
	 *            angle to rotate in degrees
	 * @param x
	 *            X axis component
	 * @param y
	 *            Y axis component
	 * @param z
	 *            Z axis component
	 */
	public static void rotateM(float[] rm, int rmOffset, float[] m,
			int mOffset, float a, float x, float y, float z) {
		synchronized (sTemp) {
			setRotateM(sTemp, 0, a, x, y, z);
			multiplyMM(rm, rmOffset, sTemp, 0, m, mOffset);
		}
	}

	/**
	 * Rotates matrix m in place by angle a (in degrees) around the axis (x, y,
	 * z).
	 * 
	 * @param m
	 *            source matrix
	 * @param mOffset
	 *            index into m where the matrix starts
	 * @param a
	 *            angle to rotate in degrees
	 * @param x
	 *            X axis component
	 * @param y
	 *            Y axis component
	 * @param z
	 *            Z axis component
	 */
	public static void rotateM(float[] m, int mOffset, float a, float x,
			float y, float z) {
		synchronized (sTemp) {
			setRotateM(sTemp, 0, a, x, y, z);
			multiplyMM(sTemp, 16, sTemp, 0, m, mOffset);
			System.arraycopy(sTemp, 16, m, mOffset, 16);
		}
	}

	/**
	 * Creates a matrix for rotation by angle a (in degrees) around the axis (x,
	 * y, z).
	 * <p>
	 * An optimized path will be used for rotation about a major axis (e.g.
	 * x=1.0f y=0.0f z=0.0f).
	 * 
	 * @param rm
	 *            returns the result
	 * @param rmOffset
	 *            index into rm where the result matrix starts
	 * @param a
	 *            angle to rotate in degrees
	 * @param x
	 *            X axis component
	 * @param y
	 *            Y axis component
	 * @param z
	 *            Z axis component
	 */
	public static void setRotateM(float[] rm, int rmOffset, float a, float x,
			float y, float z) {
		rm[rmOffset + 12] = 0;
		rm[rmOffset + 13] = 0;
		rm[rmOffset + 14] = 0;
		rm[rmOffset + 3] = 0;
		rm[rmOffset + 7] = 0;
		rm[rmOffset + 11] = 0;
		rm[rmOffset + 15] = 1;
		a *= (float) (Math.PI / 180.0f);
		float s = (float) Math.sin(a);
		float c = (float) Math.cos(a);
		if (1.0f == x && 0.0f == y && 0.0f == z) {
			rm[rmOffset] = 1;
			rm[rmOffset + 1] = 0;
			rm[rmOffset + 2] = 0;
			rm[rmOffset + 4] = 0;
			rm[rmOffset + 5] = c;
			rm[rmOffset + 6] = -s;
			rm[rmOffset + 8] = 0;
			rm[rmOffset + 9] = s;
			rm[rmOffset + 10] = c;

		} else if (0.0f == x && 1.0f == y && 0.0f == z) {
			rm[rmOffset] = c;
			rm[rmOffset + 1] = 0;
			rm[rmOffset + 2] = s;
			rm[rmOffset + 4] = 0;
			rm[rmOffset + 5] = 1;
			rm[rmOffset + 6] = 0;
			rm[rmOffset + 8] = -s;
			rm[rmOffset + 9] = 0;
			rm[rmOffset + 10] = c;
		} else if (0.0f == x && 0.0f == y && 1.0f == z) {
			rm[rmOffset] = c;
			rm[rmOffset + 1] = -s;
			rm[rmOffset + 2] = 0;
			rm[rmOffset + 4] = s;
			rm[rmOffset + 5] = c;
			rm[rmOffset + 6] = 0;
			rm[rmOffset + 8] = 0;
			rm[rmOffset + 9] = 0;
			rm[rmOffset + 10] = 1;
		} else {
			float len = length(x, y, z);
			if (1.0f != len) {
				float recipLen = 1.0f / len;
				x *= recipLen;
				y *= recipLen;
				z *= recipLen;
			}
			float nc = 1.0f - c;
			float xy = x * y;
			float yz = y * z;
			float zx = z * x;
			float xs = x * s;
			float ys = y * s;
			float zs = z * s;
			rm[rmOffset] = x * x * nc + c;
			rm[rmOffset + 1] = xy * nc - zs;
			rm[rmOffset + 2] = zx * nc + ys;
			rm[rmOffset + 4] = xy * nc + zs;
			rm[rmOffset + 5] = y * y * nc + c;
			rm[rmOffset + 6] = yz * nc - xs;
			rm[rmOffset + 8] = zx * nc - ys;
			rm[rmOffset + 9] = yz * nc + xs;
			rm[rmOffset + 10] = z * z * nc + c;
		}
	}

	/**
	 * Converts Euler angles to a rotation matrix.
	 * 
	 * @param rm
	 *            returns the result
	 * @param rmOffset
	 *            index into rm where the result matrix starts
	 * @param x
	 *            angle of rotation, in degrees
	 * @param y
	 *            angle of rotation, in degrees
	 * @param z
	 *            angle of rotation, in degrees
	 */
	public static void setRotateEulerM(float[] rm, int rmOffset, float x,
			float y, float z) {
		x *= (float) (Math.PI / 180.0f);
		y *= (float) (Math.PI / 180.0f);
		z *= (float) (Math.PI / 180.0f);
		float cx = (float) Math.cos(x);
		float sx = (float) Math.sin(x);
		float cy = (float) Math.cos(y);
		float sy = (float) Math.sin(y);
		float cz = (float) Math.cos(z);
		float sz = (float) Math.sin(z);
		float cxsy = cx * sy;
		float sxsy = sx * sy;

		rm[rmOffset] = cy * cz;
		rm[rmOffset + 4] = -cy * sz;
		rm[rmOffset + 8] = sy;
		rm[rmOffset + 12] = 0.0f;

		rm[rmOffset + 1] = cxsy * cz + cx * sz;
		rm[rmOffset + 5] = -cxsy * sz + cx * cz;
		rm[rmOffset + 9] = -sx * cy;
		rm[rmOffset + 13] = 0.0f;

		rm[rmOffset + 2] = -sxsy * cz + sx * sz;
		rm[rmOffset + 6] = sxsy * sz + sx * cz;
		rm[rmOffset + 10] = cx * cy;
		rm[rmOffset + 14] = 0.0f;

		rm[rmOffset + 3] = 0.0f;
		rm[rmOffset + 7] = 0.0f;
		rm[rmOffset + 11] = 0.0f;
		rm[rmOffset + 15] = 1.0f;
	}

	/**
	 * Defines a viewing transformation in terms of an eye point, a center of
	 * view, and an up vector.
	 * 
	 * @param rm
	 *            returns the result
	 * @param rmOffset
	 *            index into rm where the result matrix starts
	 * @param eyeX
	 *            eye point X
	 * @param eyeY
	 *            eye point Y
	 * @param eyeZ
	 *            eye point Z
	 * @param centerX
	 *            center of view X
	 * @param centerY
	 *            center of view Y
	 * @param centerZ
	 *            center of view Z
	 * @param upX
	 *            up vector X
	 * @param upY
	 *            up vector Y
	 * @param upZ
	 *            up vector Z
	 */
	public static void setLookAtM(float[] rm, int rmOffset, float eyeX,
			float eyeY, float eyeZ, float centerX, float centerY,
			float centerZ, float upX, float upY, float upZ) {

		// See the OpenGL GLUT documentation for gluLookAt for a description
		// of the algorithm. We implement it in a straightforward way:

		float fx = centerX - eyeX;
		float fy = centerY - eyeY;
		float fz = centerZ - eyeZ;

		// Normalize f
		float rlf = 1.0f / length(fx, fy, fz);
		fx *= rlf;
		fy *= rlf;
		fz *= rlf;

		// compute s = f x up (x means "cross product")
		float sx = fy * upZ - fz * upY;
		float sy = fz * upX - fx * upZ;
		float sz = fx * upY - fy * upX;

		// and normalize s
		float rls = 1.0f / length(sx, sy, sz);
		sx *= rls;
		sy *= rls;
		sz *= rls;

		// compute u = s x f
		float ux = sy * fz - sz * fy;
		float uy = sz * fx - sx * fz;
		float uz = sx * fy - sy * fx;

		rm[rmOffset] = sx;
		rm[rmOffset + 1] = sy;
		rm[rmOffset + 2] = sz;
		rm[rmOffset + 3] = 0.0f;
		rm[rmOffset + 4] = ux;
		rm[rmOffset + 5] = uy;
		rm[rmOffset + 6] = uz;
		rm[rmOffset + 7] = 0.0f;
		rm[rmOffset + 8] = -fx;
		rm[rmOffset + 9] = -fy;
		rm[rmOffset + 10] = -fz;
		rm[rmOffset + 11] = 0.0f;
		rm[rmOffset + 12] = 0.0f;
		rm[rmOffset + 13] = 0.0f;
		rm[rmOffset + 14] = 0.0f;
		rm[rmOffset + 15] = 1.0f;

		translateM(rm, rmOffset, -eyeX, -eyeY, -eyeZ);
	}

}
