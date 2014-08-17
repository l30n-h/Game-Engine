package com.brocorporation.gameengine.utils;

public class Quaternion extends Vector4f {

	public final static float PIOVER180 = (float) Math.PI / 180;
	public final static float PIOVER360 = (float) Math.PI / 360;

	public Quaternion() {
		super(0, 0, 0, 1);
	}

	public Quaternion set(final float pX, final float pY, final float pZ,
			final float pW) {
		x = pX;
		y = pY;
		z = pZ;
		w = pW;
		return this;
	}

	public Quaternion set(final Quaternion quaternion) {
		x = quaternion.x;
		y = quaternion.y;
		z = quaternion.z;
		w = quaternion.w;
		return this;
	}

	public Quaternion konjugateQ(final float pX, final float pY,
			final float pZ, final float pW) {
		return set(-pX, -pY, -pZ, pW);
	}

	public Quaternion konjugateQ(final Quaternion quaternion) {
		return konjugateQ(quaternion.x, quaternion.y, quaternion.z,
				quaternion.w);
	}

	public Quaternion multiply(final float pX, final float pY, final float pZ,
			final float pW) {
		final float t5 = (z + x) * (pX + pY);
		final float t6 = (w + y) * (pW - pZ);
		final float t7 = (w - y) * (pW + pZ);
		final float t8 = t5 + t6 + t7;
		final float t9 = ((z - x) * (pX - pY) + t8) * 0.5f;
		return set((w + x) * (pW + pX) + t9 - t8,
				(w - x) * (pY + pZ) + t9 - t7, (z + y) * (pW - pX) + t9 - t6,
				(z - y) * (pY - pZ) + t9 - t5);
	}

	public Quaternion multiply(final Quaternion quaternion) {
		return multiply(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
	}

	public Quaternion addEulerRotation(final float pX, final float pY,
			final float pZ) {
		return addRotationScaled(pX, pY, pZ, PIOVER180);
	}

	public Quaternion addEulerRotation(final Vector3f vector) {
		return addEulerRotation(vector.x, vector.y, vector.z);
	}

	public Quaternion addEulerRotationScaled(final float pX, final float pY,
			final float pZ, final float scalar) {
		return addRotationScaled(pX, pY, pZ, scalar * PIOVER360);
	}

	public Quaternion addEulerRotationScaled(final Vector3f vector,
			final float scalar) {
		return addRotationScaled(vector.x, vector.y, vector.z, scalar);
	}

	public Quaternion addRotation(final float wx, final float wy, final float wz) {
		final float sq = wx * wx + wy * wy + wz * wz;
		float qw;
		float s;
		if (sq < 0.6f) {
			qw = 1 - sq * 0.125f;
			s = 0.5f - sq * 0.02083333333f;
		} else {
			final float thetaMag = (float) Math.sqrt(sq);
			final float halfthetaMag = thetaMag * 0.5f;
			qw = (float) Math.cos(halfthetaMag);
			s = (float) Math.sin(halfthetaMag) / thetaMag;
		}
		float qx = wx * s;
		float qy = wy * s;
		float qz = wz * s;
		return multiply(qx, qy, qz, qw);
	}

	public Quaternion addRotation(final Vector3f vector) {
		return addRotation(vector.x, vector.y, vector.z);
	}

	public Quaternion addRotationScaled(final Vector3f vector,
			final float scalar) {
		return addRotationScaled(vector.x, vector.y, vector.z, scalar);
	}

	public Quaternion addRotationScaled(final float wx, final float wy,
			final float wz, final float scalar) {
		return addRotation(wx * scalar, wy * scalar, wz * scalar);
	}

	public Vector3f rotateV(final Vector3f vector) {
		return rotateV(vector, vector);
	}

	public Vector3f rotateV(final Vector3f result, final Vector3f vector) {
		final float tx = y * vector.z - z * vector.y;
		final float ty = z * vector.x - x * vector.z;
		final float tz = x * vector.y - y * vector.x;
		return result.setAdd(vector.x, vector.y, vector.z, (w * tx + y * tz - z
				* ty) * 2, (w * ty + z * tx - x * tz) * 2, (w * tz + x * ty - y
				* tx) * 2);
	}

	public Vector3f rotateHalfsize(final Vector3f result, final Vector3f vector) {
		final float vx2 = vector.x * 2;
		final float vy2 = vector.y * 2;
		final float vz2 = vector.z * 2;
		final float t1y = z * vx2;
		final float t1z = y * vx2;
		final float t2x = z * vy2;
		final float t2z = x * vy2;
		final float t3x = y * vz2;
		final float t3y = x * vz2;
		final float rX = Math.abs(z * t1y + y * t1z - vector.x)
				+ Math.abs(y * t2z - w * t2x) + Math.abs(w * t3x + z * t3y);
		final float rY = Math.abs(w * t1y + x * t1z)
				+ Math.abs(x * t2z + z * t2x - vector.y)
				+ Math.abs(z * t3x - w * t3y);
		final float rZ = Math.abs(x * t1y - w * t1z)
				+ Math.abs(w * t2z + y * t2x)
				+ Math.abs(y * t3x + x * t3y - vector.z);
		return result.set(rX, rY, rZ);
	}

	public Vector3f rotateHalfsize(final Vector3f vector) {
		return rotateHalfsize(vector, vector);
	}

	public Quaternion getQuaternion(final Vector3f vector1,
			final Vector3f vector2) {
		w = (float) Math.sqrt(vector1.dot(vector1) * vector2.dot(vector2))
				+ vector1.dot(vector2);
		((Quaternion) setCross(vector1, vector2)).norm();
		return this;
	}

	public Quaternion getQuaternion(final Vector3f vector, float rad) {
		rad *= 0.5f;
		set(vector).norm().scale((float) Math.sin(rad));
		w = (float) Math.cos(rad);
		return this;
	}

	public Quaternion getQuaternionEuler(final float degX, final float degY,
			final float degZ) {
		return getQuaternion(degX * PIOVER360, degY * PIOVER360, degZ
				* PIOVER360);
	}

	public Quaternion getQuaternionEuler(Vector3f rotation) {
		return getQuaternion(rotation.x * PIOVER360, rotation.y * PIOVER360,
				rotation.z * PIOVER360);
	}

	public Quaternion getQuaternion(Vector3f rotation) {
		return getQuaternion(rotation.x, rotation.y, rotation.z);
	}

	public Quaternion getQuaternion(final float x, final float y, final float z) {
		final float siny = (float) Math.sin(y);
		final float sinz = (float) Math.sin(z);
		final float sinx = (float) Math.sin(x);
		final float cosy = (float) Math.cos(y);
		final float cosz = (float) Math.cos(z);
		final float cosx = (float) Math.cos(x);
		final float sinxcosy = sinx * cosy;
		final float sinxsiny = sinx * siny;
		final float cosxcosy = cosx * cosy;
		final float cosxsiny = cosx * siny;
		set(sinxcosy * cosz - cosxsiny * sinz,
				cosxsiny * cosz + sinxcosy * sinz,
				cosxcosy * sinz - sinxsiny * cosz,
				cosxcosy * cosz + sinxsiny * sinz).norm();
		return this;
	}

	public void multiplyQM(float[] result, int resultOffset, float[] rhs,
			int rhsOffset) {
		final float x2 = x * x;
		final float y2 = y * y;
		final float z2 = z * z;
		final float xy = x * y;
		final float xz = x * z;
		final float yz = y * z;
		final float wx = w * x;
		final float wy = w * y;
		final float wz = w * z;

		final float l0 = 1 - 2 * (y2 + z2);
		final float l1 = 2 * (xy + wz);
		final float l2 = 2 * (xz - wy);
		final float l4 = 2 * (xy - wz);
		final float l5 = 1 - 2 * (x2 + z2);
		final float l6 = 2 * (yz + wx);
		final float l8 = 2 * (xz + wy);
		final float l9 = 2 * (yz - wx);
		final float l10 = 1 - 2 * (x2 + y2);
		final float r0 = rhs[rhsOffset];
		final float r1 = rhs[rhsOffset + 1];
		final float r2 = rhs[rhsOffset + 2];
		final float r4 = rhs[rhsOffset + 4];
		final float r5 = rhs[rhsOffset + 5];
		final float r6 = rhs[rhsOffset + 6];
		final float r8 = rhs[rhsOffset + 8];
		final float r9 = rhs[rhsOffset + 9];
		final float r10 = rhs[rhsOffset + 10];
		final float r12 = rhs[rhsOffset + 12];
		final float r13 = rhs[rhsOffset + 13];
		final float r14 = rhs[rhsOffset + 14];
		result[resultOffset] = r0 * l0 + r1 * l4 + r2 * l8;
		result[resultOffset + 1] = r0 * l1 + r1 * l5 + r2 * l9;
		result[resultOffset + 2] = r0 * l2 + r1 * l6 + r2 * l10;
		result[resultOffset + 3] = rhs[rhsOffset + 3];
		result[resultOffset + 4] = r4 * l0 + r5 * l4 + r6 * l8;
		result[resultOffset + 5] = r4 * l1 + r5 * l5 + r6 * l9;
		result[resultOffset + 6] = r4 * l2 + r5 * l6 + r6 * l10;
		result[resultOffset + 7] = rhs[rhsOffset + 7];
		result[resultOffset + 8] = r8 * l0 + r9 * l4 + r10 * l8;
		result[resultOffset + 9] = r8 * l1 + r9 * l5 + r10 * l9;
		result[resultOffset + 10] = r8 * l2 + r9 * l6 + r10 * l10;
		result[resultOffset + 11] = rhs[rhsOffset + 11];
		result[resultOffset + 12] = r12 * l0 + r13 * l4 + r14 * l8;
		result[resultOffset + 13] = r12 * l1 + r13 * l5 + r14 * l9;
		result[resultOffset + 14] = r12 * l2 + r13 * l6 + r14 * l10;
		result[resultOffset + 15] = rhs[rhsOffset + 15];
	}

	public void multiplyMQ(float[] result, int resultOffset, float[] lhs,
			int lhsOffset) {
		final float x2 = x * x;
		final float y2 = y * y;
		final float z2 = z * z;
		final float xy = x * y;
		final float xz = x * z;
		final float yz = y * z;
		final float wx = w * x;
		final float wy = w * y;
		final float wz = w * z;

		final float r0 = 1 - 2 * (y2 + z2);
		final float r1 = 2 * (xy + wz);
		final float r2 = 2 * (xz - wy);
		final float r4 = 2 * (xy - wz);
		final float r5 = 1 - 2 * (x2 + z2);
		final float r6 = 2 * (yz + wx);
		final float r8 = 2 * (xz + wy);
		final float r9 = 2 * (yz - wx);
		final float r10 = 1 - 2 * (x2 + y2);
		final float l0 = lhs[lhsOffset];
		final float l1 = lhs[lhsOffset + 1];
		final float l2 = lhs[lhsOffset + 2];
		final float l3 = lhs[lhsOffset + 3];
		final float l4 = lhs[lhsOffset + 4];
		final float l5 = lhs[lhsOffset + 5];
		final float l6 = lhs[lhsOffset + 6];
		final float l7 = lhs[lhsOffset + 7];
		final float l8 = lhs[lhsOffset + 8];
		final float l9 = lhs[lhsOffset + 9];
		final float l10 = lhs[lhsOffset + 10];
		final float l11 = lhs[lhsOffset + 11];
		result[resultOffset] = r0 * l0 + r1 * l4 + r2 * l8;
		result[resultOffset + 1] = r0 * l1 + r1 * l5 + r2 * l9;
		result[resultOffset + 2] = r0 * l2 + r1 * l6 + r2 * l10;
		result[resultOffset + 3] = r0 * l3 + r1 * l7 + r2 * l11;
		result[resultOffset + 4] = r4 * l0 + r5 * l4 + r6 * l8;
		result[resultOffset + 5] = r4 * l1 + r5 * l5 + r6 * l9;
		result[resultOffset + 6] = r4 * l2 + r5 * l6 + r6 * l10;
		result[resultOffset + 7] = r4 * l3 + r5 * l7 + r6 * l11;
		result[resultOffset + 8] = r8 * l0 + r9 * l4 + r10 * l8;
		result[resultOffset + 9] = r8 * l1 + r9 * l5 + r10 * l9;
		result[resultOffset + 10] = r8 * l2 + r9 * l6 + r10 * l10;
		result[resultOffset + 11] = r8 * l3 + r9 * l7 + r10 * l11;
		result[resultOffset + 12] = lhs[lhsOffset + 12];
		result[resultOffset + 13] = lhs[lhsOffset + 13];
		result[resultOffset + 14] = lhs[lhsOffset + 14];
		result[resultOffset + 15] = lhs[lhsOffset + 15];
	}

	public float[] getRotationMatrix(final float[] result) {
		final float x2 = x * x;
		final float y2 = y * y;
		final float z2 = z * z;
		final float xy = x * y;
		final float xz = x * z;
		final float yz = y * z;
		final float wx = w * x;
		final float wy = w * y;
		final float wz = w * z;

		result[0] = 1 - 2 * (y2 + z2);
		result[1] = 2 * (xy + wz);
		result[2] = 2 * (xz - wy);
		result[3] = 0;
		result[4] = 2 * (xy - wz);
		result[5] = 1 - 2 * (x2 + z2);
		result[6] = 2 * (yz + wx);
		result[7] = 0;
		result[8] = 2 * (xz + wy);
		result[9] = 2 * (yz - wx);
		result[10] = 1 - 2 * (x2 + y2);
		result[11] = 0;
		result[12] = 0;
		result[13] = 0;
		result[14] = 0;
		result[15] = 1;
		return result;
	}

	public Quaternion lookAt(final float pFromX, final float pFromY,
			final float pFromZ, final float pToX, final float pToY,
			final float pToZ) {
		setSubtract(pToX, pToY, pToZ, pFromX, pFromY, pFromZ);
		return lookAt(x, y, z, length());
	}

	public Quaternion lookAt(final float[] quaternion, final float pDirX,
			final float pDirY, final float pDirZ) {
		set(pDirX, pDirY, pDirZ);
		return lookAt(pDirX, pDirY, pDirZ, length());
	}

	public Quaternion lookAt(final float pDirX, final float pDirY,
			final float pDirZ, final float length) {
		set(-pDirY, pDirX, 0, length + pDirZ).norm();
		return this;
	}

	public static void getHalfRadians(final Vector3f vector, final float degX,
			final float degY, final float degZ) {
		vector.set(degX * PIOVER360, degY * PIOVER360, degZ * PIOVER360);
	}
}
