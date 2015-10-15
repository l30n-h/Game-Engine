package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.utils.MatrixExt;
import com.brocorporation.gameengine.utils.Vector3f;
import com.brocorporation.gameengine.utils.Vector4f;

public class Frustum {

	public final static byte NONE = 0;
	public final static byte CONTAINS = 1;
	public final static byte INTERSECTS = 2;

	protected final static byte FRONT = 0;
	protected final static byte BACK = 1;
	protected final static byte RIGHT = 2;
	protected final static byte LEFT = 3;
	protected final static byte BOTTOM = 4;
	protected final static byte TOP = 5;

	protected final Vector4f[] planes = { new Vector4f(), new Vector4f(),
			new Vector4f(), new Vector4f(), new Vector4f(), new Vector4f() };
	protected float m12, m13, m14;
	protected boolean hasChanged;
	protected final static Vector3f temp = new Vector3f();
	protected float[] lastFrustum = new float[16];

	public Frustum() {
	}

	public void setFrustum(final float[] frustumM) {
		// hasChanged = m12 != frustumM[12] || m13 != frustumM[13]
		// || m14 != frustumM[14];
		hasChanged = !MatrixExt.equalsM(lastFrustum, frustumM);// TODO besser
																// schon von
																// Camera
																// überprüfen
		if (hasChanged) {
			MatrixExt.setM(lastFrustum, frustumM);

			planes[FRONT].x = frustumM[12] + frustumM[8];
			planes[FRONT].y = frustumM[13] + frustumM[9];
			planes[FRONT].z = frustumM[14] + frustumM[10];
			planes[FRONT].w = -frustumM[15] - frustumM[11];

			planes[BACK].x = frustumM[12] - frustumM[8];
			planes[BACK].y = frustumM[13] - frustumM[9];
			planes[BACK].z = frustumM[14] - frustumM[10];
			planes[BACK].w = frustumM[11] - frustumM[15];

			planes[RIGHT].x = frustumM[12] - frustumM[0];
			planes[RIGHT].y = frustumM[13] - frustumM[1];
			planes[RIGHT].z = frustumM[14] - frustumM[2];
			planes[RIGHT].w = frustumM[3] - frustumM[15];

			planes[LEFT].x = frustumM[12] + frustumM[0];
			planes[LEFT].y = frustumM[13] + frustumM[1];
			planes[LEFT].z = frustumM[14] + frustumM[2];
			planes[LEFT].w = -frustumM[15] - frustumM[3];

			planes[TOP].x = frustumM[12] - frustumM[4];
			planes[TOP].y = frustumM[13] - frustumM[5];
			planes[TOP].z = frustumM[14] - frustumM[6];
			planes[TOP].w = frustumM[7] - frustumM[15];

			planes[BOTTOM].x = frustumM[12] + frustumM[4];
			planes[BOTTOM].y = frustumM[13] + frustumM[5];
			planes[BOTTOM].z = frustumM[14] + frustumM[6];
			planes[BOTTOM].w = -frustumM[15] - frustumM[7];

			normPlane(planes[FRONT]);
			normPlane(planes[BACK]);
			normPlane(planes[RIGHT]);
			normPlane(planes[LEFT]);
			normPlane(planes[TOP]);
			normPlane(planes[BOTTOM]);
		}
	}

	public boolean hasChanged() {
		return hasChanged;
	}

	private static void normPlane(Vector4f plane) {
		final Vector3f v = plane;
		final float length2 = v.dot();
		plane.scale(1F / (float) Math.sqrt(length2));
	}

	public boolean intersects(final Vector3f vertex) {
		for (final Vector4f p : planes) {
			if (p.x * vertex.x + p.y * vertex.y + p.z * vertex.z < p.w) {
				return false;
			}
		}
		return true;
	}

	public boolean intersects(final Vector3f vertex, final float radius) {
		for (final Vector4f p : planes) {
			if (p.x * vertex.x + p.y * vertex.y + p.z * vertex.z < p.w - radius) {
				return false;
			}
		}
		return true;
	}

	public byte containsIntersects(final IShape shape) {
		if (hasChanged || shape.hasChanged()) {
			Vector4f p;
			for (int i = 0; i < 6; i++) {
				p = planes[i];
				shape.getMinAlongDirection(temp, p);
				if (p.x * temp.x + p.y * temp.y + p.z * temp.z < p.w) {
					for (; i < 6; i++) {
						p = planes[i];
						shape.getMaxAlongDirection(temp, p);
						if (p.x * temp.x + p.y * temp.y + p.z * temp.z < p.w) {
							shape.setFrustumType(NONE);
							return NONE;
						}
					}
					shape.setFrustumType(INTERSECTS);
					return INTERSECTS;
				}
			}
			shape.setFrustumType(CONTAINS);
			return CONTAINS;
		}
		return shape.getFrustumType();
	}

	public boolean intersects(final IShape shape) {
		if (hasChanged || shape.hasChanged()) {
			for (final Vector4f p : planes) {
				shape.getMaxAlongDirection(temp, p);
				if (p.x * temp.x + p.y * temp.y + p.z * temp.z < p.w) {
					shape.setFrustumType(NONE);
					return false;
				}
			}
			shape.setFrustumType(INTERSECTS);
			return true;
		}
		return shape.getFrustumType() != NONE;
	}

	public boolean contains(final IShape shape) {
		if (hasChanged || shape.hasChanged()) {
			for (final Vector4f p : planes) {
				shape.getMinAlongDirection(temp, p);
				if (p.x * temp.x + p.y * temp.y + p.z * temp.z < p.w) {
					shape.setFrustumType(NONE);
					return false;
				}
			}
			shape.setFrustumType(CONTAINS);
			return true;
		}
		return shape.getFrustumType() == CONTAINS;
	}
}
