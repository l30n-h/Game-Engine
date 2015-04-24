package com.brocorporation.gameengine.elements.bodies;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.collision.AABB;
import com.brocorporation.gameengine.utils.Matrix;
import com.brocorporation.gameengine.utils.MatrixExt;
import com.brocorporation.gameengine.utils.Quaternion;
import com.brocorporation.gameengine.utils.Vector3f;

public abstract class Camera implements AABB.IBounds {

	protected final static float[] viewMatrix = new float[16];
	protected final static float[] projectionMatrix = new float[16];

	protected final Vector3f position = new Vector3f();
	protected final AABB aabb = new AABB();
	protected final Quaternion orientation = new Quaternion();
	protected final Vector3f viewDirection = new Vector3f();
	protected final Vector3f normedViewDirection = new Vector3f();
	protected final Vector3f up = new Vector3f();
	protected final Vector3f side = new Vector3f();

	protected Vector3f lookAtPosition = null;
	protected final Vector3f lookAtOffset = new Vector3f(0, 0, 1);

	protected float left, right, bottom, top, near, far, zoom = 1;

	protected boolean updateFrustumMatrix = false;

	public Camera() {
		lookAtPosition = position;
	}

	public void setPosition(final float x, final float y, final float z) {
		position.set(x, y, z);
		aabb.setPosition(position);
	}

	public void setFrustum(final float pLeft, final float pRight,
			final float pBottom, final float pTop, final float pNear,
			final float pFar) {
		near = pNear;
		far = pFar;
		left = pLeft * near;
		right = pRight * near;
		bottom = pBottom * near;
		top = pTop * near;
		updateFrustumMatrix = true;
	}

	public void setZoom(final float pZoom) {
		final float scale = pZoom / zoom;
		left *= scale;
		right *= scale;
		top *= scale;
		bottom *= scale;
		zoom = pZoom;
		updateFrustumMatrix = true;
	}

	public float getLeft() {
		return left;
	}

	public float getRight() {
		return right;
	}

	public float getTop() {
		return top;
	}

	public float getBottom() {
		return bottom;
	}

	public float getNear() {
		return near;
	}

	public float getFar() {
		return far;
	}

	public Vector3f getUp() {
		return up;
	}

	public Vector3f getSide() {
		return side;
	}

	public Vector3f getViewDirection() {
		return viewDirection;
	}

	public Vector3f getNormedViewDirection() {
		return normedViewDirection;
	}

	public void setViewDirection(float dirX, float dirY, float dirZ) {
		viewDirection.set(dirX, dirY, dirZ);
		side.setCross(viewDirection, Vector3f.UP);
		up.setCross(side, viewDirection);
		up.norm();
		side.norm();
		normedViewDirection.setNorm(viewDirection);
	}

	public Quaternion getOrientation() {
		return orientation;
	}

	public void lookAtBody(final Vector3f pLookAtPosition, final float offsetX,
			final float offsetY, final float offsetZ) {
		if (pLookAtPosition != null) {
			lookAtPosition = pLookAtPosition;
		}
		lookAtOffset.set(offsetX, offsetY, offsetZ);
	}

	protected final static Vector3f temp = new Vector3f();

	public void prepareUpdatePosition(final IUpdateInfo uInfo) {
		if (lookAtPosition != null) {
			temp.setAdd(lookAtPosition, lookAtOffset).subtract(position);
			if (!temp.equals(viewDirection)) {
				viewDirection.set(temp);
				side.setCross(viewDirection, Vector3f.UP);
				up.setCross(side, viewDirection);
				up.norm();
				side.norm();
				normedViewDirection.setNorm(viewDirection);
			}
		}
	}

	public abstract void updatePosition(final IUpdateInfo uInfo);

	public float[] getViewMatrix() {
		MatrixExt.setLookAtDirM(viewMatrix, 0, position, normedViewDirection,
				side, up);
		return viewMatrix;
	}

	public float[] getProjectionMatrix() {
		if (updateFrustumMatrix) {
			Matrix.frustumM(projectionMatrix, 0, left, right, bottom, top,
					near, far);
			updateFrustumMatrix = false;
		}
		return projectionMatrix;
	}

	public Vector3f getPosition() {
		return position;
	}

	@Override
	public AABB getAABB() {
		return aabb;
	}
}
