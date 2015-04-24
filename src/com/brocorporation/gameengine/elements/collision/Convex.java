package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.utils.AffineTransform;
import com.brocorporation.gameengine.utils.Quaternion;
import com.brocorporation.gameengine.utils.Vector3f;

public class Convex implements IShape {

	protected final AABB aabb = new AABB();
	protected final Vector3f[] baseVertices;
	protected final Vector3f[] vertices;
	protected final Vector3f position = new Vector3f();
	protected final Vector3f lastTranslation = new Vector3f();
	protected boolean hasChanged;
	protected byte frustumType;

	public Convex(final Vector3f[] pVertices) {
		baseVertices = pVertices;
		final int size = baseVertices.length;
		vertices = new Vector3f[size];
		for (int i = 0; i < size; i++) {
			vertices[i] = new Vector3f(baseVertices[i]);
		}
		AABB.getBounds(aabb, vertices);
		getCOM(position);
	}

	public Vector3f[] getVertices() {
		return vertices;
	}

	public void getCOM(Vector3f result) {
		result.set(vertices[0]);
		final int size = vertices.length;
		for (int i = 1; i < size; i++) {
			result.add(vertices[i]);
		}
		result.scale(1F / size);
	}

	@Override
	public Vector3f getPosition() {
		return position;
	}

	@Override
	public Vector3f getMaxAlongDirection(Vector3f result, Vector3f dir) {
		Vector3f max = vertices[0];
		float dotmaxdir = max.dot(dir);
		for (int i = vertices.length - 1; i >= 0; i--) {
			final float dotshapedir = vertices[i].dot(dir);
			if (dotshapedir > dotmaxdir) {
				max = vertices[i];
				dotmaxdir = dotshapedir;
			}
		}
		return result.set(max);
	}

	@Override
	public Vector3f getMinAlongDirection(Vector3f result, Vector3f dir) {
		Vector3f min = vertices[0];
		float dotmindir = min.dot(dir);
		for (int i = vertices.length - 1; i >= 0; i--) {
			final float dotshapedir = vertices[i].dot(dir);
			if (dotshapedir < dotmindir) {
				min = vertices[i];
				dotmindir = dotshapedir;
			}
		}
		return result.set(min);
	}

	@Override
	public AABB getAABB() {
		return aabb;
	}

	@Override
	public void updateBounds(AffineTransform transform,
			boolean updateTranslation, boolean updateOrientation) {
		hasChanged = updateTranslation || updateOrientation;
		if (hasChanged) {
			if (updateOrientation) {
				final Quaternion orientation = transform.getOrientation();
				final Vector3f translation = transform.getTranslation();
				for (int i = vertices.length - 1; i >= 0; i--) {
					orientation.rotateV(vertices[i], baseVertices[i]);
					vertices[i].add(translation);
				}
				position.set(translation);
				lastTranslation.set(translation);
			} else if (updateTranslation) {
				final Vector3f translation = transform.getTranslation();
				lastTranslation.subtract(translation);
				for (int i = vertices.length - 1; i >= 0; i--) {
					vertices[i].subtract(lastTranslation);
				}
				position.set(translation);
				lastTranslation.set(translation);
			}
		}
		aabb.updateBounds(transform, updateTranslation, updateOrientation);
	}

	@Override
	public void setFrustumType(byte type) {
		frustumType = type;
	}

	@Override
	public byte getFrustumType() {
		return frustumType;
	}

	@Override
	public boolean hasChanged() {
		return hasChanged;
	}

	@Override
	public void getInverseInertiaTensor(float[] inverseInertiaTensor,
			float inverseMass) {
		aabb.getInverseInertiaTensor(inverseInertiaTensor, inverseMass);
	}

	public static void getCOM(Vector3f result, Vector3f[] points) {
		result.set(points[0]);
		final int size = points.length;
		for (int i = 1; i < size; i++) {
			result.add(points[i]);
		}
		result.scale(1F / size);
	}

	public interface IBounds {

		public Convex getConvex();
	}
}
