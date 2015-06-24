package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.utils.AffineTransform;
import com.brocorporation.gameengine.utils.Vector3f;

public interface IShape {

	public Vector3f getPosition();

	public Vector3f getMaxAlongDirection(Vector3f result, Vector3f dir);

	public Vector3f getMinAlongDirection(Vector3f result, Vector3f dir);

	public Vector3f[] getAllMaxAlongDirection(Vector3f[] result, Vector3f dir,
			int count, float eps);

	public Vector3f[] getAllMinAlongDirection(Vector3f[] result, Vector3f dir,
			int count, float eps);

	public AABB getAABB();

	public void setFrustumType(byte type);

	public byte getFrustumType();

	public boolean hasChanged();

	public void updateBounds(AffineTransform transform,
			boolean updateTranslation, boolean updateOrientation);

	public void getInverseInertiaTensor(Vector3f diagInverseInertiaTensor,
			float inverseMass);
}
