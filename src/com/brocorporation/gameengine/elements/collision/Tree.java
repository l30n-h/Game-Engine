package com.brocorporation.gameengine.elements.collision;

import java.util.List;

import com.brocorporation.gameengine.utils.Vector3f;

public abstract class Tree {

	public abstract void add(final AABB pBounds, final Object o);

	public abstract void remove(AABB pBounds);

	public abstract boolean stillInside(AABB pBounds);

	public abstract void hasMoved(AABB pBounds);
	
	public abstract void optimize();

	public abstract List<Object> retrieve(List<Object> returnObjects,
			Frustum frustum);

	public abstract List<Object> retrieve(List<Object> returnObjects,
			Vector3f pFrom, Vector3f pDir, float min, float max);

	public abstract List<Object> retrieve(List<Object> returnObjects,
			Vector3f pFrom, Vector3f pDir, Vector3f halfsizeOffset, float min,
			float max);

	public abstract List<Object> retrieve(List<Object> returnObjects,
			final Vector3f pPosition, final Vector3f pHalfsize);

	public abstract List<Object> retrieve(List<Object> returnObjects,
			AABB pBounds);

	public abstract List<Object> retrieveAll(List<Object> returnObjects);
}
