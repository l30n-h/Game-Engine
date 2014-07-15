package com.brocorporation.gameengine.elements.opengl;

import com.brocorporation.gameengine.elements.collision.AABB;

public class GLMesh implements AABB.IBounds {

	protected final AABB aabb = new AABB();

	protected GLMaterial material;
	protected boolean hasUVs = false;
	protected int indiceslength;

	public void setMaterial(final GLMaterial pMaterial) {
		material = pMaterial;
	}

	public void hasUVs(final boolean pHasUVs) {
		hasUVs = pHasUVs;
	}

	public void addIndex() {
		indiceslength++;
	}

	public GLMaterial getMaterial() {
		return material;
	}

	public boolean hasUVs() {
		return hasUVs;
	}

	public int getIndicesLength() {
		return indiceslength;
	}

	@Override
	public AABB getAABB() {
		return aabb;
	}
}
