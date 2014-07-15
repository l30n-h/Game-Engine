package com.brocorporation.gameengine.elements.collision;

public class Material {

	public static final Material DEFAULT = new Material();

	protected float staticFriction;
	protected float dynamicFriction;
	protected float restitution;

	public Material() {

	}

	public Material(float pRestitution) {
		restitution = pRestitution;
	}

	public Material(float pStaticFriction, float pDynamicFriction) {
		staticFriction = pStaticFriction;
		dynamicFriction = pDynamicFriction;
	}

	public Material(float pRestitution, float pStaticFriction,
			float pDynamicFriction) {
		restitution = pRestitution;
		staticFriction = pStaticFriction;
		dynamicFriction = pDynamicFriction;
	}

	public void setStaticFriction(float sF) {
		staticFriction = sF;
	}

	public void setDynamicFriction(float dF) {
		dynamicFriction = dF;
	}

	public void setRestitution(float r) {
		restitution = r;
	}

	public float getStaticFriction() {
		return staticFriction;
	}

	public float getDynamicFriction() {
		return dynamicFriction;
	}

	public float getRestitution() {
		return restitution;
	}
}
