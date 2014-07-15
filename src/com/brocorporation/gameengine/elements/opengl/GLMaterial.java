package com.brocorporation.gameengine.elements.opengl;

import com.brocorporation.gameengine.utils.Vector3f;

public class GLMaterial {

	private float shininess = 0;
	private final Vector3f ambientColor = new Vector3f(0.2F, 0.2F, 0.2F);
	private final Vector3f diffuseColor = new Vector3f(0.8F, 0.8F, 0.8F);
	private final Vector3f specularColor = new Vector3f(0, 0, 0);
	private float refractionIndex;
	private float dissolveFaktor;
	private short illuminance;
	private String texture;

	public void setShininess(final float pShininess) {
		shininess = pShininess;
	}

	public void setAmientColor(final float r, final float g, final float b) {
		ambientColor.x = r;
		ambientColor.y = g;
		ambientColor.z = b;
	}

	public void setDiffuseColor(final float r, final float g, final float b) {
		diffuseColor.x = r;
		diffuseColor.y = g;
		diffuseColor.z = b;
	}

	public void setSpecularColor(final float r, final float g, final float b) {
		specularColor.x = r;
		specularColor.y = g;
		specularColor.z = b;
	}

	public void setRefractionIndex(final float pRefractionIndex) {
		refractionIndex = pRefractionIndex;
	}

	public void setDissolveFaktor(final float pDissolveFaktor) {
		dissolveFaktor = pDissolveFaktor;
	}

	public void setIlluminance(final short pIlluminance) {
		illuminance = pIlluminance;
	}

	public void setTexture(final String pTexture) {
		texture = pTexture;
	}

	public float getShininess() {
		return shininess;
	}

	public Vector3f getAmbientColor() {
		return ambientColor;
	}

	public Vector3f getDiffuseColor() {
		return diffuseColor;
	}

	public Vector3f getSpecularColor() {
		return specularColor;
	}

	public float getRefractionIndex() {
		return refractionIndex;
	}

	public float getDissolveFaktor() {
		return dissolveFaktor;
	}

	public short getIlluminance() {
		return illuminance;
	}

	public String getTexture() {
		return texture;
	}
}
