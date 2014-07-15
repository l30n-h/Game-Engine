package com.brocorporation.gameengine.elements.opengl;

import com.brocorporation.gameengine.utils.Vector3f;
import com.brocorporation.gameengine.utils.Vector4f;

public class BlinnPhongLight extends GLLight {

	private final Vector3f ambientColor = new Vector3f(0, 0, 0);
	private final Vector3f diffuseColor = new Vector3f(1, 1, 1);
	private final Vector3f specularColor = new Vector3f(0, 0, 0);
	private final Vector4f position = new Vector4f(0, 0, 1, 1);
	private final Vector3f spotDirection = new Vector3f(0, -1, 0);
	private float spotExponent = 0;
	private float spotCosCutoff = (float) Math.PI;
	private float constantAttenuation = 1;
	private float linearAttenuation = 0;
	private float quadraticAttenuation = 0;

	private final Vector3f globalAmbient = new Vector3f(0.2F, 0.2F, 0.2F);

	protected final BlinnPhongShader shader;

	public BlinnPhongLight(final BlinnPhongShader pShader) {
		shader = pShader;
	}

	public void setAmientColor(final float r, final float g, final float b) {
		ambientColor.set(r, g, b);
	}

	public void setDiffuseColor(final float r, final float g, final float b) {
		diffuseColor.set(r, g, b);
	}

	public void setSpecularColor(final float r, final float g, final float b) {
		specularColor.set(r, g, b);
	}

	public void setPosition(final float x, final float y, final float z,
			final float w) {
		position.set(x, y, z, w);
	}

	public void setSpotDirection(final float x, final float y, final float z) {
		spotDirection.set(x, y, z).norm();
	}

	public void setSpotExponent(final float pSpotExponent) {
		spotExponent = pSpotExponent;
	}

	public void setSpotCosCutoff(final float pSpotCosCutoff) {
		spotCosCutoff = pSpotCosCutoff;
	}

	public void setConstantAttenuation(final float pConstantAttenuation) {
		constantAttenuation = pConstantAttenuation;
	}

	public void setLinearAttenuation(final float pLinearAttenuation) {
		linearAttenuation = pLinearAttenuation;
	}

	public void setQuadraticAttenuation(final float pQuadraticAttenuation) {
		quadraticAttenuation = pQuadraticAttenuation;
	}

	public void setGlobalAmientColor(final float r, final float g, final float b) {
		globalAmbient.set(r, g, b);
	}

	@Override
	public void initLight() {
		shader.setGlobalAmbient(globalAmbient);
		shader.setLightAmbient(ambientColor);
		shader.setLightDiffuse(diffuseColor);
		shader.setLightSpecular(specularColor);
		shader.setLightPosition(position);
		shader.setLightSpotDirection(spotDirection);
		shader.setLightSpotExponent(spotExponent);
		shader.setLightSpotCosCutoff(spotCosCutoff);
		shader.setLightConstantAttenuation(constantAttenuation);
		shader.setLightLinearAttenuation(linearAttenuation);
		shader.setLightQuadraticAttenuation(quadraticAttenuation);
	}

	@Override
	public void render() {
		shader.setLightPosition(position);
	}
}
