package com.brocorporation.gameengine.elements.opengl;

import org.lwjgl.opengl.GL13;
import org.lwjgl.opengl.GL20;

import com.brocorporation.gameengine.utils.MatrixExt;
import com.brocorporation.gameengine.utils.Vector3f;
import com.brocorporation.gameengine.utils.Vector4f;

public class BlinnPhongShader extends GLShader {

	public final static String[] attributes = { "a_Position", "a_Normal",
			"a_UV" };
	protected final static String[] uniforms = { "u_MVPMatrix", "u_MVMatrix",
			"u_NMatrix", "u_Texture", "u_Light.ambient", "u_Light.diffuse",
			"u_Light.specular", "u_Light.position", "u_Light.spotDirection",
			"u_Light.spotExponent", "u_Light.spotCosCutoff",
			"u_Light.constantAttenuation", "u_Light.linearAttenuation",
			"u_Light.quadraticAttenuation", "u_Material.diffuse",
			"u_Material.ambient", "u_Material.specular",
			"u_Material.shininess", "u_Material.hasTexture", "u_GlobalAmbient" };

	public final static byte a_Position = 0;
	public final static byte a_Normal = 1;
	public final static byte a_UV = 2;

	protected final static byte u_MVPMatrix = 0;
	protected final static byte u_MVMatrix = 1;
	protected final static byte u_NMatrix = 2;
	protected final static byte u_Texture = 3;
	protected final static byte u_LightAmbient = 4;
	protected final static byte u_LightDiffuse = 5;
	protected final static byte u_LightSpecular = 6;
	protected final static byte u_LightPosition = 7;
	protected final static byte u_LightSpotDirection = 8;
	protected final static byte u_LightSpotExponent = 9;
	protected final static byte u_LightSpotCosCutoff = 10;
	protected final static byte u_LightConstantAttenuation = 11;
	protected final static byte u_LightLinearAttenuation = 12;
	protected final static byte u_LightQuadraticAttenuation = 13;
	protected final static byte u_MaterialDiffuse = 14;
	protected final static byte u_MaterialAmbient = 15;
	protected final static byte u_MaterialSpecular = 16;
	protected final static byte u_MaterialShininess = 17;
	protected final static byte u_MaterialHasTexture = 18;
	protected final static byte u_GlobalAmbient = 19;

	protected int lastTextureUnit;
	protected Vector3f lastGlobalAmbient;
	protected Vector3f lastLightAmbient;
	protected Vector3f lastLightDiffuse;
	protected Vector3f lastLightSpecular;
	protected final Vector4f lastLightPosition = new Vector4f();
	protected Vector3f lastLightSpotDirection;
	protected float lastLightSpotExponent;
	protected float lastLightSpotCosCutoff;
	protected float lastLightConstantAttenuation;
	protected float lastLightLinearAttenuation;
	protected float lastLightQuadraticAttenuation;
	protected Vector3f lastMaterialAmbient;
	protected Vector3f lastMaterialDiffuse;
	protected Vector3f lastMaterialSpecular;
	protected float lastMaterialShininess;
	protected boolean lastMaterialHasTexture;

	public BlinnPhongShader() {
		super("res/raw/blinn_phong_shader.vsh",
				"res/raw/blinn_phong_shader.fsh");
	}

	public void setMVPMatrix(float[] mvpMatrix) {
		GL20.glUniformMatrix4(uHandle[u_MVPMatrix], false,
				MatrixExt.getMatrix(mvpMatrix));
	}

	public void setMVMatrix(float[] mvMatrix) {
		GL20.glUniformMatrix4(uHandle[u_MVMatrix], false,
				MatrixExt.getMatrix(mvMatrix));
	}

	public void setNMatrix(float[] nMatrix) {
		GL20.glUniformMatrix3(uHandle[u_NMatrix], false,
				MatrixExt.getMatrix(nMatrix));
	}

	public void setTexture(int unit) {
		if (unit != lastTextureUnit) {
			GL20.glUniform1i(uHandle[u_Texture], 0);
			lastTextureUnit = unit;
		}
	}

	public void setGlobalAmbient(Vector3f globalAmbient) {
		if (!globalAmbient.equals(lastGlobalAmbient)) {
			GL20.glUniform3f(uHandle[u_GlobalAmbient], globalAmbient.x,
					globalAmbient.y, globalAmbient.z);
			lastGlobalAmbient = globalAmbient;
		}
	}

	public void setLightAmbient(Vector3f ambientColor) {
		if (!ambientColor.equals(lastLightAmbient)) {
			GL20.glUniform3f(uHandle[u_LightAmbient], ambientColor.x,
					ambientColor.y, ambientColor.z);
			lastLightAmbient = ambientColor;
		}
	}

	public void setLightDiffuse(Vector3f diffuseColor) {
		if (!diffuseColor.equals(lastLightDiffuse)) {
			GL20.glUniform3f(uHandle[u_LightDiffuse], diffuseColor.x,
					diffuseColor.y, diffuseColor.z);
			lastLightDiffuse = diffuseColor;
		}
	}

	public void setLightSpecular(Vector3f specularColor) {
		if (!specularColor.equals(lastLightSpecular)) {
			GL20.glUniform3f(uHandle[u_LightSpecular], specularColor.x,
					specularColor.y, specularColor.z);
			lastLightSpecular = specularColor;
		}
	}

	public void setLightPosition(Vector4f position) {
		if (!position.equals(lastLightPosition)) {
			GL20.glUniform4f(uHandle[u_LightPosition], position.x, position.y,
					position.z, position.w);
			lastLightPosition.set(position);
		}
	}

	public void setLightSpotDirection(Vector3f spotDirection) {
		if (!spotDirection.equals(lastLightSpotDirection)) {
			GL20.glUniform3f(uHandle[u_LightSpotDirection], spotDirection.x,
					spotDirection.y, spotDirection.z);
			lastLightSpotDirection = spotDirection;
		}
	}

	public void setLightSpotExponent(float spotExponent) {
		if (spotExponent != lastLightSpotExponent) {
			GL20.glUniform1f(uHandle[u_LightSpotExponent], spotExponent);
			lastLightSpotExponent = spotExponent;
		}
	}

	public void setLightSpotCosCutoff(float spotCosCutoff) {
		if (spotCosCutoff != lastLightSpotCosCutoff) {
			GL20.glUniform1f(uHandle[u_LightSpotCosCutoff], spotCosCutoff);
			lastLightSpotCosCutoff = spotCosCutoff;
		}
	}

	public void setLightConstantAttenuation(float constantAttenuation) {
		if (constantAttenuation != lastLightConstantAttenuation) {
			GL20.glUniform1f(uHandle[u_LightConstantAttenuation],
					constantAttenuation);
			lastLightConstantAttenuation = constantAttenuation;
		}
	}

	public void setLightLinearAttenuation(float linearAttenuation) {
		if (linearAttenuation != lastLightLinearAttenuation) {
			GL20.glUniform1f(uHandle[u_LightLinearAttenuation],
					linearAttenuation);
			lastLightLinearAttenuation = linearAttenuation;
		}
	}

	public void setLightQuadraticAttenuation(float quadraticAttenuation) {
		if (quadraticAttenuation != lastLightQuadraticAttenuation) {
			GL20.glUniform1f(uHandle[u_LightQuadraticAttenuation],
					quadraticAttenuation);
			lastLightQuadraticAttenuation = quadraticAttenuation;
		}
	}

	public void setMaterialAmbient(Vector3f ambient) {
		if (!ambient.equals(lastMaterialAmbient)) {
			GL20.glUniform3f(uHandle[u_MaterialAmbient], ambient.x, ambient.y,
					ambient.z);
			lastMaterialAmbient = ambient;
		}
	}

	public void setMaterialDiffuse(Vector3f diffuse) {
		if (!diffuse.equals(lastMaterialDiffuse)) {
			GL20.glUniform3f(uHandle[u_MaterialDiffuse], diffuse.x, diffuse.y,
					diffuse.z);
			lastMaterialDiffuse = diffuse;
		}
	}

	public void setMaterialSpecular(Vector3f specular) {
		if (!specular.equals(lastMaterialSpecular)) {
			GL20.glUniform3f(uHandle[u_MaterialSpecular], specular.x,
					specular.y, specular.z);
			lastMaterialSpecular = specular;
		}
	}

	public void setMaterialShininess(float shininess) {
		if (shininess != lastMaterialShininess) {
			GL20.glUniform1f(uHandle[u_MaterialShininess], shininess);
			lastMaterialShininess = shininess;
		}
	}

	public void setMaterialHasTexture(boolean hasTexture) {
		if (hasTexture != lastMaterialHasTexture) {
			GL20.glUniform1i(uHandle[u_MaterialHasTexture], hasTexture ? 1 : 0);
			lastMaterialHasTexture = hasTexture;
		}
	}

	@Override
	public void initShader() {
		super.initShader();
		GL13.glActiveTexture(GL13.GL_TEXTURE0);
	}

	@Override
	public String[] getAttributes() {
		return attributes;
	}

	@Override
	public String[] getUniforms() {
		return uniforms;
	}
}
