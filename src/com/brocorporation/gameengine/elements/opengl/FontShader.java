package com.brocorporation.gameengine.elements.opengl;

import org.lwjgl.opengl.GL13;
import org.lwjgl.opengl.GL20;

import com.brocorporation.gameengine.utils.Vector3f;
import com.brocorporation.gameengine.utils.Vector4f;

public class FontShader extends GLShader {

	public final static String[] attributes = { "a_Position", "a_UV" };
	public final static String[] uniforms = { "u_Position", "u_Size",
			"u_Texture", "u_Color" };

	public final static byte a_Position = 0;
	public final static byte a_UV = 1;

	protected final static byte u_Position = 0;
	protected final static byte u_Size = 1;
	protected final static byte u_Texture = 2;
	protected final static byte u_Color = 3;

	protected Vector3f lastPosition;
	protected Vector4f lastColor;
	protected float lastSize, lastScalarUV;
	protected int lastTextureUnit;

	public FontShader() {
		super("res/raw/vfontshader.vsh", "res/raw/ffontshader.fsh");
	}

	public void setPosition(Vector3f position) {
		if (!position.equals(lastPosition)) {
			GL20.glUniform3f(uHandle[u_Position], position.x, position.y,
					position.z);
			lastPosition = position;
		}
	}

	public void setSize(float scalar) {
		if (scalar != lastSize) {
			GL20.glUniform1f(uHandle[u_Size], scalar);
			lastSize = scalar;
		}
	}

	public void setTextureUnit(int unit) {
		if (unit != lastTextureUnit) {
			GL20.glUniform1i(uHandle[u_Texture], unit);
			lastTextureUnit = unit;
		}
	}

	public void setColor(Vector4f color) {
		if (!color.equals(lastColor)) {
			GL20.glUniform4f(uHandle[u_Color], color.x, color.y, color.z,
					color.w);
			lastColor = color;
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
