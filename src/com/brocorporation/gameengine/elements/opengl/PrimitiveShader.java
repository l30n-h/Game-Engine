package com.brocorporation.gameengine.elements.opengl;

import org.lwjgl.opengl.GL20;

import com.brocorporation.gameengine.utils.MatrixExt;
import com.brocorporation.gameengine.utils.Vector4f;

public class PrimitiveShader extends GLShader {

	public final static String[] attributes = { "a_Position" };
	protected final static String[] uniforms = { "u_MVPMatrix", "u_Color" };

	public final static byte a_Position = 0;

	protected final static byte u_MVPMatrix = 0;
	protected final static byte u_Color = 1;

	protected final Vector4f lastColor = new Vector4f();
	protected final float[] lastVPMatrix = new float[16];

	public PrimitiveShader() {
		super("res/raw/primitive_shader.vsh", "res/raw/primitive_shader.fsh");
	}

	public void setMVPMatrix(float[] vpMatrix) {
		if (!MatrixExt.equalsM(vpMatrix, lastVPMatrix)) {
			GL20.glUniformMatrix4(uHandle[u_MVPMatrix], false,
					MatrixExt.getMatrix(vpMatrix));
			MatrixExt.setM4(lastVPMatrix, vpMatrix);
		}
	}

	public void setColor(Vector4f color) {
		if (!color.equals(lastColor)) {
			GL20.glUniform4f(uHandle[u_Color], color.x, color.y, color.z,
					color.w);
			lastColor.set(color);
		}
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
