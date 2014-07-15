package com.brocorporation.gameengine.elements.opengl;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;

import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL20;

public abstract class GLShader {

	protected final int[] aHandle;
	protected final int[] uHandle;

	protected final String vertexShaderCode;
	protected final String fragmentShaderCode;
	protected int program;

	public GLShader(final String vsh, final String fsh) {
		aHandle = new int[getAttributes().length];
		uHandle = new int[getUniforms().length];
		vertexShaderCode = readShader(vsh);
		fragmentShaderCode = readShader(fsh);
	}

	public void initShader() {
		final String[] attributes = getAttributes();
		final String[] uniforms = getUniforms();
		final int vertexShader = compileShader(GL20.GL_VERTEX_SHADER,
				vertexShaderCode);
		final int fragmentShader = compileShader(GL20.GL_FRAGMENT_SHADER,
				fragmentShaderCode);
		program = createAndLinkProgram(vertexShader, fragmentShader, attributes);

		use();
		for (int i = attributes.length - 1; i >= 0; i--) {
			aHandle[i] = GL20.glGetAttribLocation(program, attributes[i]);
		}
		for (int i = uniforms.length - 1; i >= 0; i--) {
			uHandle[i] = GL20.glGetUniformLocation(program, uniforms[i]);
		}
	}

	public void use() {
		GL20.glUseProgram(getProgram());
	}

	public int getProgram() {
		return program;
	}

	public int[] getAttributeHandles() {
		return aHandle;
	}

	public int[] getUniformHandles() {
		return uHandle;
	}

	public abstract String[] getAttributes();

	public abstract String[] getUniforms();

	protected static String readShader(final String filename) {
		BufferedReader bufferedReader = null;
		final StringBuilder body = new StringBuilder();
		String nextLine;
		try {
			bufferedReader = Files.newBufferedReader(Paths.get(filename),
					StandardCharsets.UTF_8);
			while ((nextLine = bufferedReader.readLine()) != null) {
				body.append(nextLine).append('\n');
			}
		} catch (IOException e) {
			return null;
		} finally {
			try {
				if (bufferedReader != null) {
					bufferedReader.close();
				}
			} catch (IOException e) {
			}
			bufferedReader = null;
		}
		return body.toString();
	}

	protected static int compileShader(final int shaderType,
			final String shaderSource) {
		int shaderHandle = GL20.glCreateShader(shaderType);
		if (shaderHandle != 0) {
			GL20.glShaderSource(shaderHandle, shaderSource);
			GL20.glCompileShader(shaderHandle);

			if (GL20.glGetShaderi(shaderHandle, GL20.GL_COMPILE_STATUS) == GL11.GL_FALSE) {
				System.out.println(shaderSource);
				System.err.println("ShaderHelper, Error compiling shader: "
						+ GL20.glGetShaderInfoLog(shaderHandle, 1000));
				
				GL20.glDeleteShader(shaderHandle);
				shaderHandle = 0;
			}
		}

		if (shaderHandle == 0) {
			throw new RuntimeException("Error creating shader.");
		}
		return shaderHandle;
	}

	protected static int createAndLinkProgram(final int vertexShaderHandle,
			final int fragmentShaderHandle, final String[] attributes) {
		int programHandle = GL20.glCreateProgram();

		if (programHandle != 0) {
			GL20.glAttachShader(programHandle, vertexShaderHandle);
			GL20.glAttachShader(programHandle, fragmentShaderHandle);

			if (attributes != null) {
				final int size = attributes.length;
				for (int i = 0; i < size; i++) {
					GL20.glBindAttribLocation(programHandle, i, attributes[i]);
				}
			}

			GL20.glLinkProgram(programHandle);

			if (GL20.glGetProgrami(programHandle, GL20.GL_LINK_STATUS) == GL11.GL_FALSE) {
				System.err.println("ShaderHelper, Error compiling program: "
						+ GL20.glGetProgramInfoLog(programHandle, 1000));
				GL20.glDeleteProgram(programHandle);
				programHandle = 0;
			}
		}

		if (programHandle == 0) {
			throw new RuntimeException("Error creating program.");
		}
		return programHandle;
	}
}
