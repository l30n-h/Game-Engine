package com.brocorporation.gameengine.elements.opengl;

import java.awt.image.BufferedImage;
import java.io.File;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Set;

import javax.imageio.ImageIO;

import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL30;

public class GLTexture {

	protected final static byte BYTES_PER_RGB_PIXEL = 3;
	protected final static byte BYTES_PER_RGBA_PIXEL = 4;

	private final HashMap<String, BufferedImage> nameBitmapList = new HashMap<String, BufferedImage>();
	private final HashMap<String, Integer> nameHandleMap = new HashMap<String, Integer>();

	private static String currentTexture = null;
	private static int currentTextureHandle = 0;

	public void loadTexture(final String pTexture) {
		if (pTexture != null && pTexture.isEmpty() == false
				&& nameBitmapList.containsKey(pTexture) == false) {
			try {
				final File file = new File("assets/" + pTexture);
				if (file != null) {
					final BufferedImage image = ImageIO.read(file);
					if (image != null) {
						nameBitmapList.put(pTexture, image);
					}
				}
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

	public void initTextures(final boolean mipmap) {
		nameHandleMap.clear();
		final Set<Entry<String, BufferedImage>> set = nameBitmapList.entrySet();
		final int size = set.size();
		final IntBuffer textureHandle = BufferUtils.createIntBuffer(size);
		GL11.glGenTextures(textureHandle);
		for (final Entry<String, BufferedImage> entry : set) {
			final String name = entry.getKey();
			if (nameHandleMap.containsKey(name) == false) {
				final int texHandle = textureHandle.get();
				if (texHandle != 0) {
					final BufferedImage image = entry.getValue();
					GL11.glBindTexture(GL11.GL_TEXTURE_2D, texHandle);
					final int width = image.getWidth();
					final int height = image.getHeight();
					final int[] pixels = new int[width * height];
					image.getRGB(0, 0, width, height, pixels, 0, width);
					if (image.getColorModel().hasAlpha()) {
						final ByteBuffer buffer = BufferUtils
								.createByteBuffer(pixels.length
										* BYTES_PER_RGBA_PIXEL);
						int ywidth = 0;
						for (int y = 0; y < height; y++) {
							for (int x = 0; x < width; x++) {
								final int pixel = pixels[ywidth + x];
								buffer.put((byte) ((pixel >> 16) & 0xFF));
								buffer.put((byte) ((pixel >> 8) & 0xFF));
								buffer.put((byte) (pixel & 0xFF));
								buffer.put((byte) ((pixel >> 24) & 0xFF));
							}
							ywidth += width;
						}

						buffer.flip();

						GL11.glTexImage2D(GL11.GL_TEXTURE_2D, 0, GL11.GL_RGBA8,
								width, height, 0, GL11.GL_RGBA,
								GL11.GL_UNSIGNED_BYTE, buffer);
					} else {
						final ByteBuffer buffer = BufferUtils
								.createByteBuffer(pixels.length
										* BYTES_PER_RGB_PIXEL);
						int ywidth = 0;
						for (int y = 0; y < height; y++) {
							for (int x = 0; x < width; x++) {
								int pixel = pixels[ywidth + x];
								buffer.put((byte) ((pixel >> 16) & 0xFF));
								buffer.put((byte) ((pixel >> 8) & 0xFF));
								buffer.put((byte) (pixel & 0xFF));
							}
							ywidth += width;
						}
						buffer.flip();

						GL11.glTexImage2D(GL11.GL_TEXTURE_2D, 0, GL11.GL_RGB8,
								width, height, 0, GL11.GL_RGB,
								GL11.GL_UNSIGNED_BYTE, buffer);
					}

					if (mipmap) {
						GL30.glGenerateMipmap(GL11.GL_TEXTURE_2D);
						GL11.glTexParameteri(GL11.GL_TEXTURE_2D,
								GL11.GL_TEXTURE_MAG_FILTER, GL11.GL_NEAREST);
						GL11.glTexParameteri(GL11.GL_TEXTURE_2D,
								GL11.GL_TEXTURE_MIN_FILTER,
								GL11.GL_NEAREST_MIPMAP_NEAREST);
					} else {
						GL11.glTexParameteri(GL11.GL_TEXTURE_2D,
								GL11.GL_TEXTURE_MAG_FILTER, GL11.GL_NEAREST);
						GL11.glTexParameteri(GL11.GL_TEXTURE_2D,
								GL11.GL_TEXTURE_MIN_FILTER, GL11.GL_NEAREST);
					}
					GL11.glTexParameteri(GL11.GL_TEXTURE_2D,
							GL11.GL_TEXTURE_WRAP_S, GL11.GL_REPEAT);
					GL11.glTexParameteri(GL11.GL_TEXTURE_2D,
							GL11.GL_TEXTURE_WRAP_T, GL11.GL_REPEAT);
					nameHandleMap.put(name, texHandle);
				}
			}
		}
		GL11.glDeleteTextures(textureHandle);
		textureHandle.clear();
	}

	public void initTexturesAndRecycle(final boolean mipmap) {
		initTextures(mipmap);
		nameBitmapList.clear();
	}

	public int getTextureHandle(final String pTexture) {
		final Integer handle = nameHandleMap.get(pTexture);
		return handle != null ? handle : 0;
	}
	
	public void bindTexture(final String texture) {
		if (texture != currentTexture) {
			final int texturehandle = getTextureHandle(texture);
			if (texturehandle != currentTextureHandle) {
				GL11.glBindTexture(GL11.GL_TEXTURE_2D, texturehandle);
				currentTextureHandle = texturehandle;
				currentTexture = texture;
			}
		}
	}

	public int getCurrentTextureHandle() {
		return currentTextureHandle;
	}

	public String getCurrentTexture() {
		return currentTexture;
	}
}
