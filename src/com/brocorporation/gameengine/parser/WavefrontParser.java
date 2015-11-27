package com.brocorporation.gameengine.parser;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;

import javax.swing.SwingWorker;

import com.brocorporation.gameengine.elements.opengl.GLMaterial;
import com.brocorporation.gameengine.elements.opengl.GLMesh;
import com.brocorporation.gameengine.elements.opengl.GLTexture;
import com.brocorporation.gameengine.utils.Utils;
import com.brocorporation.gameengine.utils.Vector3f;

public class WavefrontParser extends Thread {
	private final BufferedReader[] bufferedReader = new BufferedReader[2];
	private int buffer;
	private final ArrayList<String> tempStringList = new ArrayList<String>(3);
	private final float[] tempFloat = new float[3];

	private final WavefrontObject wavefrontObject = new WavefrontObject();
	private final GLTexture glTexture;
	private final HashMap<String, GLMaterial> wavefrontMaterialMap = new HashMap<String, GLMaterial>();
	private GLMaterial currentMaterial;
	private GLMesh currentMesh;
	private final ArrayList<float[]> vertices = new ArrayList<float[]>();
	private final ArrayList<float[]> uvs = new ArrayList<float[]>();
	private final ArrayList<float[]> normals = new ArrayList<float[]>();

	private final HashMap<String, Short> indicesMap = new HashMap<String, Short>();
	private short indicesposition = 0;

	private boolean isMeshCreated = false;

	private boolean generateBounds;
	private Vector3f currentPosition;
	private Vector3f currentHalfsize;
	private float minX = Float.POSITIVE_INFINITY;
	private float minY = Float.POSITIVE_INFINITY;
	private float minZ = Float.POSITIVE_INFINITY;
	private float maxX = Float.NEGATIVE_INFINITY;
	private float maxY = Float.NEGATIVE_INFINITY;
	private float maxZ = Float.NEGATIVE_INFINITY;

	public WavefrontParser(final String objPath, final String mtlPath, GLTexture pGLTexture) throws IOException {
		bufferedReader[0] = Files.newBufferedReader(Paths.get(mtlPath), StandardCharsets.UTF_8);
		bufferedReader[1] = Files.newBufferedReader(Paths.get(objPath), StandardCharsets.UTF_8);
		glTexture = pGLTexture;
	}

	public void parse(final boolean pGenerateBounds) {
		generateBounds = pGenerateBounds;
		buffer = 0;
		start();
	}

	private void parseMaterialFile(final String line) {
		try {
			final String linecontent = line.substring(line.indexOf(' ') + 1);
			if (line.startsWith("newmtl ")) {
				currentMaterial = new GLMaterial();
				wavefrontMaterialMap.put(linecontent, currentMaterial);
			} else if (line.startsWith("Ns ")) {
				currentMaterial.setShininess(Utils.parseFloat(linecontent, 0, linecontent.length()));
			} else if (line.startsWith("Ka ")) {
				splitFloat(linecontent, ' ', tempFloat);
				currentMaterial.setAmientColor(tempFloat[0], tempFloat[1], tempFloat[2]);
			} else if (line.startsWith("Kd ")) {
				splitFloat(linecontent, ' ', tempFloat);
				currentMaterial.setDiffuseColor(tempFloat[0], tempFloat[1], tempFloat[2]);
			} else if (line.startsWith("Ks ")) {
				splitFloat(linecontent, ' ', tempFloat);
				currentMaterial.setSpecularColor(tempFloat[0], tempFloat[1], tempFloat[2]);
			} else if (line.startsWith("illum ")) {
				currentMaterial.setIlluminance((short) Utils.parseFloat(linecontent, 0, linecontent.length()));
			} else if (line.startsWith("Ni ")) {
				currentMaterial.setRefractionIndex(Utils.parseFloat(linecontent, 0, linecontent.length()));
			} else if (line.startsWith("d ")) {
				currentMaterial.setDissolveFaktor(Utils.parseFloat(linecontent, 0, linecontent.length()));
			} else if (line.startsWith("map_Kd ")) {
				currentMaterial.setTexture(linecontent);
				glTexture.loadTexture(linecontent);
			}
		} catch (Exception e) {
		}
	}

	private void parseObjectFile(final String line) {
		try {
			final String linecontent = line.substring(line.indexOf(' ') + 1);
			if (line.startsWith("o ")) {
				isMeshCreated = true;
				currentMesh = new GLMesh();
				wavefrontObject.addMesh(currentMesh);
				if (generateBounds) {
					resetBounds();
					currentPosition = currentMesh.getAABB().getPosition();
					currentHalfsize = currentMesh.getAABB().getHalfsize();
				}
			} else if (line.startsWith("v ")) {
				vertices.add(splitToVertex(linecontent, ' ', new float[3]));
			} else if (line.startsWith("vt ")) {
				uvs.add(splitToVertex(linecontent, ' ', new float[2]));
			} else if (line.startsWith("vn ")) {
				normals.add(splitToVertex(linecontent, ' ', new float[3]));
			} else if (line.startsWith("usemtl ")) {
				if (!isMeshCreated) {
					currentMesh = new GLMesh();
					wavefrontObject.addMesh(currentMesh);
					if (generateBounds) {
						resetBounds();
						currentPosition = currentMesh.getAABB().getPosition();
						currentHalfsize = currentMesh.getAABB().getHalfsize();
					}
				}
				isMeshCreated = false;
				currentMesh.setMaterial(wavefrontMaterialMap.get(linecontent));
			} else if (line.startsWith("f ")) {
				final ArrayList<String> sIndices = splitString(linecontent, ' ');
				for (final String sIndex : sIndices) {
					final Short indexObj;
					if ((indexObj = indicesMap.get(sIndex)) == null) {
						splitFloat(sIndex, '/', tempFloat);
						for (int i = 0; i < tempFloat.length; i++) {
							final float index = tempFloat[i];
							if (Float.isNaN(index) == false) {
								short start = (short) (index - 1);
								switch (i) {
								case 0:
									final float[] vertex = vertices.get(start);
									if (generateBounds) {
										checkBounds(vertex);
									}
									wavefrontObject.addVBO(vertex);
									break;
								case 1:
									wavefrontObject.addVBO(uvs.get(start));
									currentMesh.hasUVs(true);
									break;
								case 2:
									wavefrontObject.addVBO(normals.get(start));
									break;
								}
							} else {
								switch (i) {
								case 1:
									wavefrontObject.addVBO(new float[2]);
									currentMesh.hasUVs(false);
									break;
								}
							}
						}
						indicesMap.put(sIndex, indicesposition);
						currentMesh.addIndex();
						wavefrontObject.addIndex(indicesposition);
						indicesposition++;
					} else {
						currentMesh.addIndex();
						wavefrontObject.addIndex(indexObj);
					}
				}
			}
		} catch (Exception e) {
		}
	}

	private void read(BufferedReader reader) {
		String line;
		try {
			if (reader != null) {
				while ((line = reader.readLine()) != null) {
					switch (buffer) {
					case 0:
						parseMaterialFile(line);
						break;
					case 1:
						parseObjectFile(line);
						break;
					}
				}
			}
		} catch (Exception e) {
		} finally {
			try {
				reader.close();
			} catch (Exception e) {
			}
			reader = null;
		}
	}

	@Override
	public void run() {
		read(bufferedReader[buffer]);
		buffer++;
		read(bufferedReader[buffer]);
	}

	public WavefrontObject get() throws InterruptedException {
		join();
		return wavefrontObject;
	}

	private float[] splitToVertex(final String line, final char split, final float[] vector) {
		int start = 0;
		int end = line.indexOf(split, start);
		int i = 0;
		while (end != -1 && i < vector.length) {
			vector[i++] = Utils.parseFloat(line, start, end);
			start = end + 1;
			end = line.indexOf(split, start);
		}
		end = line.length();
		vector[i] = Utils.parseFloat(line, start, end);
		return vector;
	}

	private void splitFloat(final String line, final char split, final float[] array) {
		int count = 0;
		int start = 0;
		int end = line.indexOf(split, start);
		while (end != -1) {
			if (end - start > 0) {
				array[count] = Utils.parseFloat(line, start, end);
			} else {
				array[count] = Float.NaN;
			}
			count++;
			start = end + 1;
			end = line.indexOf(split, start);
		}
		end = line.length();
		if (end - start > 0) {
			array[count] = Utils.parseFloat(line, start, end);
		} else {
			array[count] = Float.NaN;
		}
	}

	private ArrayList<String> splitString(final String line, final char split) {
		tempStringList.clear();
		int i2 = 0;
		int j2 = line.indexOf(split);
		while (j2 >= 0) {
			tempStringList.add(line.substring(i2, j2));
			i2 = j2 + 1;
			j2 = line.indexOf(split, i2);
		}
		tempStringList.add(line.substring(i2));
		return tempStringList;
	}

	private void checkBounds(final float[] vertex) {
		if (vertex[0] < minX) {
			minX = vertex[0];
		} else if (vertex[0] > maxX) {
			maxX = vertex[0];
		}
		if (vertex[1] < minY) {
			minY = vertex[1];
		} else if (vertex[1] > maxY) {
			maxY = vertex[1];
		}
		if (vertex[2] < minZ) {
			minZ = vertex[2];
		} else if (vertex[2] > maxZ) {
			maxZ = vertex[2];
		}

		currentHalfsize.set(Math.abs(maxX - minX) / 2, Math.abs(maxY - minY) / 2, Math.abs(maxZ - minZ) / 2);
		currentPosition.set(maxX - currentHalfsize.x, maxY - currentHalfsize.y, maxZ - currentHalfsize.z);
	}

	private void resetBounds() {
		minX = Float.POSITIVE_INFINITY;
		minY = Float.POSITIVE_INFINITY;
		minZ = Float.POSITIVE_INFINITY;
		maxX = Float.NEGATIVE_INFINITY;
		maxY = Float.NEGATIVE_INFINITY;
		maxZ = Float.NEGATIVE_INFINITY;
	}
}
