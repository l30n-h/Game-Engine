package com.brocorporation.gameengine;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.lwjgl.input.Keyboard;
import org.lwjgl.input.Mouse;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.DisplayMode;
import org.lwjgl.opengl.GL11;

import com.brocorporation.gameengine.elements.bodies.Actor;
import com.brocorporation.gameengine.elements.bodies.Camera;
import com.brocorporation.gameengine.elements.bodies.DynamicBody;
import com.brocorporation.gameengine.elements.bodies.Item;
import com.brocorporation.gameengine.elements.bodies.Plane;
import com.brocorporation.gameengine.elements.bodies.RigidBody;
import com.brocorporation.gameengine.elements.bodies.StaticBody;
import com.brocorporation.gameengine.elements.bodies.StaticLight;
import com.brocorporation.gameengine.elements.bodies.TrackingCamera;
import com.brocorporation.gameengine.elements.bodies.World;
import com.brocorporation.gameengine.elements.collision.AABB;
import com.brocorporation.gameengine.elements.collision.Collidable;
import com.brocorporation.gameengine.elements.collision.Convex;
import com.brocorporation.gameengine.elements.collision.Frustum;
import com.brocorporation.gameengine.elements.collision.Manifold;
import com.brocorporation.gameengine.elements.collision.Manifold.ManifoldContact;
import com.brocorporation.gameengine.elements.collision.Material;
import com.brocorporation.gameengine.elements.collision.Octree;
import com.brocorporation.gameengine.elements.collision.RaycastHit;
import com.brocorporation.gameengine.elements.collision.Sphere;
import com.brocorporation.gameengine.elements.opengl.BlinnPhongLight;
import com.brocorporation.gameengine.elements.opengl.BlinnPhongShader;
import com.brocorporation.gameengine.elements.opengl.FontShader;
import com.brocorporation.gameengine.elements.opengl.FontShape;
import com.brocorporation.gameengine.elements.opengl.GLShape;
import com.brocorporation.gameengine.elements.opengl.GLTexture;
import com.brocorporation.gameengine.elements.opengl.MainShape;
import com.brocorporation.gameengine.elements.opengl.PrimitiveShader;
import com.brocorporation.gameengine.elements.opengl.PrimitiveShape;
import com.brocorporation.gameengine.parser.WavefrontParser;
import com.brocorporation.gameengine.utils.MatrixExt;
import com.brocorporation.gameengine.utils.Quaternion;
import com.brocorporation.gameengine.utils.Vector3f;

public class MyGLSurfaceView extends GameEngine {

	private MainShape roomShape;
	private MainShape boxShape;
	private MainShape coinShape;
	private MainShape sphereShape;

	private World world;
	private Actor currentActor;
	private Actor actor;
	private Actor actor2;

	private float[] projectionMatrix;
	private float[] viewMatrix;
	private final float[] modelMatrix = new float[16];
	private final float[] vpMatrix = new float[16];
	private final float[] mvpTempMatrix = new float[16];

	private final float[] lightPosInModelSpace = new float[] { 0, 0, 0, 1 };
	private final float[] lightPosInEyeSpace = new float[4];

	private int width, height;

	private final Frustum frustum = new Frustum();
	private PrimitiveShader primitiveShader;
	private PrimitiveShape primitiveShape;
	private BlinnPhongShader blinnPhongShader;
	private FontShader fontShader;
	private FontShape fontShape;
	private GLTexture glTexture;
	private BlinnPhongLight blinnPhongLight;
	private final List<Object> frustumList = new ArrayList<Object>();

	public MyGLSurfaceView() {
		super(60);
	}

	public void create() {
		try {
			Display.setDisplayMode(new DisplayMode(800, 500));
			Display.setTitle("School Run");
			Display.setResizable(true);
			Display.create();
		} catch (Exception e) {
			e.printStackTrace();
			Display.destroy();
			System.exit(1);
		}
	}

	@Override
	protected void load() {
		try {
			glTexture = new GLTexture();
			primitiveShader = new PrimitiveShader();
			primitiveShape = new PrimitiveShape(primitiveShader);
			blinnPhongShader = new BlinnPhongShader();
			fontShader = new FontShader();
			fontShape = new FontShape(fontShader, glTexture);
			blinnPhongLight = new BlinnPhongLight(blinnPhongShader);
			final WavefrontParser roomParser = new WavefrontParser(
					"assets/VPK+Treppe+3.obj", "assets/VPK+Treppe+3.mtl",
					glTexture);
			roomParser.parse(true);

			final WavefrontParser boxParser = new WavefrontParser(
					"assets/Human3.obj", "assets/Human3.mtl", glTexture);
			boxParser.parse(false);
			final WavefrontParser sphereParser = new WavefrontParser(
					"assets/ball_smooth.obj", "assets/ball_smooth.mtl", glTexture);
			sphereParser.parse(false);

			final WavefrontParser coinParser = new WavefrontParser(
					"assets/Coin.obj", "assets/Coin.mtl", glTexture);
			coinParser.parse(false);

			roomShape = new MainShape(blinnPhongShader, glTexture,
					roomParser.get());
			boxShape = new MainShape(blinnPhongShader, glTexture,
					boxParser.get());
			sphereShape = new MainShape(blinnPhongShader, glTexture,
					sphereParser.get());
			coinShape = new MainShape(blinnPhongShader, glTexture,
					coinParser.get());
			glTexture.loadTexture("font_texture.png");
			roomShape.setFrustum(frustum);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	protected void init() {
		world = new World();

		final Vector3f normalLeft = new Vector3f(-1, 0, 0);
		final Vector3f normalRight = new Vector3f(1, 0, 0);
		final Vector3f normalFront = new Vector3f(0, 0, 1);
		final Vector3f normalBack = new Vector3f(0, 0, -1);
		final Vector3f normalUp = new Vector3f(0, 1, 0);
		final Vector3f normalDown = new Vector3f(0, -1, 0);
		final Vector3f normalStairsBig = new Vector3f(-0.48564293117F,
				0.87415727612f, 0);
		final Vector3f normalStairsSmall = new Vector3f(0.48564293117F,
				0.87415727612F, 0);
		final Vector3f normalStairsBigCeiling = new Vector3f(0.48564293117F,
				-0.87415727612F, 0);
		final Vector3f normalStairsSmallCeiling = new Vector3f(-0.48564293117F,
				-0.87415727612F, 0);
		final Material ceilingMaterial = new Material(1, 1);
		for (int i = 0; i < 2; i++) {
			final float height = i * 4;
			final Convex stairsBig = new Convex(new Vector3f[] {
					new Vector3f(4.04F, -1.75F + height, -22.32F),
					new Vector3f(4.04F, -1.75F + height, -20.28F),
					new Vector3f(7.64F, 0.25F + height, -20.28F),
					new Vector3f(7.64F, 0.25F + height, -22.32F) });
			final Convex stairsBigCeiling = new Convex(new Vector3f[] {
					new Vector3f(4.4F, -2.25F + height, -22.32F),
					new Vector3f(4.4F, -2.25F + height, -20.28F),
					new Vector3f(8F, -0.25F + height, -20.28F),
					new Vector3f(8F, -0.25F + height, -22.32F) });
			final Convex stairsBackBig = new Convex(new Vector3f[] {
					new Vector3f(4.4F, -1.75F + height, -22.32F),
					new Vector3f(4.4F, -2.25F + height, -22.32F),
					new Vector3f(8F, 0.25F + height, -22.32F),
					new Vector3f(8F, -0.25F + height, -22.32F) });
			final Convex stairsFrontBig = new Convex(new Vector3f[] {
					new Vector3f(4.4F, -1.75F + height, -20.28F),
					new Vector3f(4.4F, -2.25F + height, -20.28F),
					new Vector3f(8F, 0.25F + height, -20.28F),
					new Vector3f(8F, -0.25F + height, -20.28F) });
			final Convex stairsSmallRight = new Convex(new Vector3f[] {
					new Vector3f(8.36F, 0.25F + height, -22.48F),
					new Vector3f(8.36F, 0.25F + height, -24F),
					new Vector3f(4.76F, 2.25F + height, -24F),
					new Vector3f(4.76F, 2.25F + height, -22.48F) });
			final Convex stairsSmallRightCeiling = new Convex(new Vector3f[] {
					new Vector3f(8F, -0.25F + height, -22.48F),
					new Vector3f(8F, -0.25F + height, -24F),
					new Vector3f(4.4F, 1.75F + height, -24F),
					new Vector3f(4.4F, 1.75F + height, -22.48F) });
			final Convex stairsBackSmallRight = new Convex(new Vector3f[] {
					new Vector3f(8F, 0.25F + height, -22.48F),
					new Vector3f(8F, -0.25F + height, -22.48F),
					new Vector3f(4.4F, 2.25F + height, -22.48F),
					new Vector3f(4.4F, 1.75F + height, -22.48F) });
			final Convex stairsSmallLeft = new Convex(new Vector3f[] {
					new Vector3f(8.36F, 0.25F + height, -18.6F),
					new Vector3f(8.36F, 0.25F + height, -20.12F),
					new Vector3f(4.76F, 2.25F + height, -20.12F),
					new Vector3f(4.76F, 2.25F + height, -18.6F) });
			final Convex stairsSmallLeftCeiling = new Convex(new Vector3f[] {
					new Vector3f(8F, -0.25F + height, -18.6F),
					new Vector3f(8F, -0.25F + height, -20.12F),
					new Vector3f(4.4F, 1.75F + height, -20.12F),
					new Vector3f(4.4F, 1.75F + height, -18.6F) });
			final Convex stairsFrontSmallLeft = new Convex(new Vector3f[] {
					new Vector3f(8F, 0.25F + height, -20.12F),
					new Vector3f(8F, -0.25F + height, -20.12F),
					new Vector3f(4.4F, 2.25F + height, -20.12F),
					new Vector3f(4.4F, 1.75F + height, -20.12F) });
			final Convex floorSmall = new Convex(new Vector3f[] {
					new Vector3f(7.64F, 0.25F + height, -24F),
					new Vector3f(7.64F, 0.25F + height, -18.6F),
					new Vector3f(10.4F, 0.25F + height, -18.6F),
					new Vector3f(10.4F, 0.25F + height, -24F) });

			final Convex stairsSmallRight2 = new Convex(new Vector3f[] {
					new Vector3f(4.04F, -1.75F + height, 20.12F),
					new Vector3f(4.04F, -1.75F + height, 18.6F),
					new Vector3f(7.64F, 0.25F + height, 18.6F),
					new Vector3f(7.64F, 0.25F + height, 20.12F) });
			final Convex stairsSmallRightCeiling2 = new Convex(new Vector3f[] {
					new Vector3f(4.4F, -2.25F + height, 20.12F),
					new Vector3f(4.4F, -2.25F + height, 18.6F),
					new Vector3f(8F, -0.25F + height, 18.6F),
					new Vector3f(8F, -0.25F + height, 20.12F) });
			final Convex stairsBackSmallRight2 = new Convex(new Vector3f[] {
					new Vector3f(4.4F, 1.75F + height, 20.12F),
					new Vector3f(4.4F, -2.25F + height, 20.12F),
					new Vector3f(8F, 0.25F + height, 20.12F),
					new Vector3f(8F, -0.25F + height, 20.12F) });

			final Convex stairsSmallLeft2 = new Convex(new Vector3f[] {
					new Vector3f(8.36F, 0.25F + height, 21.8F),
					new Vector3f(8.36F, 0.25F + height, 20.28F),
					new Vector3f(4.76F, 2.25F + height, 20.28F),
					new Vector3f(4.76F, 2.25F + height, 21.8F) });
			final Convex stairsSmallLeftCeiling2 = new Convex(new Vector3f[] {
					new Vector3f(8F, -0.25F + height, 21.8F),
					new Vector3f(8F, -0.25F + height, 20.28F),
					new Vector3f(4.4F, 1.75F + height, 20.28F),
					new Vector3f(4.4F, 1.75F + height, 21.8F) });
			final Convex stairsFrontSmallLeft2 = new Convex(new Vector3f[] {
					new Vector3f(8F, 0.25F + height, 20.28F),
					new Vector3f(8F, -0.25F + height, 20.28F),
					new Vector3f(4.4F, 2.25F + height, 20.28F),
					new Vector3f(4.4F, 1.75F + height, 20.28F) });

			final Convex floorSmall2 = new Convex(new Vector3f[] {
					new Vector3f(7.64F, 0.25F + height, 18.6F),
					new Vector3f(7.64F, 0.25F + height, 21.8F),
					new Vector3f(10.4F, 0.25F + height, 21.8F),
					new Vector3f(10.4F, 0.25F + height, 18.6F) });
			world.add(new Plane(normalStairsBig, stairsBig));
			final Plane stairsBigCeilingPlane = new Plane(
					normalStairsBigCeiling, stairsBigCeiling);
			stairsBigCeilingPlane.setMaterial(ceilingMaterial);
			world.add(stairsBigCeilingPlane);
			world.add(new Plane(normalBack, stairsBackBig));
			world.add(new Plane(normalFront, stairsFrontBig));
			world.add(new Plane(normalStairsSmall, stairsSmallRight));
			final Plane stairsSmallRightCeilingPlane = new Plane(
					normalStairsSmallCeiling, stairsSmallRightCeiling);
			stairsSmallRightCeilingPlane.setMaterial(ceilingMaterial);
			world.add(stairsSmallRightCeilingPlane);
			world.add(new Plane(normalFront, stairsBackSmallRight));
			world.add(new Plane(normalStairsSmall, stairsSmallLeft));
			final Plane stairsSmallLeftCeilingPlane = new Plane(
					normalStairsSmallCeiling, stairsSmallLeftCeiling);
			stairsSmallLeftCeilingPlane.setMaterial(ceilingMaterial);
			world.add(stairsSmallLeftCeilingPlane);
			world.add(new Plane(normalBack, stairsFrontSmallLeft));
			world.add(new Plane(normalUp, floorSmall));

			world.add(new Plane(normalStairsBig, stairsSmallRight2));
			final Plane stairsSmallRightCeilingPlane2 = new Plane(
					normalStairsBigCeiling, stairsSmallRightCeiling2);
			stairsSmallRightCeilingPlane2.setMaterial(ceilingMaterial);
			world.add(stairsSmallRightCeilingPlane2);
			world.add(new Plane(normalFront, stairsBackSmallRight2));

			world.add(new Plane(normalStairsSmall, stairsSmallLeft2));
			final Plane stairsSmallLeftCeilingPlane2 = new Plane(
					normalStairsSmallCeiling, stairsSmallLeftCeiling2);
			stairsSmallLeftCeilingPlane2.setMaterial(ceilingMaterial);
			world.add(stairsSmallLeftCeilingPlane2);
			world.add(new Plane(normalBack, stairsFrontSmallLeft2));
			world.add(new Plane(normalUp, floorSmall2));
		}
		final Convex floorBig1 = new Convex(new Vector3f[] {
				new Vector3f(-1.6F, -1.75F, -24F),
				new Vector3f(-1.6F, -1.75F, 21.8F),
				new Vector3f(10.4F, -1.75F, 21.8F),
				new Vector3f(10.4F, -1.75F, -24F) });
		final Convex floorBig2 = new Convex(new Vector3f[] {
				new Vector3f(-1.6F, 2.25F, -24F),
				new Vector3f(-1.6F, 2.25F, 21.8F),
				new Vector3f(4.7592F, 2.25F, 21.8F),
				new Vector3f(4.7592F, 2.25F, -24F) });
		final Convex floorBig3 = new Convex(new Vector3f[] {
				new Vector3f(-1.6F, 6.25F, -24F),
				new Vector3f(-1.6F, 6.25F, 21.8F),
				new Vector3f(4.7592F, 6.25F, 21.8F),
				new Vector3f(4.7592F, 6.25F, -24F) });
		final Convex ceilingBig1 = new Convex(new Vector3f[] {
				new Vector3f(-1.6F, 1.75F, -24F),
				new Vector3f(-1.6F, 1.75F, 21.8F),
				new Vector3f(4.759F, 1.75F, 21.8F),
				new Vector3f(4.759F, 1.75F, -24F) });
		final Convex ceilingBig2 = new Convex(new Vector3f[] {
				new Vector3f(-1.6F, 5.75F, -24F),
				new Vector3f(-1.6F, 5.75F, 21.8F),
				new Vector3f(4.759F, 5.75F, 21.8F),
				new Vector3f(4.759F, 5.75F, -24F) });
		final Convex ceilingBig3 = new Convex(new Vector3f[] {
				new Vector3f(-1.6F, 9.75F, -24F),
				new Vector3f(-1.6F, 9.75F, 21.8F),
				new Vector3f(10.4F, 9.75F, 21.8F),
				new Vector3f(10.4F, 9.75F, -24F) });
		final Convex ceilingSmall1 = new Convex(new Vector3f[] {
				new Vector3f(7.64F, 3.75F, -24F),
				new Vector3f(7.64F, 3.75F, -18.6F),
				new Vector3f(10.4F, 3.75F, -18.6F),
				new Vector3f(10.4F, 3.75F, -24F) });
		final Convex ceilingSmall2 = new Convex(new Vector3f[] {
				new Vector3f(7.64F, 3.75F, 18.6F),
				new Vector3f(7.64F, 3.75F, 21.8F),
				new Vector3f(10.4F, 3.75F, 21.8F),
				new Vector3f(10.4F, 3.75F, 18.6F) });
		final Convex wallLeft = new Convex(new Vector3f[] {
				new Vector3f(-1.1F, -1.75F, -24F),
				new Vector3f(-1.1F, -1.75F, 21.8F),
				new Vector3f(-1.1F, 9.75F, 21.8F),
				new Vector3f(-1.1F, 9.75F, -24F) });
		final Convex wallRight = new Convex(new Vector3f[] {
				new Vector3f(1.1F, -1.75F, -18.6F),
				new Vector3f(1.1F, -1.75F, 18.6F),
				new Vector3f(1.1F, 9.75F, 18.6F),
				new Vector3f(1.1F, 9.75F, -18.6F) });
		final Convex wallFront = new Convex(new Vector3f[] {
				new Vector3f(-1.1F, -1.75F, 21.8F),
				new Vector3f(10.4F, -1.75F, 21.8F),
				new Vector3f(10.4F, 9.75F, 21.8F),
				new Vector3f(-1.1F, 9.75F, 21.8F) });
		final Convex wallBack = new Convex(new Vector3f[] {
				new Vector3f(-1.6F, -1.75F, -24F),
				new Vector3f(10.4F, -1.75F, -24F),
				new Vector3f(10.4F, 9.75F, -24F),
				new Vector3f(-1.6F, 9.75F, -24F) });
		final Convex wallFront2 = new Convex(new Vector3f[] {
				new Vector3f(1.1F, -1.75F, -18.6F),
				new Vector3f(10.4F, -1.75F, -18.6F),
				new Vector3f(10.4F, 9.75F, -18.6F),
				new Vector3f(1.1F, 9.75F, -18.6F) });
		final Convex wallBack2 = new Convex(new Vector3f[] {
				new Vector3f(1.1F, -1.75F, 18.6F),
				new Vector3f(10.4F, -1.75F, 18.6F),
				new Vector3f(10.4F, 9.75F, 18.6F),
				new Vector3f(1.1F, 9.75F, 18.6F) });
		final Convex wallRight2 = new Convex(new Vector3f[] {
				new Vector3f(10.4F, -1.75F, -24F),
				new Vector3f(10.4F, -1.75F, -18.6F),
				new Vector3f(10.4F, 9.75F, -18.6F),
				new Vector3f(10.4F, 9.75F, -24F) });
		final Convex wallRight3 = new Convex(new Vector3f[] {
				new Vector3f(10.4F, -1.75F, 18.6F),
				new Vector3f(10.4F, -1.75F, 21.8F),
				new Vector3f(10.4F, 9.75F, 21.8F),
				new Vector3f(10.4F, 9.75F, 18.6F) });
		world.add(new Plane(normalUp, floorBig1));
		// world.add(new RigidBody(floorBig1, 0));
		world.add(new Plane(normalUp, floorBig2));
		world.add(new Plane(normalUp, floorBig3));
		world.add(new Plane(normalDown, ceilingBig1));
		world.add(new Plane(normalDown, ceilingBig2));
		world.add(new Plane(normalDown, ceilingBig3));
		world.add(new Plane(normalDown, ceilingSmall1));
		world.add(new Plane(normalDown, ceilingSmall2));
		world.add(new Plane(normalLeft, wallRight));
		world.add(new Plane(normalRight, wallLeft));
		world.add(new Plane(normalBack, wallFront));
		world.add(new Plane(normalFront, wallBack));
		world.add(new Plane(normalBack, wallFront2));
		world.add(new Plane(normalFront, wallBack2));
		world.add(new Plane(normalLeft, wallRight2));
		world.add(new Plane(normalLeft, wallRight3));

		final int coinsCount = 20;
		final int hl = coinsCount / 2;
		for (int i = 0; i < coinsCount; i++) {
			final Item coin = new Item(new Sphere(0.15f), 0);
			coin.setPosition(-0.5F, -Math.abs(hl - i) * 0.1F, 5 - 0.8F * i);
			// final float x = ((float) Math.random() * 2 - 1);
			// final float y = ((float) Math.random() * 2 - 1);
			// final float z = ((float) Math.random() * 36 - 18);
			// coin.setPosition(x, y, z);
			coin.rotate(0, 30 * i, 0);
			coin.setAngularVelocity(0, 180, 0);
			coin.setGLShape(coinShape);
			world.add(coin);
		}
		Vector3f[] a = new Vector3f[] { new Vector3f(+0.2f, +0.85f, +0.2f),
				new Vector3f(+0.2f, +0.85f, -0.2f),
				new Vector3f(+0.2f, -0.85f, +0.2f),
				new Vector3f(+0.2f, -0.85f, -0.2f),
				new Vector3f(-0.2f, +0.85f, +0.2f),
				new Vector3f(-0.2f, +0.85f, -0.2f),
				new Vector3f(-0.2f, -0.85f, +0.2f),
				new Vector3f(-0.2f, -0.85f, -0.2f) };
		float a_x = 0.85f, a_y = 0.85f, a_z = 0.85f;
		Vector3f[] a2 = new Vector3f[] { new Vector3f(+a_x, +a_y, +a_z),
				new Vector3f(+a_x, +a_y, -a_z), new Vector3f(+a_x, -a_y, +a_z),
				new Vector3f(+a_x, -a_y, -a_z), new Vector3f(-a_x, +a_y, +a_z),
				new Vector3f(-a_x, +a_y, -a_z), new Vector3f(-a_x, -a_y, +a_z),
				new Vector3f(-a_x, -a_y, -a_z) };

//		 final int bodyCount = 10;
//		 final int hCount = bodyCount / 2;
//		 final Material m = new Material(1*0,1.0f*0,0.9f*0);
//		 for (int i = 0; i < bodyCount; i++) {
//		 // final RigidBody b = new RigidBody(new Sphere(0.75f), (i * 20 + 20)
//		 // * 0 + 10);
//		 // b.setGLShape(sphereShape);
//		 final RigidBody b = new RigidBody(new Convex(a2), (i * 20 + 20) * 0 +
//		 10);
//		 b.setGLShape(boxShape);
//		 b.setMaterial(m);
//		 b.isGravityEnabled(true);
//		 world.add(b);
//		
//		 if (i < hCount) {
//		 b.setPosition(0, -1.75f+a_y, 10 - i * 2*(a_z+0.1f));
//		 } else {
//		 b.setPosition(0, -1.75f+3*a_y, 10 - (i - hCount) * 2*(a_z+0.1f));
//		 }
//		 }

		actor2 = new Actor(new Convex(a), Actor.INFINITY_MASS);
		actor2.setPosition(0, 0 - 0.9f, 12);
		actor2.setGLShape(boxShape);
		actor2.rotate(0, 140, 0);
		actor2.setJumpingHeight(2);
		actor2.isGravityEnabled(true);
		actor2.setMaxVelocity(13);
		world.add(actor2);

		actor = new Actor(new Convex(a), 80);
		actor.setGLShape(boxShape);
//		 actor = new Actor(new Sphere(0.75f), 80);
//		 actor.setGLShape(sphereShape);
		actor.setPosition(0, 0 - 0.9f * 0-1.73f*0, 9);
//		 actor.setPosition(7, 8, -21.25f);
		actor.setJumpingHeight(1);
		actor.isGravityEnabled(true);
//		 actor.setAngularVelocity(10, 10, 10);
		actor.setMaterial(new Material(1f * 0, 1f, 0.5f));
		world.add(actor);
		World.debugid = actor.getID();
		currentActor = actor2;

		final TrackingCamera camera = new TrackingCamera();
		camera.setZoom(0.3f);
		updown.getQuaternionEuler(-30, 0, 0);
		qu.setMultiply(leftright, updown).norm();
		qu.rotateV(followOffsetTmp, followOffset);
		camera.followBody(currentActor, followOffsetTmp.x, followOffsetTmp.y, followOffsetTmp.z);
		camera.lookAtBody(currentActor.getPosition(), 0, 1.6F, 0);
		final StaticLight light = new StaticLight(new AABB(), 0);
		light.followBody(currentActor, 0, 1.7F, -3.5f);
		world.setActiveCamera(camera);
		world.setActiveLight(light);

		glTexture.initTexturesAndRecycle(true);
		fontShader.initShader();
		primitiveShader.initShader();
		blinnPhongShader.initShader();
		blinnPhongLight.setConstantAttenuation(0.8F);
		blinnPhongLight.initLight();
		roomShape.initBuffer();
		boxShape.initBuffer();
		sphereShape.initBuffer();
		coinShape.initBuffer();
		fontShape.initBuffer();
		fontShape.setTexture("font_texture.png", 15, 5, 25, 40, 25, 11, 512);
		fontShape.setPosition(-1, 1, -1);
		fontShape.setSize(1f / 8);
		fontShape.setColor(0, 1, 0, 1);
		fontShape.setText("FPS:", 0);

		GL11.glEnable(GL11.GL_CULL_FACE);
		GL11.glCullFace(GL11.GL_BACK);
		GL11.glEnable(GL11.GL_DEPTH_TEST);
		GL11.glClearDepth(GL11.GL_LEQUAL);
		GL11.glClearColor(0.0f, 0.5f, 0.9f, 1.0f);
		GL11.glBlendFunc(GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA);
	}

	public void picking(Vector3f resultStartPoint, Vector3f resultDir,
			float mouseX, float mouseY, float screenWidth, float screenHeight) {
		final float x = (mouseX / screenWidth - 0.5F) * 2;
		final float y = (mouseY / screenHeight - 0.5F) * 2;
		MatrixExt.invertM(mvpTempMatrix, 0, vpMatrix, 0);
		final float v0 = mvpTempMatrix[0] * x + mvpTempMatrix[1] * y
				+ mvpTempMatrix[3];
		final float v1 = mvpTempMatrix[4] * x + mvpTempMatrix[5] * y
				+ mvpTempMatrix[7];
		final float v2 = mvpTempMatrix[8] * x + mvpTempMatrix[9] * y
				+ mvpTempMatrix[11];
		final float v3 = mvpTempMatrix[12] * x + mvpTempMatrix[13] * y
				+ mvpTempMatrix[15];
		resultStartPoint.set(v0 - mvpTempMatrix[2], v1 - mvpTempMatrix[6], v2
				- mvpTempMatrix[10]);
		resultStartPoint.scale(1F / (v3 - mvpTempMatrix[14]));
		resultDir.set(v0, v1, v2);
		resultDir.scale(1F / v3);
		resultDir.subtract(resultStartPoint);
	}

	final Vector3f fromPoint = new Vector3f();
	final Vector3f dir = new Vector3f();
	AABB selectedBody;
	List<Object> list = new ArrayList<Object>();

	@Override
	protected void update(IUpdateInfo uInfo) {
		final float dTime = uInfo.getRate();
		while (Mouse.next()) {
			if (Mouse.getEventButtonState()) {
				if (Mouse.getEventButton() == 0) {
					picking(fromPoint, dir, Mouse.getX(), Mouse.getY(), width,
							height);
					list.clear();
					world.getTree().retrieve(list, fromPoint, dir, 0,
							Float.POSITIVE_INFINITY);
					float min = Float.POSITIVE_INFINITY;
					final RaycastHit hit = RaycastHit.DEFAULT;
					AABB sB = null;
					for (final Object o : list) {
						if (o instanceof Plane) {
							final Plane p = (Plane) o;
							if (p.intersectsRay(hit, fromPoint, dir, 0, min)) {
								min = hit.getScalar();
								sB = p.getShape().getAABB();
							}
						} else if (o instanceof Collidable) {
							final AABB b = ((Collidable) o).getShape()
									.getAABB();
							if (b.getAABB().intersectsRay(hit, fromPoint, dir,
									0, min)) {
								min = hit.getScalar();
								sB = b;
							}
						}
					}
					selectedBody = selectedBody != sB ? sB : null;
				}
			}
		}
		while (Keyboard.next()) {
			if (Keyboard.getEventKeyState()) {
				if (Keyboard.getEventKey() == Keyboard.KEY_TAB) {
					if (currentActor == actor) {
						currentActor = actor2;
					} else {
						currentActor = actor;
					}
					final TrackingCamera camera = (TrackingCamera) world
							.getActiveCamera();
					qu.setMultiply(leftright, updown).norm();
					qu.rotateV(followOffsetTmp, followOffset);
					camera.followBody(currentActor, followOffsetTmp.x,
							followOffsetTmp.y, followOffsetTmp.z);
					camera.lookAtBody(currentActor.getPosition(), 0, 1.6F, 0);
					final StaticLight light = world.getActiveLight();
					light.followBody(currentActor, 0, 1.7F, -1);
				} else if (Keyboard.getEventKey() == Keyboard.KEY_F1) {
					renderbounds = !renderbounds;
				} else if (Keyboard.getEventKey() == Keyboard.KEY_F2) {
					renderSweptBounds = !renderSweptBounds;
				} else if (Keyboard.getEventKey() == Keyboard.KEY_F3) {
					renderTree = !renderTree;
				} else if (Keyboard.getEventKey() == Keyboard.KEY_F4) {
					renderRay = !renderRay;
				} else if (Keyboard.getEventKey() == Keyboard.KEY_F5) {
					renderRoom = !renderRoom;
				} else if (Keyboard.getEventKey() == Keyboard.KEY_F6) {
					renderDynamics = !renderDynamics;
				} else if (Keyboard.getEventKey() == Keyboard.KEY_F7) {
					wireframed = !wireframed;
					roomShape.drawWireframe(wireframed);
					boxShape.drawWireframe(wireframed);
					sphereShape.drawWireframe(wireframed);
					coinShape.drawWireframe(wireframed);
				} else if (Keyboard.getEventKey() == Keyboard.KEY_F8) {
					renderContacts = !renderContacts;
				} else if (Keyboard.getEventKey() == Keyboard.KEY_ESCAPE) {
					if (isPaused()) {
						resume();
					} else
						pause();
				}
			}
		}
		if (world.getActiveCamera() instanceof TrackingCamera
				&& (Keyboard.isKeyDown(Keyboard.KEY_NUMPAD2)
						|| Keyboard.isKeyDown(Keyboard.KEY_NUMPAD4)
						|| Keyboard.isKeyDown(Keyboard.KEY_NUMPAD6) || Keyboard
							.isKeyDown(Keyboard.KEY_NUMPAD8))) {
			if (Keyboard.isKeyDown(Keyboard.KEY_NUMPAD2)) {
				updown.addRotationEuler(-90 * dTime, 0, 0);
			} else if (Keyboard.isKeyDown(Keyboard.KEY_NUMPAD4)) {
				leftright.addRotationEuler(0, 90 * dTime, 0);
			} else if (Keyboard.isKeyDown(Keyboard.KEY_NUMPAD6)) {
				leftright.addRotationEuler(0, -90 * dTime, 0);
			} else if (Keyboard.isKeyDown(Keyboard.KEY_NUMPAD8)) {
				updown.addRotationEuler(90 * dTime, 0, 0);
			}
			qu.setMultiply(leftright, updown).norm();
			qu.rotateV(followOffsetTmp, followOffset);
			((TrackingCamera) world.getActiveCamera()).followBody(currentActor,
					followOffsetTmp.x, followOffsetTmp.y, followOffsetTmp.z);
		}
		if (Keyboard.isKeyDown(Keyboard.KEY_ADD)) {
			float z = world.getActiveCamera().getZoom() - 0.5f * dTime;
			world.getActiveCamera().setZoom(Math.max(0.02f, z));
		}
		if (Keyboard.isKeyDown(Keyboard.KEY_SUBTRACT)) {
			float z = world.getActiveCamera().getZoom() + 0.5f * dTime;
			world.getActiveCamera().setZoom(Math.min(1, z));
		}
		if (Keyboard.isKeyDown(Keyboard.KEY_UP)) {
			currentActor.push(uInfo);
		}
		if (Keyboard.isKeyDown(Keyboard.KEY_DOWN)) {
			currentActor.pushInverse(uInfo);
		}
		if (Keyboard.isKeyDown(Keyboard.KEY_LEFT)) {
			currentActor.rotate(0, 90 * dTime, 0);
		}
		if (Keyboard.isKeyDown(Keyboard.KEY_RIGHT)) {
			currentActor.rotate(0, -90 * dTime, 0);
		}
		if (Keyboard.isKeyDown(Keyboard.KEY_SPACE)) {
			currentActor.jump();
		}
		if (Keyboard.isKeyDown(Keyboard.KEY_D)) {
			resume();
		}
		if (!isPaused()) {
			cPSet = false;
			updates++;
			world.update(uInfo);
		}
		pause();
	}

	static Quaternion updown = new Quaternion();
	static Quaternion leftright = new Quaternion();
	static Quaternion qu = new Quaternion();
	static Vector3f followOffset = new Vector3f(0, 1.6f, 3.5f);
	static Vector3f followOffsetTmp = new Vector3f();

	public static AABB cPoint = new AABB(0.08f, 0.08f, 0.08f);
	public static AABB cPoint2 = new AABB(0.08f, 0.08f, 0.08f);
	public static Vector3f cPNormal = new Vector3f();
	public static boolean cPSet;

	boolean renderRoom = true, renderDynamics = true, wireframed = false,
			renderContacts = true;
	boolean renderbounds, renderSweptBounds, renderTree, renderRay;

	int frames = 0;
	int updates = 0;
	long lastTime = System.currentTimeMillis();

	private void fps() {
		frames++;
		long time = System.currentTimeMillis();
		long dif = time-lastTime;
		if (dif >= 1000) {
			fontShape.setText(Long.toString(dif / 1000 * frames) +" F: "+Integer.toString(updates),
					4);
			lastTime = time;
			frames = 0;
		}

	}

	@Override
	protected void onSurfaceChanged(int pWidth, int pHeight) {
		width = pWidth;
		height = pHeight;
		GL11.glViewport(0, 0, width, height);
		final Camera camera = world.getActiveCamera();
		if (camera != null) {
			final float ratio = (float) width / height;
			camera.setFrustum(-ratio, ratio, -1, 1, 0.01F, 100);
		}
	}

	@Override
	protected void render() {
		GL11.glClear(GL11.GL_DEPTH_BUFFER_BIT | GL11.GL_COLOR_BUFFER_BIT);

		final Camera camera = world.getActiveCamera();
		final StaticLight light = world.getActiveLight();

		if (camera != null) {
			viewMatrix = camera.getViewMatrix();
			projectionMatrix = camera.getProjectionMatrix();
		}
		MatrixExt.multiplyMM(vpMatrix, 0, projectionMatrix, 0, viewMatrix, 0);
		frustum.setFrustum(vpMatrix);

		blinnPhongShader.use();
		if (light != null) {
//			light.getAffineTransform().getTransformationMatrix(modelMatrix);
//			MatrixExt.multiplyMV(lightPosInEyeSpace, 0, modelMatrix, 0,
//					lightPosInModelSpace, 0);
			lightPosInEyeSpace[0]=camera.getPosition().x;
			lightPosInEyeSpace[1]=camera.getPosition().y;
			lightPosInEyeSpace[2]=camera.getPosition().z;
			lightPosInEyeSpace[3]=1;
			MatrixExt.multiplyMV(lightPosInEyeSpace, 0, viewMatrix, 0,
					lightPosInEyeSpace, 0);
			blinnPhongLight.setPosition(lightPosInEyeSpace[0],
					lightPosInEyeSpace[1], lightPosInEyeSpace[2],
					lightPosInEyeSpace[3]);
			blinnPhongLight.render();
		}

		if (roomShape != null && renderRoom) {
			MatrixExt.setIdentityM(modelMatrix, 0);
			roomShape.render(modelMatrix, viewMatrix, projectionMatrix,
					mvpTempMatrix);
		}
		world.getTree().retrieve(frustumList, frustum);
		if (renderDynamics) {
			for (final Object bounds : frustumList) {
				if (bounds instanceof StaticBody) {
					final StaticBody body = (StaticBody) bounds;
					final GLShape shape = body.getGLShape();
					if (shape instanceof MainShape) {
						((MainShape) shape).render(body.getAffineTransform());
					}
				}
			}
			coinShape.render(viewMatrix, projectionMatrix, mvpTempMatrix);
			boxShape.render(viewMatrix, projectionMatrix, mvpTempMatrix);
			sphereShape.render(viewMatrix, projectionMatrix, mvpTempMatrix);
		}
		primitiveShader.use();

		if (cPSet) {
			primitiveShape.setColor(1, 1, 0, 1);
			primitiveShape.addAABB(cPoint);
			Vector3f p = cPoint.getPosition();
			primitiveShape.addLine(p.x, p.y, p.z, p.x + cPNormal.x, p.y
					+ cPNormal.y, p.z + cPNormal.z);
			primitiveShape.render(vpMatrix);
		}
		if (renderContacts) {
			Collection<Manifold> manifolds = Manifold.manifolds.values();
			for (Manifold m : manifolds) {
				for (int i = 0; i < m.size(); i++) {
					ManifoldContact c = m.getContact(i);
					primitiveShape.setColor(1, 1f, 0, 1);
					cPoint2.setPosition(c.getWorldA());
					primitiveShape.addAABB(cPoint2);
					Vector3f p = cPoint2.getPosition();
					Vector3f normal = c.getNormal();
					primitiveShape.addLine(p.x, p.y, p.z, p.x + normal.x, p.y
							+ normal.y, p.z + normal.z);
					primitiveShape.render(vpMatrix);

					cPoint2.setPosition(c.getWorldB());
					primitiveShape.addAABB(cPoint2);
					p = cPoint2.getPosition();
					normal = c.getNormal();
					primitiveShape.addLine(p.x, p.y, p.z, p.x + normal.x, p.y
							+ normal.y, p.z + normal.z);
					primitiveShape.render(vpMatrix);
				}
			}
		}

		if (selectedBody != null) {
			primitiveShape.setColor(0, 1, 0, 1);
			primitiveShape.addAABB(selectedBody.getAABB());
			primitiveShape.render(vpMatrix);
		}
		if (renderbounds) {
			primitiveShape.setColor(1, 0, 0, 1);
			for (final Object o : frustumList) {
				if (o instanceof Collidable) {
					primitiveShape.addAABB(((Collidable) o).getShape()
							.getAABB());
				}
			}
			primitiveShape.render(vpMatrix);
		}

		if (renderSweptBounds) {
			primitiveShape.setColor(1, 0, 1, 1);
			for (final Object o : frustumList) {
				if (o instanceof DynamicBody) {
					primitiveShape.addAABB(((DynamicBody) o).getSweptAABB());
				}
			}
			primitiveShape.render(vpMatrix);
		}
		frustumList.clear();
		if (renderTree) {
			final Octree tree = (Octree) world.getTree();
			primitiveShape.setColor(1, 0, 0, 1);
			tree.draw(primitiveShape, currentActor.getShape().getAABB());
			primitiveShape.render(vpMatrix);

			primitiveShape.setColor(0, 0, 1, 1);
			tree.draw(primitiveShape, frustum);
			primitiveShape.render(vpMatrix);
		}

		if (renderRay) {
			primitiveShape.setColor(0, 0, 1, 1);
			primitiveShape.addLine(fromPoint.x, fromPoint.y, fromPoint.z,
					fromPoint.x + dir.x * 1000, fromPoint.y + dir.y * 100000,
					fromPoint.z + dir.z * 1000);
			primitiveShape.render(vpMatrix);
		}
		fontShader.use();
		// GL11.glDepthMask(false);
		GL11.glEnable(GL11.GL_BLEND);
		fontShape.render();
		GL11.glDisable(GL11.GL_BLEND);
		// GL11.glDepthMask(true);
		fps();
	}

	public static void main(String... args) {
		new MyGLSurfaceView().start();
	}
}
