package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.utils.Distance;
import com.brocorporation.gameengine.utils.Vector3f;

public class MPR {

	protected final static int MAX_ITERATIONS = 34;
	protected final static float CollideEpsilon = 1e-4f;
	protected final static float CollideEpsilon_2 = CollideEpsilon
			* CollideEpsilon;

	protected final static Vector3f temp1 = new Vector3f();
	protected final static Vector3f temp2 = new Vector3f();
	protected final static Vector3f temp3 = new Vector3f();

	public static boolean intersects(IShape shape1, IShape shape2) {
		final int res = discoverPortal(shape1, shape2);
		if (res < 0)
			return false;
		if (res > 0)
			return true;
		return refinePortal(shape1, shape2);
	}

	public static boolean intersects(Contact contact, IShape shape1,
			IShape shape2) {
		bu.setLength(0);
		Vector3f[] v1 = ((Convex) shape1).getVertices();
		Vector3f[] v2 = ((Convex) shape2).getVertices();
		for (int i = 0; i < v1.length; i++) {
			for (int j = 0; j < v2.length; j++) {
				temp1.setSubtract(v1[i], v2[j]);
				bu.append("p" + i + "" + j + ":Punkt(" + temp1.toString().replaceAll("E", "*10^")+")\n");
			}
		}
		bu.append("\n\n");
		final int res = discoverPortal(shape1, shape2);
		if (res < 0) {
			return false;
		} else if (res == 1) {
			findPenetrationTouch(contact, shape1, shape2);
		} else if (res == 2) {
			findPenetrationSegment(contact);
		} else if (res == 0) {
			if (!refinePortal(shape1, shape2)) {
				return false;
			}
			findPenetration(contact, shape1, shape2);

		}
		return true;
	}
	
	public static boolean getPenetration(Contact contact, IShape shape1, IShape shape2) {
		bu.setLength(0);
		Vector3f[] v1 = ((Convex) shape1).getVertices();
		Vector3f[] v2 = ((Convex) shape2).getVertices();
		for (int i = 0; i < v1.length; i++) {
			for (int j = 0; j < v2.length; j++) {
				temp1.setSubtract(v1[i], v2[j]);
				bu.append("p" + i + "" + j + ":Punkt(" + temp1.toString().replaceAll("E", "*10^")+")\n");
			}
		}
		bu.append("\n\n");
		final int res = discoverPortal(shape1, shape2);
		if (res < 0) {
			contact.setDistance(0);
			contact.getNormal().set(0,0,0);
			return false;
		} else if (res == 1) {
			findPenetrationTouch(contact, shape1, shape2);
		} else if (res == 2) {
			findPenetrationSegment(contact);
		} else if (res == 0) {
//			if (!refinePortal(shape1, shape2)) {
//				contact.setDistance(0);
//				contact.getNormal().set(0,0,0);
//				return false;
//			}
//			findPenetrationS(contact, shape1, shape2);
			findPenetration(contact,shape1,shape2);
		}
		return true;
	}

	public static class Element {
		public Vector3f v = new Vector3f();
		public Vector3f pA = new Vector3f();
		public Vector3f pB = new Vector3f();
	}

	static Element e0 = new Element();
	static Element e1 = new Element();
	static Element e2 = new Element();
	static Element e3 = new Element();
	static Element e4 = new Element();

	static Vector3f dir = new Vector3f();

	protected static byte discoverPortal(IShape shape1, IShape shape2) {
		e0.v.setSubtract(e0.pA.set(shape1.getPosition()),
				e0.pB.set(shape2.getPosition()));
		bu.append("discover\n");
		bu.append("v0:Punkt(" + e0.v + ")\n");
		if (e0.v.isZero()) {
			e0.v.set(0.00001f, 0, 0);
		}
		minSupport(e1, shape1, shape2, e0.v);
		if (e1.v.dot(e0.v) >= 0)
			return -1;
		dir.setCross(e0.v, e1.v);
		if (dir.isZero()) {
			return 2;
		}
		maxSupport(e2, shape1, shape2, dir);
		if (e2.v.dot(dir) <= 0)
			return -1;
		temp1.setSubtract(e1.v, e0.v);
		temp2.setSubtract(e2.v, e0.v);
		dir.setCross(temp1, temp2);
		if (dir.dot(e0.v) > 0) {
			final Element e = e1;
			e1 = e2;
			e2 = e;
			dir.invert();
			bu.append("swap 1<->2\n");
		}
		while (true) {
			maxSupport(e3, shape1, shape2, dir);
			bu.append("v1:Punkt(" + e1.v + ")\n");
			bu.append("v2:Punkt(" + e2.v + ")\n");
			bu.append("v3:Punkt(" + e3.v + ")\n");
			bu.append("d : " + dir + "\n\n");
			if (e3.v.dot(dir) <= 0)
				return -1;
			if (temp1.setCross(e1.v, e3.v).dot(e0.v) < 0) {
				set(e2, e3);
			} else if (temp1.setCross(e3.v, e2.v).dot(e0.v) < 0) {
				set(e1, e3);
			} else {
				return 0;
			}
			temp1.setSubtract(e1.v, e0.v);
			temp2.setSubtract(e2.v, e0.v);
			dir.setCross(temp1, temp2);
		}
	}

	protected static boolean refinePortal(IShape shape1, IShape shape2) {
		bu.append("refine\n");
		while (true) {
			bu.append("v1:Punkt(" + e1.v + ")\n");
			bu.append("v2:Punkt(" + e2.v + ")\n");
			bu.append("v3:Punkt(" + e3.v + ")\n");
			portalDir(dir);
			if (dir.dot(e1.v) >= 0)
				return true;
			maxSupport(e4, shape1, shape2, dir);
			bu.append("v4:Punkt(" + e4.v + ")\n");
			if (dir.dot(e4.v) < 0 || portalReachTolerance(e4, dir)) {
				return false;
			}
			expandPortal(e4);
		}
	}

	static Vector3f zero = new Vector3f();

	public static StringBuilder bu = new StringBuilder();

	protected static void findPenetration(Contact contact, IShape shape1,
			IShape shape2) {
		bu.append("penetration\n");
		int iterations = 0;
		while (true) {
			portalDir(dir);
			maxSupport(e4, shape1, shape2, dir);
			bu.append("v1:Punkt(" + e1.v + ")\n");
			bu.append("v2:Punkt(" + e2.v + ")\n");
			bu.append("v3:Punkt(" + e3.v + ")\n");
			bu.append("v4:Punkt(" + e4.v + ")\n");
			bu.append("d : " + dir + "\n");
			if (portalReachTolerance(e4, dir) || iterations > MAX_ITERATIONS) {
				break;
			}
			
			temp3.setCross(e4.v, e0.v);
			bu.append("expand: "+e1.v.dot(temp3) + "\t" + e2.v.dot(temp3) + "\t"
					+ e3.v.dot(temp3) + "\n");
			
			
			v4v1.setSubtract(e4.v, e1.v);
			v4v2.setSubtract(e4.v, e2.v);
			v4v3.setSubtract(e4.v, e3.v);
			temp1.setCross(v4v1, v4v2);
			boolean a = Distance.intersectsLineTriangle(zero, dir, temp1, e4.v, e1.v, e2.v);
			temp1.setCross(v4v1, v4v3);
			boolean b = Distance.intersectsLineTriangle(zero, dir, temp1, e4.v, e1.v, e3.v);
			temp1.setCross(v4v2, v4v3);
			boolean c = Distance.intersectsLineTriangle(zero, dir, temp1, e4.v, e2.v, e3.v);
			if(a){
				set(e3,e4);
			} else if(b){
				set(e2,e4);
			} else if(c){
				set(e1,e4);
			}//ELSE search in dir of edge?¿?
			bu.append(a + "\t" + b + "\t"+c + "\n");
			
			
			
			//expandPortal(e4);
			iterations++;
			bu.append("\n");
		}
		final Vector3f n = contact.getNormal();
		//float d = Distance.distanceToPlane(n, e1.v, e2.v, e3.v);
		n.setNorm(dir);
		float d = e1.v.dot(n);
		if (d <= CollideEpsilon/*_2*/) {
			findPenetrationTouch(contact, shape1, shape2);
		} else {
			//d = (float) Math.sqrt(d);
			contact.setDistance(-d);
			//n.norm();
			findPos(contact.getPointA(), shape1, shape2, n);

			bu.append("n : " + n + "\t" + d + "\n");
			bu.append(iterations
					+ "____________________________________\n");
			if (d > 0.1 || iterations>5)
				System.out.println(bu);
		}
		bu.setLength(0);
		return;
	}
	
	static Vector3f v4v1 = new Vector3f();
	static Vector3f v4v2 = new Vector3f();
	static Vector3f v4v3 = new Vector3f();

	protected static void findPenetrationS(Contact c, IShape shape1,
			IShape shape2) {
		int iterations = 0;
		while (true) {
			portalDir(dir);
			maxSupport(e4, shape1, shape2, dir);
			if (portalReachTolerance(e4, dir) || iterations > MAX_ITERATIONS) {
				final Vector3f n = c.getNormal();
				float d = Distance.distanceToTriangle(n, e1.v, e2.v, e3.v);
				if (d <= CollideEpsilon_2) {
					findPenetrationTouch(c, shape1, shape2);
				} else {
					d = (float) Math.sqrt(d);
					c.setDistance(-d);
					n.norm();
					findPos(c.getPointA(), shape1, shape2, n);
				}
				return;
			}
			expandPortal(e4);
			iterations++;
		}
	}

	protected static void findPenetrationTouch(Contact c, IShape shape1,
			IShape shape2) {
		c.setDistance(0);
		c.getNormal().set(0, 0, 0);
		c.getPointA().setAdd(e1.pA, e1.pB).scale(0.5f);
	}

	protected static void findPenetrationSegment(Contact c) {
		c.getPointA().setAdd(e1.pA, e1.pB).scale(0.5f);
		c.getNormal().set(e1.v);
		c.setDistance(-c.getNormal().normLength());
	}

	protected static void findPos(Vector3f pos, IShape shape1, IShape shape2,
			Vector3f normedNormal) {
		float b0 = temp1.setCross(e1.v, e2.v).dot(e3.v);
		float b1 = temp2.setCross(e3.v, e2.v).dot(e0.v);
		float b2 = temp3.setCross(e0.v, e1.v).dot(e3.v);
		float b3 = -temp1.dot(e0.v);

		float sum = b0 + b1 + b2 + b3;

		if (sum <= 0) {
			b1 = -temp2.dot(normedNormal);
			b2 = temp3.setCross(e3.v, e1.v).dot(normedNormal);
			b3 = temp1.dot(normedNormal);
			sum = b1 + b2 + b3;
			final float invHalfSum = 0.5f / sum;
			pos.set(((e1.pA.x + e1.pB.x) * b1 + (e2.pA.x + e2.pB.x) * b2 + (e3.pA.x + e3.pB.x)
					* b3)
					* invHalfSum, ((e1.pA.y + e1.pB.y) * b1
					+ (e2.pA.y + e2.pB.y) * b2 + (e3.pA.y + e3.pB.y) * b3)
					* invHalfSum, ((e1.pA.z + e1.pB.z) * b1
					+ (e2.pA.z + e2.pB.z) * b2 + (e3.pA.z + e3.pB.z) * b3)
					* invHalfSum);
		} else {
			final float invHalfSum = 0.5f / sum;
			pos.set(((e0.pA.x + e0.pB.x) * b0 + (e1.pA.x + e1.pB.x) * b1
					+ (e2.pA.x + e2.pB.x) * b2 + (e3.pA.x + e3.pB.x) * b3)
					* invHalfSum,
					((e0.pA.y + e0.pB.y) * b0 + (e1.pA.y + e1.pB.y) * b1
							+ (e2.pA.y + e2.pB.y) * b2 + (e3.pA.y + e3.pB.y)
							* b3)
							* invHalfSum, ((e0.pA.z + e0.pB.z) * b0
							+ (e1.pA.z + e1.pB.z) * b1 + (e2.pA.z + e2.pB.z)
							* b2 + (e3.pA.z + e3.pB.z) * b3)
							* invHalfSum);
		}
	}

	protected static void expandPortal(Element e) {
		temp3.setCross(e.v, e0.v);
		bu.append("expand: "+e1.v.dot(temp3) + "\t" + e2.v.dot(temp3) + "\t"
				+ e3.v.dot(temp3) + "\n");
		if (e1.v.dot(temp3) > 0) {
			if (e2.v.dot(temp3) > 0) {
				set(e1, e);
			} else {
				set(e3, e);
			}
		} else {
			if (e3.v.dot(temp3) > 0) {
				set(e2, e);
			} else {
				set(e1, e);
			}
		}
	}
	
	
	protected static void expandPortalS(Element e) {
		temp3.setCross(e.v, e0.v);
		if (e1.v.dot(temp3) > 0) {
			if (e2.v.dot(temp3) > 0) {
				set(e1, e);
			} else {
				set(e3, e);
			}
		} else {
			if (e3.v.dot(temp3) > 0) {
				set(e2, e);
			} else {
				set(e1, e);
			}
		}
	}

	protected static void portalDir(Vector3f dir) {
		temp1.setSubtract(e2.v, e1.v);
		temp2.setSubtract(e3.v, e1.v);
		dir.setCross(temp1, temp2);
	}

	protected static boolean portalReachTolerance(Element e, Vector3f dir) {
		final float min = temp2.setSubtract(e.v, e1.v).dot(dir);
		return min <= 0 || min * min <= CollideEpsilon_2 * dir.dot(dir);
	}

	protected static void minSupport(Element e, IShape shape1, IShape shape2,
			Vector3f dir) {
		shape1.getMinAlongDirection(e.pA, dir);
		shape2.getMaxAlongDirection(e.pB, dir);
		e.v.setSubtract(e.pA, e.pB);
	}

	protected static void maxSupport(Element e, IShape shape1, IShape shape2,
			Vector3f dir) {
		shape1.getMaxAlongDirection(e.pA, dir);
		shape2.getMinAlongDirection(e.pB, dir);
		e.v.setSubtract(e.pA, e.pB);
	}

	protected static void set(Element a, Element b) {
		a.v.set(b.v);
		a.pA.set(b.pA);
		a.pB.set(b.pB);
	}
}
