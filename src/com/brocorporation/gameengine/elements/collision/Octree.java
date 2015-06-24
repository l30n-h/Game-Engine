package com.brocorporation.gameengine.elements.collision;

import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.brocorporation.gameengine.elements.opengl.PrimitiveShape;
import com.brocorporation.gameengine.utils.Vector3f;

public class Octree extends Tree {

	protected final static byte SUBDIVISIONS = 8;
	protected final static byte MAX_LEVELS = 5;
	protected final static int MAX_OBJECTS = 10;

	protected HashMap<AABB, Node> objectNodeMap = new HashMap<AABB, Node>();
	protected final Node root;

	public Octree(final float pX, final float pY, final float pZ,
			final float size) {
		root = new Node(pX, pY, pZ, size);
	}

	public void draw(PrimitiveShape shape, Frustum frustum) {
		root.draw(shape, frustum);
	}

	@Override
	public void optimize() {
		root.shouldBeEnabled();
	}

	public void draw(PrimitiveShape shape, AABB pBounds) {
		final Node node = objectNodeMap.get(pBounds);
		if (node != null) {
			shape.addAABB(node);
		}
	}

	@Override
	public boolean stillInside(AABB pBounds) {
		final Node node = objectNodeMap.get(pBounds);
		if (node != null) {
			return node.contains(pBounds.getPosition(), pBounds.getHalfsize());
		}
		return false;
	}

	@Override
	public void hasMoved(AABB pBounds) {
		final Node node = objectNodeMap.get(pBounds);
		if (node != null) {
			if (node.contains(pBounds.getPosition(), pBounds.getHalfsize())) {
				node.insertDeeper(pBounds);
			} else {
				Node parentNode = node.parent;
				if (parentNode != null) {
					final Object o = node.remove(pBounds);
					final Vector3f pos = pBounds.getPosition();
					final Vector3f half = pBounds.getHalfsize();
					while (parentNode.parent != null
							&& !parentNode.contains(pos, half)) {
						parentNode = parentNode.parent;
					}
					parentNode.add(pBounds, o);
				}
			}
		}
	}

	@Override
	public void add(AABB pBounds, Object o) {
		root.add(pBounds, o);
	}

	@Override
	public void remove(AABB pBounds) {
		final Node node = objectNodeMap.get(pBounds);
		if (node != null) {
			node.remove(pBounds);
		}
	}

	@Override
	public List<Object> retrieve(List<Object> returnObjects, Frustum frustum) {
		return root.retrieve(returnObjects, frustum);
	}

	@Override
	public List<Object> retrieve(List<Object> returnObjects, Vector3f pFrom,
			Vector3f pDir, float min, float max) {
		return root.retrieve(returnObjects, pFrom, pDir, min, max);
	}

	@Override
	public List<Object> retrieve(List<Object> returnObjects, Vector3f pFrom,
			Vector3f pDir, Vector3f halfsizeOffset, float min, float max) {
		return root.retrieve(returnObjects, pFrom, halfsizeOffset, pDir, min,
				max);
	}

	@Override
	public List<Object> retrieve(List<Object> returnObjects,
			Vector3f pPosition, Vector3f pHalfsize) {
		return root.retrieve(returnObjects, pPosition, pHalfsize);
	}

	@Override
	public List<Object> retrieve(List<Object> returnObjects, AABB pBounds) {
		final Node node = objectNodeMap.get(pBounds);
		if (node != null) {
			return node.retrieveWithParent(returnObjects,
					pBounds.getPosition(), pBounds.getHalfsize());
		}
		return root.retrieve(returnObjects, pBounds.getPosition(),
				pBounds.getHalfsize());
	}

	@Override
	public List<Object> retrieveAll(List<Object> returnObjects) {
		return root.retrieveAll(returnObjects);
	}

	protected class Node extends AABB {
		protected final byte level;
		protected final HashMap<AABB, Object> objects;
		protected final Node parent;
		protected Node[] children;
		protected boolean enabled;

		protected Node(final float pX, final float pY, final float pZ,
				final float pHalfsize) {
			parent = null;
			level = 0;
			objects = new HashMap<AABB, Object>();
			position.set(pX, pY, pZ);
			halfsize.set(pHalfsize, pHalfsize, pHalfsize);
		}

		protected Node(final Node pParent, final float pX, final float pY,
				final float pZ, final float pHalfsize, final byte pLevel) {
			parent = pParent;
			level = pLevel;
			objects = new HashMap<AABB, Object>(MAX_OBJECTS + 1, 1);
			position.set(pX, pY, pZ);
			halfsize.set(pHalfsize, pHalfsize, pHalfsize);
		}

		protected void split() {
			final float hs = halfsize.x / 2;
			final float px = position.x + hs;
			final float py = position.y + hs;
			final float pz = position.z + hs;
			final float mx = position.x - hs;
			final float my = position.y - hs;
			final float mz = position.z - hs;
			final byte l = (byte) (level + 1);
			children = new Node[SUBDIVISIONS];
			children[0] = new Node(this, mx, py, mz, hs, l);
			children[1] = new Node(this, px, py, mz, hs, l);
			children[2] = new Node(this, mx, py, pz, hs, l);
			children[3] = new Node(this, px, py, pz, hs, l);
			children[4] = new Node(this, mx, my, mz, hs, l);
			children[5] = new Node(this, px, my, mz, hs, l);
			children[6] = new Node(this, mx, my, pz, hs, l);
			children[7] = new Node(this, px, my, pz, hs, l);
		}

		protected byte getIndex(final AABB pBounds) {
			if (children != null) {
				final Vector3f p = pBounds.getPosition();
				if (p.x != position.x && p.y != position.y && p.z != position.z) {
					final byte index = (byte) ((p.x < position.x ? 0 : 1)
							+ (p.y > position.y ? 0 : 4) + (p.z < position.z ? 0
							: 2));
					if (children[index].contains(p, pBounds.getHalfsize())) {
						return index;
					}
				}
			}
			return -1;
		}

		protected boolean contains(final Vector3f pPosition,
				final Vector3f pHalfsize) {
			return AABB.contains(position, halfsize, pPosition, pHalfsize);
		}

		protected void insertDeeper(final AABB pBounds) {
			final byte index = getIndex(pBounds);
			if (index != -1) {
				final Object o = objects.remove(pBounds);
				children[index].add(pBounds, o);
			}
		}

		protected boolean isEmpty() {
			return objects.isEmpty();
		}

		protected void add(final AABB pBounds, final Object o) {
			byte index = getIndex(pBounds);
			if (index != -1) {
				children[index].add(pBounds, o);
				return;
			}
			objectNodeMap.put(pBounds, this);
			objects.put(pBounds, o);
			if (level < MAX_LEVELS && objects.size() > MAX_OBJECTS) {
				if (children == null) {
					split();
				}
				final Iterator<Map.Entry<AABB, Object>> it = objects.entrySet()
						.iterator();
				while (it.hasNext()) {
					final Map.Entry<AABB, Object> entry = it.next();
					final AABB bound = entry.getKey();
					index = getIndex(bound);
					if (index != -1) {
						it.remove();
						children[index].add(bound, entry.getValue());
					}
				}
			}
		}

		protected Object remove(final AABB pBounds) {
			objectNodeMap.remove(pBounds);
			return objects.remove(pBounds);
		}

		protected boolean shouldBeEnabled() {
			boolean childEnabled = false;
			if (children != null) {
				for (byte i = 0; i < SUBDIVISIONS; i++) {
					if (children[i].shouldBeEnabled()) {
						childEnabled = true;
					}
				}
			}
			if (isEmpty()) {
				hasChanged = enabled != childEnabled;
				enabled = childEnabled;
			} else {
				if (!enabled) {
					enabled = true;
					hasChanged = true;
				} else
					hasChanged = false;
			}
			return enabled;
		}

		protected List<Object> retrieve(List<Object> returnObjects,
				Frustum frustum) {
			if (enabled) {
				final byte a = frustum.containsIntersects(this);
				if (a == Frustum.CONTAINS) {
					returnObjects.addAll(objects.values());
					if (children != null) {
						for (byte i = 0; i < SUBDIVISIONS; i++) {
							children[i].retrieveAll(returnObjects);
						}
					}
				} else if (a == Frustum.INTERSECTS) {
					final Set<Map.Entry<AABB, Object>> set = objects.entrySet();
					for (final Map.Entry<AABB, Object> bounds : set) {
						if (frustum.intersects(bounds.getKey())) {
							returnObjects.add(bounds.getValue());
						}
					}
					if (children != null) {
						for (byte i = 0; i < SUBDIVISIONS; i++) {
							children[i].retrieve(returnObjects, frustum);
						}
					}
				}
			}
			return returnObjects;
		}

		protected List<Object> retrieve(List<Object> returnObjects,
				Vector3f pFrom, Vector3f pDir, float min, float max) {
			if (enabled
					&& AABB.intersectsRay(RaycastHit.DEFAULT, position,
							halfsize, pFrom, pDir, min, max)) {
				if (children != null) {
					for (byte i = 0; i < SUBDIVISIONS; i++) {
						children[i].retrieve(returnObjects, pFrom, pDir, min,
								max);
					}
				}
				returnObjects.addAll(objects.values());
			}
			return returnObjects;
		}

		protected List<Object> retrieve(List<Object> returnObjects,
				Vector3f pFrom, Vector3f pDir, Vector3f halfsizeOffset,
				float min, float max) {
			if (enabled
					&& AABB.intersectsRay(RaycastHit.DEFAULT, position,
							halfsize, halfsizeOffset, pFrom, pDir, min, max)) {
				if (children != null) {
					for (byte i = 0; i < SUBDIVISIONS; i++) {
						children[i].retrieve(returnObjects, pFrom, pDir,
								halfsizeOffset, min, max);
					}
				}
				returnObjects.addAll(objects.values());
			}
			return returnObjects;
		}

		protected List<Object> retrieveWithParent(List<Object> returnObjects,
				final Vector3f pPosition, final Vector3f pHalfsize) {
			if (enabled
					&& AABB.intersects(position, halfsize, pPosition, pHalfsize)) {
				Node parentNode = parent;
				while (parentNode != null) {
					returnObjects.addAll(parentNode.objects.values());
					parentNode = parentNode.parent;
				}
				if (children != null) {
					for (byte i = 0; i < SUBDIVISIONS; i++) {
						children[i].retrieve(returnObjects, pPosition,
								pHalfsize);
					}
				}
				returnObjects.addAll(objects.values());
			}
			return returnObjects;
		}

		protected List<Object> retrieve(List<Object> returnObjects,
				final Vector3f pPosition, final Vector3f pHalfsize) {
			if (enabled
					&& AABB.intersects(position, halfsize, pPosition, pHalfsize)) {
				if (children != null) {
					for (byte i = 0; i < SUBDIVISIONS; i++) {
						children[i].retrieve(returnObjects, pPosition,
								pHalfsize);
					}
				}
				returnObjects.addAll(objects.values());
			}
			return returnObjects;
		}

		protected List<Object> retrieveAll(List<Object> returnObjects) {
			if (enabled) {
				if (children != null) {
					for (byte i = 0; i < SUBDIVISIONS; i++) {
						children[i].retrieveAll(returnObjects);
					}
				}
				returnObjects.addAll(objects.values());
			}
			return returnObjects;
		}

		public void draw(PrimitiveShape shape, Frustum frustum) {
			if (frustum.intersects(this)) {
				if (children != null) {
					for (byte i = 0; i < SUBDIVISIONS; i++) {
						children[i].draw(shape, frustum);
					}
				}
				if (enabled) {
					shape.addAABB(this);
				}
			}
		}
	}
}
