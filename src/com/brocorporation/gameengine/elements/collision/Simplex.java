package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.utils.Vector3f;

public class Simplex {

	public final static Simplex DEFAULT = new Simplex();

	protected Element[] elements = { new Element(), new Element(),
			new Element(), new Element() };
	protected byte size = 0;
	protected Vector3f ref;

	public void setRef(Vector3f v) {
		ref = v;
	}

	public byte size() {
		return size;
	}

	public void add(Vector3f v) {
		if (size < 4) {
			elements[size].v.set(v);
			size++;
		}
	}

	public void add(Vector3f v, Vector3f s1, Vector3f s2) {
		if (size < 4) {
			final Element e = elements[size];
			e.v.set(v);
			e.pA.set(s1);
			e.pB.set(s2);
			size++;
		}
	}

	public boolean contains(Vector3f v) {
		for (int i = 0; i < size; i++) {
			if (elements[i].v.equals(v)) {
				return true;
			}
		}
		return false;
	}

	public Vector3f getV(int i) {
		if (0 <= i && i < size) {
			if (ref == null) {
				return elements[i].v;
			} else {
				final Element e = elements[i];
				return e.ref.setSubtract(ref, e.v);
			}
		}
		return null;
	}

	public Element get(int i) {
		if (0 <= i && i < size) {
			return elements[i];
		}
		return null;
	}

	public Element getNew() {
		if (size < 4) {
			return elements[size++];
		}
		return null;
	}

	public void swap(int a, int b) {
		if (a < size && b < size && a >= 0 && b >= 0) {
			final Element e = elements[a];
			elements[a] = elements[b];
			elements[b] = e;
		}
	}

	public void removeA() {
		remove(size - 1);
	}

	public void removeB() {
		remove(size - 2);
	}

	public void removeC() {
		remove(size - 3);
	}

	public void removeD() {
		remove(0);
	}

	public void remove(int i) {
		if (i < size && 0 <= i) {
			if (--size > i) {
				final Element ei = elements[i];
				while (i < size) {
					elements[i] = elements[++i];
				}
				elements[i] = ei;
			}
		}
	}

	public void clear() {
		size = 0;
		ref = null;
	}

	public class Element {
		public Vector3f v = new Vector3f();
		public Vector3f ref = new Vector3f();
		public Vector3f pA = new Vector3f();
		public Vector3f pB = new Vector3f();

		public Vector3f getV() {
			return v;
		}

		public Vector3f getPointA() {
			return pA;
		}

		public Vector3f getPointB() {
			return pB;
		}

		public Vector3f getRef() {
			return ref;
		}
	}
}
