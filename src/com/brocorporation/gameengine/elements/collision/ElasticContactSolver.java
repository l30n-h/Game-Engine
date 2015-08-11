package com.brocorporation.gameengine.elements.collision;

import java.util.ArrayDeque;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.bodies.DynamicBody;
import com.brocorporation.gameengine.elements.bodies.StaticBody;

public class ElasticContactSolver {

	private final static ArrayDeque<ElasticContact> useStack = new ArrayDeque<ElasticContact>();
	private final static ArrayDeque<ElasticContact> unusedStack = new ArrayDeque<ElasticContact>();

	public static ElasticContact addContact(final StaticBody stcBody,
			final DynamicBody dynBody, final Contact contact) {
		try {
			final ElasticContact c;
			if (unusedStack.isEmpty()) {
				c = new ElasticContact(stcBody, dynBody, contact);
			} else {
				c = unusedStack.pop();
				c.reset(stcBody, dynBody, contact);
			}
			useStack.push(c);
			return c;
		} catch (Exception e) {
		}
		return null;
	}

	public static void run(final IUpdateInfo uInfo) {
		if (!useStack.isEmpty()) {
			for (final ElasticContact c : useStack) {
				c.prepare(uInfo);
			}
			for (int i = 1; i < 10; i++) {
				for (final ElasticContact c : useStack) {
					c.solve(uInfo);
				}
			}
			while (!useStack.isEmpty()) {
				final ElasticContact c;
				if ((c = useStack.pop()) != null) {
					c.reset();
					unusedStack.push(c);
				}
			}
		}
	}

	public static void releaseAll() {
		while (!useStack.isEmpty()) {
			final ElasticContact c;
			if ((c = useStack.pop()) != null) {
				c.reset();
				unusedStack.push(c);
			}
		}
	}
}
