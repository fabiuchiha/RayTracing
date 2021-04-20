#include "rayAccelerator.h"
#include "macros.h"
#include <algorithm>
#include <cassert>

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
			//this->n_objs = n_objs_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) {		
	BVHNode *root = new BVHNode();
	for (Object* obj : objs) {
		// copy planes to their own array, they don't work with AABBs
		Plane* p = dynamic_cast<Plane*>(obj);
		if (p != nullptr) {
			planes.push_back(p);
		} else {
			objects.push_back(obj);
		}
	}
	nodes.push_back(root);
	build_recursive(0, objects.size(), root); // -> root node takes all the
}

void BVH::build_recursive(int left_index, int right_index, BVHNode *node) {
	// set AABB of node
	AABB aabb = objects[left_index]->GetBoundingBox();
	for (int i = left_index+1; i < right_index; i++) {
		aabb.extend(objects[i]->GetBoundingBox());
	}
	aabb.min.x -= EPSILON; aabb.min.y -= EPSILON; aabb.min.z -= EPSILON;
	aabb.max.x += EPSILON; aabb.max.y += EPSILON; aabb.max.z += EPSILON;
	node->setAABB(aabb);

	int num_objs = right_index - left_index;

	// recursion stop test
	if (num_objs <= Threshold) {
		assert(num_objs > 0);
		node->makeLeaf(left_index, num_objs);
		return;
	}

	// check which axis has the largest range of centroids
	float mins[3] = {  FLT_MAX };
	float maxs[3] = { -FLT_MAX };
	for (int i = left_index; i < right_index; i++) {
		Object* obj = objects[i];
		AABB bbox = obj->GetBoundingBox();
		float centroid[3] = { 
			bbox.centroid().x,
			bbox.centroid().y,
			bbox.centroid().z,
		};

		for (int d = 0; d < 3; d++) {
			if (centroid[d] < mins[d]) {
				mins[d] = centroid[d];
			}

			if (centroid[d] > maxs[d]) {
				maxs[d] = centroid[d];
			}
		}
	}
	float diffs[3];
	int max_dim = 0;
	for (int d = 0; d < 3; d++) {
		diffs[d] = maxs[d] - mins[d];
		if (diffs[d] > diffs[max_dim]) {
			max_dim = d;
		}
	}

	// sort objects along this axis
	Comparator cmp;
	cmp.dimension = max_dim;
	std::sort(objects.begin() + left_index, objects.begin() + right_index, cmp);

	// find a good splitting point.
	// first iterate through the nodes until we are approx in the middle of the split axis in space
	float middle = mins[max_dim] + diffs[max_dim] / 2.0f;
	int split_idx = left_index;
	for (int i = left_index; i < right_index; i++) {
		Object* obj = objects[i];
		AABB bbox = obj->GetBoundingBox();
		float centroid[3] = { 
			bbox.centroid().x,
			bbox.centroid().y,
			bbox.centroid().z,
		};

		if (middle < centroid[max_dim]) {
			split_idx = i;
			break;
		}
	}

	// if one side contains no nodes, revert to median splitting
	if (split_idx == left_index || split_idx == right_index) {
		split_idx = left_index + num_objs / 2;
	}

	// set this node to be non-leaf
	node->makeNode(nodes.size());

	// allocate new nodes
	BVHNode *left  = new BVHNode();
	BVHNode *right = new BVHNode();
	nodes.push_back(left);
	nodes.push_back(right);

	// recursivley init the new nodes
	assert(split_idx >= left_index && split_idx <= right_index);
	build_recursive(left_index, split_idx, left);
	build_recursive(split_idx, right_index, right);

	// sanity checks
	assert(node->getAABB().includes(left->getAABB()));
	if (left->isLeaf()) {
		assert(left->getNObjs() > 0);
		for (int i = left->getIndex(); i < left->getIndex()+left->getNObjs(); i++) {
			assert(left->getAABB().includes(objects[i]->GetBoundingBox()));
		}
	}
	assert(node->getAABB().includes(right->getAABB()));
	if (right->isLeaf()) {
		assert(right->getNObjs() > 0);
		for (int i = right->getIndex(); i < right->getIndex()+right->getNObjs(); i++) {
			assert(right->getAABB().includes(objects[i]->GetBoundingBox()));
		}
	}

	//right_index, left_index and split_index refer to the indices in the objects vector
	// do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector. 
	// node.index can have a index of objects vector or a index of nodes vector		
}

bool BVH::traverse_recursive(BVHNode* currentNode, Ray& ray, Object** hit_obj, float& tmin) {
	bool hit = false;
	tmin = FLT_MAX;  //contains the closest primitive intersection

	while (1) {
		if (currentNode->isLeaf()) {
			// intersect with all primitives in the leave
			for (int i = currentNode->getIndex(); i < currentNode->getIndex() + currentNode->getNObjs(); i++) {
				Object* obj = objects[i];
				float d;
				if (obj->intercepts(ray, d)) {
					assert(d > 0);
					hit = true;
					if (d < tmin) {
						hit = true;
						tmin = d;
						*hit_obj = obj;
					}
				}
			}
		} else {
			// intersect children
			BVHNode* left  = nodes[currentNode->getIndex()  ];
			BVHNode* right = nodes[currentNode->getIndex()+1];
			bool left_hit, right_hit;
			float left_intersection, right_intersection;

			left_hit  = left->getAABB().intercepts(ray, left_intersection);
			right_hit = right->getAABB().intercepts(ray, right_intersection);

			// make sure left is nearer than right
			if ((left_hit && right_hit && right_intersection < left_intersection) || (!left_hit && right_hit)) {
				std::swap(left, right);
				std::swap(left_hit, right_hit);
				std::swap(left_intersection, right_intersection);
			}

			// left hit
			if (left_hit) {
				currentNode = left;
			}
			
			// right hit
			if (right_hit) {
				hit_stack.push_back(StackItem(right, right_intersection));
			}

			if (left_hit || right_hit) {
				continue;
			}
		}

		// recursive call on the stack
		while (!hit_stack.empty()) {
			StackItem p = hit_stack.back();
			hit_stack.pop_back();

			// if we already have a hit, we can check if the AABB of the stack elem
			// is further away and discard it if so
			if (p.t > tmin) {
				continue;
			}

			float d;
			Object* ho = nullptr;
			if (traverse_recursive(p.ptr, ray, &ho, d)) {
				if (d < tmin) {
					hit = true;
					tmin = d;
					*hit_obj = ho;
				}
			}
		}

		return hit;
	}
}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
	float tmin = FLT_MAX;  //contains the closest primitive intersection

	for (Plane* p : planes) {
		float d;
		if (p->intercepts(ray, d)) {
			if (d < tmin) {
				tmin = d;
				*hit_obj = static_cast<Object*>(p);
			}
		}
	}


	//bool traverse_hit = false;
	Object* ho = nullptr;
	float d;
	if (traverse_recursive(nodes[0], ray, &ho, d)) {
		//traverse_hit = true;
		if (d < tmin) {
			tmin = d;
			*hit_obj = ho;
		}
	}

	assert(tmin > 0.0f);

	if (tmin != FLT_MAX) {
		hit_point = ray.origin + ray.direction * tmin;
	}
	
	/* 
	================ THIS IS JUST A SANITY CHECK ================
	if (!traverse_hit) {
		for (int i = 0; i < objects.size(); i++) {
			float x;
			if (objects[i]->intercepts(ray, x)) {
				// find node which was in charge of the obj
				int current_node_index;
				for (int j = 0; j < nodes.size(); j++) {
					BVHNode* n = nodes[j];
					if (n->isLeaf() && n->getIndex() <= i && n->getIndex()+n->getNObjs() > i) {
						current_node_index = j;
					}
				}

				std::vector<int> ancestors;
				while(current_node_index != 0) {
					for (int j = 0; j < nodes.size(); j++) {
						BVHNode* n = nodes[j];
						if (!n->isLeaf() && n->getIndex() <= current_node_index && n->getIndex()+2 > current_node_index) {
							ancestors.push_back(current_node_index);
							current_node_index = j;
							break;
						}
					}
				}

				traverse_recursive(nodes[0], ray, &ho, d);
				assert(false);
			}
		}
	}*/

	return (tmin != FLT_MAX);
}

bool BVH::Traverse(Ray& ray) {  //shadow ray with length
		float tmp;

		double length = ray.direction.length(); //distance between light and intersection point
		ray.direction.normalize();

		Object* hit_obj;
		Vector hit_point;

		return Traverse(ray, &hit_obj, hit_point);
	}		
