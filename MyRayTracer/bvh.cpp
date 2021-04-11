#include "rayAccelerator.h"
#include "macros.h"
#include <algorithm>

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

	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	AABB world_bbox = AABB(min, max);

	for (Object* obj : objs) {
		AABB bbox = obj->GetBoundingBox();
		world_bbox.extend(bbox);
		objects.push_back(obj);
	}
	world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
	world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
	root->setAABB(world_bbox);
	nodes.push_back(root);
	build_recursive(0, objects.size(), root); // -> root node takes all the 
}

void BVH::build_recursive(int left_index, int right_index, BVHNode *node) {
	int num_objs = right_index - left_index;

	// recursion stop test
	if (num_objs <= 2) {
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
	int split_idx = 0;
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
	build_recursive(left_index, split_idx - left_index, left);
	build_recursive(split_idx, right_index - split_idx, right);

	//right_index, left_index and split_index refer to the indices in the objects vector
	// do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector. 
	// node.index can have a index of objects vector or a index of nodes vector		
}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
	float tmp;
	float tmin = FLT_MAX;  //contains the closest primitive intersection
	bool hit = false;

	stack<std::pair<BVHNode*, float>> hit_stack;

	BVHNode* currentNode = nodes[0];

	//PUT YOUR CODE HERE
	while (1) {
		if (currentNode->isLeaf()) {
			// intersect with all primitives in the leave
			for (int i = currentNode->getIndex(); i < currentNode->getNObjs(); i++) {
				Object* obj = objects[i];
				float d;
				if (obj->intercepts(ray, d)) {
					hit = true;
					if (d < tmin) {
						tmin = d;
						*hit_obj = obj;
					}
				}
			}

			// calc hit_point if we had an intersection
			if (tmin != FLT_MAX) {
				hit_point = ray.origin + ray.direction * tmin;
			}

			break;
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
				hit_stack.push(std::pair<BVHNode*, float>(right, right_intersection));
			}

			if (left_hit || right_hit) {
				continue;
			}
		}

		// recursive call on the stack
		while (!hit_stack.empty()) {
			pair<BVHNode*, float> p = hit_stack.top();
			hit_stack.pop();
			Object* ho = nullptr;
			Vector hp;

			// TODO: if we already have a hit, we could check if the AABB of the stack elem
			// is further away and discard it if so

			if (Traverse(ray, &ho, hp)) {
				float d = (hp - ray.origin).length();
				if (d < tmin) {
					tmin = d;
					*hit_obj = ho;
					hit_point = hp;
				}
			}
		}

		return (tmin != FLT_MAX);
	}
}

bool BVH::Traverse(Ray& ray) {  //shadow ray with length
			float tmp;

			double length = ray.direction.length(); //distance between light and intersection point
			ray.direction.normalize();

			//PUT YOUR CODE HERE
	}		
