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

	// TODO: recursion stop test

	// check which axis has the largest range of centroids
	float mins[3] = {  FLT_MAX };
	float maxs[3] = { -FLT_MAX };
	for (int i = 0; i < num_objs; i++) {
		Object* obj = objects[i+left_index];
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

	//right_index, left_index and split_index refer to the indices in the objects vector
	// do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector. 
	// node.index can have a index of objects vector or a index of nodes vector		
}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
			float tmp;
			float tmin = FLT_MAX;  //contains the closest primitive intersection
			bool hit = false;

			BVHNode* currentNode = nodes[0];
			
			//PUT YOUR CODE HERE
	}

bool BVH::Traverse(Ray& ray) {  //shadow ray with length
			float tmp;

			double length = ray.direction.length(); //distance between light and intersection point
			ray.direction.normalize();

			//PUT YOUR CODE HERE
	}		
