//
//	[Author] = Ming Tai Ha
//

#include "searchstate.h"
#include "heuristic.h"
#include <iostream>
#include <vector>
#include <cassert>
#include <armadillo>

#define hparent(index) (((index)-1)/2)
#define lchild(index)  (((index)*2)+1)
#define rchild(index)  (((index)*2)+2)

#define TESTING 0 

using namespace std;


state::state(int x, int y, state * parent) {
	this->x = x;
	this->y = y;
	g_value = -1;
	h_value = -1;
	this->parent = parent;
}

state::~state() {
}

void state::setG(int start_x, int start_y) {
	g_value = mdist(this->x, this->y, start_x, start_y);	
//	g_value = 1 + ((this->parent == NULL) ? 0 : this->parent->g_value);
}

void state::setH(int goal_x, int goal_y) {
	h_value = mdist(this->x, this->y, goal_x, goal_y);
}

bool state::operator<(const state &other) {
	return (this->g_value + this->h_value) < (other.g_value + other.h_value);
}

bool state::operator>(const state &other) {
	return (this->g_value + this->h_value) > (other.g_value + other.h_value);
}

bool state::operator<=(const state &other) {
	return (this->g_value + this->h_value) <= (other.g_value + other.h_value);
}

bool state::operator>=(const state &other) {
	return (this->g_value + this->h_value) >= (other.g_value + other.h_value);
}

bool state::operator==(const state &other) {
	return (this->g_value + this->h_value) == (other.g_value + other.h_value);
}

bool state::operator!=(const state &other) {
	return (this->g_value + this->h_value) != (other.g_value + other.h_value);
}

ostream &operator<<(ostream &out, state &st) {
	out << "[x y] = [" << st.x << " " << st.y << "]\t\t \n\tCOST: [g h] = [" << st.g_value << " " << st.h_value << "]\n\tPARENT: " << st.parent << "\n";

}


/// ------------ HEAP -------------- ///


heap_n::heap_n() {
	queue.reserve(65535);
}

heap_n::~heap_n() {
}

void heap_n::swap(int a, int b) {
	state * temp = this->queue[a];
	this->queue[a] = this->queue[b];
	this->queue[b] = temp;
}

void heap_n::siftup() {
	assert(this->queue.size() > 0);
	int index = this->queue.size() - 1;
	while(index != 0) {
		if (*(queue[index]) < *(queue[hparent(index)])) {
			this->swap(index, hparent(index));
			index = hparent(index);
		} else {
			break;
		}
	}
}

void heap_n::siftdown() {
//	assert(n_elem > 0);
	int index = this->queue.size() - 1;
	int ptr = 0;
	int end = index - 1;
	swap(0, index);
	
	int new_index = lchild(index);
	while(ptr < end) {
		if (new_index == index) {
			break;
		}
		if (rchild(index) < index && *(this->queue[rchild(ptr)]) < *(this->queue[lchild(ptr)])){
			new_index = rchild(ptr);
		}
		if (new_index < index && *(this->queue[new_index]) < *(this->queue[ptr])) {
			this->swap(new_index, ptr);
			ptr = new_index;
		} else {
			break;
		}
	}
}

void heap_n::insert(state * item) {
//	//printf("Inserting %p\n", item);
//	//cout << *item << endl;
//	//printf("Indubitably, my good sir\n");
	this->queue.push_back(item);
//	//printf("The push_back?\n");
	this->siftup();
//	//printf("Exit\n");
}


state * heap_n::remove() {
	if(this->queue.size() == 0) {
		return NULL;
	} else {
		this->swap(0, this->queue.size() - 1);
		state * s = this->queue[this->queue.size()-1];
//		//printf("Removing %p\n", s);
		this->queue.pop_back();
//		this->swap(0, --(this->n_elem) - 1);
		this->siftdown();
		return s;
	}
}

bool heap_n::isEmpty() {
	if(this->queue.size() == 0) {
		return true;
	} else {
		return false;
	}
}


/// ------------ SEARCHTREE -------------- /// 


searchtree::searchtree() {
}

searchtree::searchtree(int sx, int sy, int gx, int gy, imat map) {
  init(sx, sy, gx, gy, map);
}

void searchtree::init(int start_x, int start_y, int goal_x, int goal_y, imat map) {
	this->map = map;
	this->start_x = start_x;
	this->start_y = start_y;
	this->goal_x = goal_x;
	this->goal_y = goal_y;
	visited = zeros<imat>(size(map, 1), size(map, 1));
	queued = zeros<imat>(size(map, 1), size(map, 1));
	root = new state(start_x, start_y, NULL);
	root->setG(0, 0);
	root->setH(goal_x, goal_y);
	pqueue = heap_n();
	pqueue.insert(root);
}
	
searchtree::~searchtree(){
}

void searchtree::addChildren(state * cur, heap_n &pqueue, imat &visited, imat &queued, imat &map,
					int start_x, int start_y, int goal_x, int goal_y) {
	
	state * temp;
	int x_s = cur->x;
	int y_s = cur->y;
	
	ivec x_t = {x_s - 1, x_s, x_s + 1, x_s};
	ivec y_t = {y_s, y_s + 1, y_s, y_s - 1};

	for(int i = 0; i < 4; i++) {
		if (x_t(i) < 0 || x_t(i) >= size(map, 1) || y_t(i) < 0 || y_t(i) >= size(map, 1)) {
			continue;
		}
		if (map(x_t(i), y_t(i)) == 1) {
			continue;
		}
		if (visited(x_t(i), y_t(i)) == 1) {
			continue;
		}
		temp = new state(x_t(i), y_t(i), cur);
		temp->setG(start_x, start_y);
		temp->setH(goal_x, goal_y);
//		cout << "Child: " << *temp << endl;
//		cout << "Parent: " << *cur << endl;
//		cur->children.push_back(temp);
		if ((visited(x_t(i), y_t(i)) == 0)) {
//			queued(x_t(i), y_t(i)) = 1;
			if (map(x_t(i), y_t(i)) == 0) {
				cout << "Child: " << *temp << endl;
				cout << "Parent: " << *cur << endl;
				pqueue.insert(temp);
				queued(x_t(i), y_t(i)) = 1;

			} else {
			}
		} else {
		}
	}
}

void searchtree::addToTree(state * node, imat &visited) {
	if (node->parent == NULL) {
		visited(node->x , node->y) = 1;
		return;			// Exception for root case
	}
	//printf("Parent %p\n", node->parent);
	node->parent->children.push_back(node);
	//printf("addToTree push_back complete\n");
	visited(node->x, node->y) = 1;
}

/// ------------ TESTING -------------- ///


#if TESTING


int main() {

	return 0;

}

#endif
