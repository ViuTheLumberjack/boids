//
// Created by viu on 08/05/2025.
//

#ifndef KDTREE_H
#define KDTREE_H

#include <vector>

#include "Boid.h"

namespace ModelAOS {

struct Node
{
    modelAOS::Boid *boid; // To store k dimensional boid
    Node *left, *right;
};

class KDTree {
private:

    Node* root;
    float visualRange;

    Node *newNode(modelAOS::Boid *boid);
    Node *insertRec(Node *root, modelAOS::Boid *boid, unsigned depth);
    bool areNeighbors(const modelAOS::Boid *boid1, const modelAOS::Boid *boid2) const;
    void queryRec(const Node* root, const modelAOS::Boid *boid, unsigned depth, std::vector<modelAOS::Boid *> &result) const;

    void freeTree(const Node* node);

public:
    KDTree(modelAOS::Boid* boids, int boidNum, float visualRange);

    Node* insert(Node *root, modelAOS::Boid *boid);
    std::vector<modelAOS::Boid *> query(const modelAOS::Boid *boid) const;

    ~KDTree();
};

} // Utils

#endif //KDTREE_H
