//
// Created by viu on 10/05/2025.
//

#include "KDTree.h"
#include "Boid.h"

namespace ModelAOS {
    // A method to create a node of K D tree
    Node *KDTree::newNode(modelAOS::Boid *boid) {
        const auto temp = new Node();
        temp->boid = boid;
        temp->left = temp->right = nullptr;
        return temp;
    }

    // Inserts a new node and returns root of modified tree
    // The parameter depth is used to decide axis of comparison
    Node *KDTree::insertRec(Node *root, modelAOS::Boid *boid, unsigned depth) {
        // Tree is empty?
        if (root == nullptr)
            return newNode(boid);

        // Calculate current dimension (cd) of comparison
        unsigned cd = depth % 2;
        float value = cd == 0 ? boid->xPosition : boid->yPosition;
        float rootValue = cd == 0 ? root->boid->xPosition : root->boid->yPosition;

        // Compare the new boid with root on current dimension 'cd'
        // and decide the left or right subtree
        if (value < rootValue)
            root->left = insertRec(root->left, boid, depth + 1);
        else
            root->right = insertRec(root->right, boid, depth + 1);

        return root;
    }

    // A utility method to determine if two boids are in the same neighborhood
    bool KDTree::areNeighbors(const modelAOS::Boid *boid1, const modelAOS::Boid *boid2) const {
        const float dx = boid1->xPosition - boid2->xPosition;
        const float dy = boid1->yPosition - boid2->yPosition;
        return boid1 != boid2 && dx * dx + dy * dy <= visualRange * visualRange;
    }

    // Searches a boid represented by "boid[]" in the K D tree.
    // The parameter depth is used to determine current axis.
    void KDTree::queryRec(const Node *root, const modelAOS::Boid *boid, const unsigned depth,
                          std::vector<modelAOS::Boid *> &result) const {
        // Base cases
        if (root == nullptr)
            return;
        if (areNeighbors(root->boid, boid))
            result.push_back(root->boid);

        // Current dimension is computed using current depth and total
        // dimensions (k)
        unsigned cd = depth % 2;
        float value = cd == 0 ? boid->xPosition : boid->yPosition;
        float rootValue = cd == 0 ? root->boid->xPosition : root->boid->yPosition;

        // Compare boid with root with respect to cd (Current dimension)
        if (value - visualRange < rootValue)
            queryRec(root->left, boid, depth + 1, result);

        // Check right subtree if possible
        if (value + visualRange >= rootValue)
            queryRec(root->right, boid, depth + 1, result);
    }

    void KDTree::freeTree(const Node *node) {
        if (node == nullptr)
            return;
        freeTree(node->left);
        freeTree(node->right);
        delete node;
    }

    KDTree::KDTree(modelAOS::Boid *boids, const int boidNum, const float visualRange) {
        this->visualRange = visualRange;
        root = nullptr;
        for (int i = 0; i < boidNum; ++i) {
            root = insert(root, &boids[i]);
        }
    }

    // Function to insert a new boid with given boid in
    // KD Tree and return new root. It mainly uses above recursive
    // function "insertRec()"
    Node *KDTree::insert(Node *root, modelAOS::Boid *boid) {
        return insertRec(root, boid, 0);
    }

    // Function to search all neighbors to a given boid in the K D tree
    std::vector<modelAOS::Boid *> KDTree::query(const modelAOS::Boid *boid) const {
        // Pass current depth as 0
        auto result = std::vector<modelAOS::Boid *>();
        queryRec(root, boid, 0, result);
        return result;
    }

    KDTree::~KDTree() {
        freeTree(root);
    }
}
