#pragma once

#include <cmath>
#include <cstdint>

#include <type_traits>

#define WRITE_ONCE(x, val) x = (val)

class BasicRedBlackTree {
public:
    enum class Color : uint8_t { RED = 0, BLACK = 1 };

    class Node {
    public:
        friend class BasicRedBlackTree;

        // template <typename T>
        // friend class RedBlackTree;

        Node() = default;

        Color color() const { return static_cast<Color>(parent_and_color & 1); }
        bool is_red() const { return color() == Color::RED; }
        bool is_black() const { return color() == Color::BLACK; }

        Node* parent() const {
            return reinterpret_cast<Node*>(parent_and_color & ~static_cast<uintptr_t>(1));
        }
        // Node* right() const { return right_; }
        // Node* left() const { return left_; }

        Node* next() const {
            Node *parent, *node = const_cast<Node*>(this);

            /*
             * If we have a right-hand child, go down and then left as far
             * as we can.
             */
            if (node->right) {
                node = node->right;
                while (node->left)
                    node = node->left;
                return node;
            }

            /*
             * No right-hand children. Everything down and left is smaller than us,
             * so any 'next' node must be in the general direction of our parent.
             * Go up the tree; any time the ancestor is a right-hand child of its
             * parent, keep going up. First time it's a left-hand child of its
             * parent, said parent is our 'next' node.
             */
            while ((parent = node->parent()) && node == parent->right)
                node = parent;

            return parent;
        }

        Node* prev() const {
            Node *parent, *node = const_cast<Node*>(this);

            /*
             * If we have a left-hand child, go down and then right as far
             * as we can.
             */
            if (node->left) {
                node = node->left;
                while (node->right)
                    node = node->right;
                return node;
            }

            /*
             * No left-hand children. Go up till we find an ancestor which
             * is a right-hand child of its parent.
             */
            while ((parent = node->parent()) && node == parent->left)
                node = parent;

            return parent;
        }

        void set_parent_and_color(Node* parent, Color color) {
            parent_and_color = reinterpret_cast<uintptr_t>(parent) | static_cast<uintptr_t>(color);
        }

        void set_parent(Node* parent) { set_parent_and_color(parent, color()); }

        void set_red() { parent_and_color &= ~static_cast<uintptr_t>(1); }
        void set_black() { parent_and_color |= static_cast<uintptr_t>(1); }
        void set_color(Color color) {
            if (color == Color::RED)
                set_red();
            else
                set_black();
        }

        Node* red_get_parent() const { return reinterpret_cast<Node*>(parent_and_color); }

        uintptr_t parent_and_color;
        Node* right;
        Node* left;
    };

    // void insert_root(Node* node) {
    //     node->set_parent_and_color(nullptr, Color::BLACK);
    //     node->left = node->right = nullptr;

    //     root = node;
    // }

    // void insert_left(Node* parent, Node* child) {
    //     parent->left = child;

    //     child->set_parent_and_color(parent, Color::RED);
    //     child->left = child->right = nullptr;

    //     insert_color(child);
    // }

    // void insert_right(Node* parent, Node* child) {
    //     parent->right = child;

    //     child->set_parent_and_color(parent, Color::RED);
    //     child->left = child->right = nullptr;

    //     insert_color(child);
    // }

    static inline void link_node(Node* node, Node* parent, Node** link) {
        node->set_parent_and_color(parent, Color::RED);
        node->left = node->right = nullptr;

        *link = node;
    }

    void insert_color(Node* node) {
        Node *parent = node->parent(), *gparent, *tmp;

        while (true) {
            /*
             * Loop invariant: node is red.
             */
            if (!parent) [[unlikely]] {
                /*
                 * The inserted node is root. Either this is the
                 * first node, or we recursed at Case 1 below and
                 * are no longer violating 4).
                 */
                node->set_parent_and_color(nullptr, Color::BLACK);
                break;
            }

            /*
             * If there is a black parent, we are done.
             * Otherwise, take some corrective action as,
             * per 4), we don't want a red root or two
             * consecutive red nodes.
             */
            if (parent->is_black())
                break;

            gparent = parent->red_get_parent();

            tmp = gparent->right;
            if (parent != tmp) { /* parent == gparent->rb_left */
                if (tmp && tmp->is_red()) {
                    /*
                     * Case 1 - node's uncle is red (color flips).
                     *
                     *       G            g
                     *      / \          / \
                     *     p   u  -->   P   U
                     *    /            /
                     *   n            n
                     *
                     * However, since g's parent might be red, and
                     * 4) does not allow this, we need to recurse
                     * at g.
                     */
                    tmp->set_parent_and_color(gparent, Color::BLACK);
                    parent->set_parent_and_color(gparent, Color::BLACK);
                    node   = gparent;
                    parent = node->parent();
                    node->set_parent_and_color(parent, Color::RED);
                    continue;
                }

                tmp = parent->right;
                if (node == tmp) {
                    /*
                     * Case 2 - node's uncle is black and node is
                     * the parent's right child (left rotate at parent).
                     *
                     *      G             G
                     *     / \           / \
                     *    p   U  -->    n   U
                     *     \           /
                     *      n         p
                     *
                     * This still leaves us in violation of 4), the
                     * continuation into Case 3 will fix that.
                     */
                    tmp = node->left;
                    WRITE_ONCE(parent->right, tmp);
                    WRITE_ONCE(node->left, parent);
                    if (tmp)
                        tmp->set_parent_and_color(parent, Color::BLACK);
                    parent->set_parent_and_color(node, Color::RED);
                    parent = node;
                    tmp    = node->right;
                }

                /*
                 * Case 3 - node's uncle is black and node is
                 * the parent's left child (right rotate at gparent).
                 *
                 *        G           P
                 *       / \         / \
                 *      p   U  -->  n   g
                 *     /                 \
                 *    n                   U
                 */
                WRITE_ONCE(gparent->left, tmp); /* == parent->rb_right */
                WRITE_ONCE(parent->right, gparent);
                if (tmp)
                    tmp->set_parent_and_color(gparent, Color::BLACK);
                rotate_set_parents(gparent, parent, Color::RED);
                break;
            } else {
                tmp = gparent->left;
                if (tmp && tmp->is_red()) {
                    /* Case 1 - color flips */
                    tmp->set_parent_and_color(gparent, Color::BLACK);
                    parent->set_parent_and_color(gparent, Color::BLACK);
                    node   = gparent;
                    parent = node->parent();
                    node->set_parent_and_color(parent, Color::RED);
                    continue;
                }

                tmp = parent->left;
                if (node == tmp) {
                    /* Case 2 - right rotate at parent */
                    tmp = node->right;
                    WRITE_ONCE(parent->left, tmp);
                    WRITE_ONCE(node->right, parent);
                    if (tmp)
                        tmp->set_parent_and_color(parent, Color::BLACK);
                    parent->set_parent_and_color(node, Color::RED);
                    parent = node;
                    tmp    = node->left;
                }

                /* Case 3 - left rotate at gparent */
                WRITE_ONCE(gparent->right, tmp); /* == parent->rb_left */
                WRITE_ONCE(parent->left, gparent);
                if (tmp)
                    tmp->set_parent_and_color(gparent, Color::BLACK);
                rotate_set_parents(gparent, parent, Color::RED);
                break;
            }
        }
    }

    void erase(Node* node) {
        Node* rebalance;
        rebalance = __erase(node);
        if (rebalance)
            __erase_color(rebalance);
    }

    /*
     * This function returns the first node (in sort order) of the tree.
     */
    Node* first() const {
        Node* n;

        n = root;
        if (!n)
            return nullptr;
        while (n->left)
            n = n->left;
        return n;
    }

    Node* last() const {
        Node* n;

        n = root;
        if (!n)
            return nullptr;
        while (n->right)
            n = n->right;
        return n;
    }

    Node* root = nullptr;

private:
    void change_child(Node* old_node, Node* new_node, Node* parent) {
        if (parent) {
            if (parent->left == old_node)
                WRITE_ONCE(parent->left, new_node);
            else
                WRITE_ONCE(parent->right, new_node);
        } else
            WRITE_ONCE(root, new_node);
    }

    /*
     * Helper function for rotations:
     * - old's parent and color get assigned to new
     * - old gets assigned new as a parent and 'color' as a color.
     */
    void rotate_set_parents(Node* old_node, Node* new_node, Color color) {
        Node* parent               = old_node->parent();
        new_node->parent_and_color = old_node->parent_and_color;
        old_node->set_parent_and_color(new_node, color);
        change_child(old_node, new_node, parent);
    }

    Node* __erase(Node* node) {
        Node* child = node->right;
        Node* tmp   = node->left;
        Node *parent, *rebalance;
        uintptr_t pc;

        if (!tmp) {
            /*
             * Case 1: node to erase has no more than 1 child (easy!)
             *
             * Note that if there is one child it must be red due to 5)
             * and node must be black due to 4). We adjust colors locally
             * so as to bypass __rb_erase_color() later on.
             */
            pc     = node->parent_and_color;
            parent = ((Node*)(pc & ~3));
            change_child(node, child, parent);
            if (child) {
                child->parent_and_color = pc;
                rebalance               = nullptr;
            } else
                rebalance = ((pc) & 1) ? parent : nullptr;
            tmp = parent;
        } else if (!child) {
            /* Still case 1, but this time the child is node->rb_left */
            tmp->parent_and_color = pc = node->parent_and_color;
            parent                     = ((Node*)(pc & ~3));
            change_child(node, tmp, parent);
            rebalance = nullptr;
            tmp       = parent;
        } else {
            Node *successor = child, *child2;

            tmp = child->left;
            if (!tmp) {
                /*
                 * Case 2: node's successor is its right child
                 *
                 *    (n)          (s)
                 *    / \          / \
                 *  (x) (s)  ->  (x) (c)
                 *        \
                 *        (c)
                 */
                parent = successor;
                child2 = successor->right;

            } else {
                /*
                 * Case 3: node's successor is leftmost under
                 * node's right child subtree
                 *
                 *    (n)          (s)
                 *    / \          / \
                 *  (x) (y)  ->  (x) (y)
                 *      /            /
                 *    (p)          (p)
                 *    /            /
                 *  (s)          (c)
                 *    \
                 *    (c)
                 */
                do {
                    parent    = successor;
                    successor = tmp;
                    tmp       = tmp->left;
                } while (tmp);
                child2 = successor->right;
                WRITE_ONCE(parent->left, child2);
                WRITE_ONCE(successor->right, child);
                child->set_parent(successor);
            }

            tmp = node->left;
            WRITE_ONCE(successor->left, tmp);
            tmp->set_parent(successor);

            pc  = node->parent_and_color;
            tmp = ((Node*)(pc & ~3));
            change_child(node, successor, tmp);

            if (child2) {
                child2->set_parent_and_color(parent, Color::BLACK);
                rebalance = nullptr;
            } else {
                rebalance = successor->is_black() ? parent : nullptr;
            }
            successor->parent_and_color = pc;
            tmp                         = successor;
        }

        return rebalance;
    }

    void __erase_color(Node* parent) {
        Node *node = nullptr, *sibling, *tmp1, *tmp2;

        while (true) {
            /*
             * Loop invariants:
             * - node is black (or NULL on first iteration)
             * - node is not the root (parent is not NULL)
             * - All leaf paths going through parent and node have a
             *   black node count that is 1 lower than other leaf paths.
             */
            sibling = parent->right;
            if (node != sibling) { /* node == parent->rb_left */
                if (sibling->is_red()) {
                    /*
                     * Case 1 - left rotate at parent
                     *
                     *     P               S
                     *    / \             / \
                     *   N   s    -->    p   Sr
                     *      / \         / \
                     *     Sl  Sr      N   Sl
                     */
                    tmp1 = sibling->left;
                    WRITE_ONCE(parent->right, tmp1);
                    WRITE_ONCE(sibling->left, parent);
                    tmp1->set_parent_and_color(parent, Color::BLACK);
                    rotate_set_parents(parent, sibling, Color::RED);
                    sibling = tmp1;
                }
                tmp1 = sibling->right;
                if (!tmp1 || tmp1->is_black()) {
                    tmp2 = sibling->left;
                    if (!tmp2 || tmp2->is_black()) {
                        /*
                         * Case 2 - sibling color flip
                         * (p could be either color here)
                         *
                         *    (p)           (p)
                         *    / \           / \
                         *   N   S    -->  N   s
                         *      / \           / \
                         *     Sl  Sr        Sl  Sr
                         *
                         * This leaves us violating 5) which
                         * can be fixed by flipping p to black
                         * if it was red, or by recursing at p.
                         * p is red when coming from Case 1.
                         */
                        sibling->set_parent_and_color(parent, Color::RED);
                        if (parent->is_red())
                            parent->set_black();
                        else {
                            node   = parent;
                            parent = node->parent();
                            if (parent)
                                continue;
                        }
                        break;
                    }
                    /*
                     * Case 3 - right rotate at sibling
                     * (p could be either color here)
                     *
                     *   (p)           (p)
                     *   / \           / \
                     *  N   S    -->  N   sl
                     *     / \             \
                     *    sl  Sr            S
                     *                       \
                     *                        Sr
                     *
                     * Note: p might be red, and then both
                     * p and sl are red after rotation(which
                     * breaks property 4). This is fixed in
                     * Case 4 (in __rb_rotate_set_parents()
                     *         which set sl the color of p
                     *         and set p RB_BLACK)
                     *
                     *   (p)            (sl)
                     *   / \            /  \
                     *  N   sl   -->   P    S
                     *       \        /      \
                     *        S      N        Sr
                     *         \
                     *          Sr
                     */
                    tmp1 = tmp2->right;
                    WRITE_ONCE(sibling->left, tmp1);
                    WRITE_ONCE(tmp2->right, sibling);
                    WRITE_ONCE(parent->right, tmp2);
                    if (tmp1)
                        tmp1->set_parent_and_color(sibling, Color::BLACK);
                    tmp1    = sibling;
                    sibling = tmp2;
                }
                /*
                 * Case 4 - left rotate at parent + color flips
                 * (p and sl could be either color here.
                 *  After rotation, p becomes black, s acquires
                 *  p's color, and sl keeps its color)
                 *
                 *      (p)             (s)
                 *      / \             / \
                 *     N   S     -->   P   Sr
                 *        / \         / \
                 *      (sl) sr      N  (sl)
                 */
                tmp2 = sibling->left;
                WRITE_ONCE(parent->right, tmp2);
                WRITE_ONCE(sibling->left, parent);
                tmp1->set_parent_and_color(sibling, Color::BLACK);
                if (tmp2)
                    tmp2->set_parent(parent);
                rotate_set_parents(parent, sibling, Color::BLACK);
                break;
            } else {
                sibling = parent->left;
                if (sibling->is_red()) {
                    /* Case 1 - right rotate at parent */
                    tmp1 = sibling->right;
                    WRITE_ONCE(parent->left, tmp1);
                    WRITE_ONCE(sibling->right, parent);
                    tmp1->set_parent_and_color(parent, Color::BLACK);
                    rotate_set_parents(parent, sibling, Color::RED);
                    sibling = tmp1;
                }
                tmp1 = sibling->left;
                if (!tmp1 || tmp1->is_black()) {
                    tmp2 = sibling->right;
                    if (!tmp2 || tmp2->is_black()) {
                        /* Case 2 - sibling color flip */
                        sibling->set_parent_and_color(parent, Color::RED);
                        if (parent->is_red())
                            parent->set_black();
                        else {
                            node   = parent;
                            parent = node->parent();
                            if (parent)
                                continue;
                        }
                        break;
                    }
                    /* Case 3 - left rotate at sibling */
                    tmp1 = tmp2->left;
                    WRITE_ONCE(sibling->right, tmp1);
                    WRITE_ONCE(tmp2->left, sibling);
                    WRITE_ONCE(parent->left, tmp2);
                    if (tmp1)
                        tmp1->set_parent_and_color(sibling, Color::BLACK);
                    tmp1    = sibling;
                    sibling = tmp2;
                }
                /* Case 4 - right rotate at parent + color flips */
                tmp2 = sibling->right;
                WRITE_ONCE(parent->left, tmp2);
                WRITE_ONCE(sibling->right, parent);
                tmp1->set_parent_and_color(sibling, Color::BLACK);
                if (tmp2)
                    tmp2->set_parent(parent);
                rotate_set_parents(parent, sibling, Color::BLACK);
                break;
            }
        }
    }
};

template <typename T>
class RedBlackTree final {
public:
    class Node : private BasicRedBlackTree::Node {
    public:
        friend class RedBlackTree;
        Node() { set_dangling(); }

        bool is_dangling() const { return BasicRedBlackTree::Node::parent_and_color == 0; }

        bool is_red() const { return BasicRedBlackTree::Node::is_red(); }
        bool is_black() const { return BasicRedBlackTree::Node::is_black(); }

        T* parent() {
            return static_cast<T*>(static_cast<Node*>(BasicRedBlackTree::Node::parent()));
        }
        T* right() { return static_cast<T*>(static_cast<Node*>(BasicRedBlackTree::Node::right)); }
        T* left() { return static_cast<T*>(static_cast<Node*>(BasicRedBlackTree::Node::left)); }

        T* next() { return static_cast<T*>(static_cast<Node*>(BasicRedBlackTree::Node::next())); }
        T* prev() { return static_cast<T*>(static_cast<Node*>(BasicRedBlackTree::Node::prev())); }

    private:
        void set_dangling() { BasicRedBlackTree::Node::parent_and_color = 0; }
    };

    bool insert(T& node) requires(std::is_base_of_v<Node, T>) {
        if (!static_cast<Node&>(node).is_dangling())
            return false;

        BasicRedBlackTree::Node **link = &(tree_.root), *parent = nullptr;

        /* Figure out where to put new node */
        while (*link) {
            parent = *link;

            T& current = *static_cast<T*>(static_cast<Node*>(*link));
            if (node < current)
                link = &((*link)->left);
            else
                link = &((*link)->right);
        }

        /* Add new node and rebalance tree. */
        tree_.link_node(static_cast<Node*>(&node), parent, link);
        tree_.insert_color(static_cast<Node*>(&node));
        return true;
    }

    bool insert_set(T& node) requires(std::is_base_of_v<Node, T>) {
        if (!static_cast<Node&>(node).is_dangling())
            return false;

        BasicRedBlackTree::Node **link = &(tree_.root), *parent = nullptr;

        /* Figure out where to put new node */
        while (*link) {
            parent = *link;

            T& current = *static_cast<T*>(static_cast<Node*>(*link));
            if (node < current)
                link = &((*link)->left);
            else if (current < node)
                link = &((*link)->right);
            else
                return false;
        }

        /* Add new node and rebalance tree. */
        tree_.link_node(static_cast<Node*>(&node), parent, link);
        tree_.insert_color(static_cast<Node*>(&node));
        return true;
    }

    bool erase(T& node) requires(std::is_base_of_v<Node, T>) {
        if (static_cast<Node&>(node).is_dangling())
            return false;

        tree_.erase(static_cast<Node*>(&node));
        static_cast<Node&>(node).set_dangling();
        return true;
    }

    bool empty() const requires(std::is_base_of_v<Node, T>) { return tree_.root == nullptr; }

    T* root() const requires(std::is_base_of_v<Node, T>) {
        return static_cast<T*>(static_cast<Node*>(tree_.root));
    }
    T* first() const requires(std::is_base_of_v<Node, T>) {
        return static_cast<T*>(static_cast<Node*>(tree_.first()));
    }
    T* last() const requires(std::is_base_of_v<Node, T>) {
        return static_cast<T*>(static_cast<Node*>(tree_.last()));
    }

private:
    BasicRedBlackTree tree_;
};