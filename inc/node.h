#ifndef NODE_H
#define NODE_H

#include <stdint.h>

struct node {
    uint16_t val;
    struct node *next, *prev;
};

#endif /* ifndef NODE_H */
