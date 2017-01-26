#ifndef CIRCULAR_QUEUE_H
#define CIRCULAR_QUEUE_H

#include "node.h"

struct queue {
    struct node *head, *tail;
};

uint16_t enqueue(struct queue *q, uint16_t val);
void init_queue(struct queue *q, uint8_t size);

#endif /* ifndef CIRCULAR_QUEUE_H */
