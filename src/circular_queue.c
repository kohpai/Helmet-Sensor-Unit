#include <stdint.h>
#include <stdlib.h>

#include "circular_queue.h"

void init_queue(struct queue *q, uint8_t size)
{
    uint8_t count;
    struct node *tmp, *prev_tmp;

    tmp = (struct node*)malloc(sizeof(struct node));
    tmp->val = 0;
    tmp->next = tmp->prev = NULL;
    prev_tmp = tmp;
    q->tail = prev_tmp;

    for (count = 0; count < (size - 1); ++count) {
        struct node *tmp = (struct node*)malloc(sizeof(struct node));

        tmp->val    = 0;
        tmp->next   = prev_tmp;
        tmp->prev   = NULL;

        prev_tmp->prev  = tmp;
        prev_tmp        = tmp;
    }

    q->head = prev_tmp;
}

uint16_t enqueue(struct queue *q, uint16_t val)
{
    uint16_t garbage = q->tail->val;

    q->tail->val = val;
    q->tail->next = q->head;
    q->tail->prev->next = NULL;
    q->head = q->tail;
    q->tail = q->tail->prev;
    q->head->next->prev = q->head;
    q->head->prev = NULL;

    return garbage;
}
