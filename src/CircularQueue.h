#ifndef CIRCULARQUEUE_H
#define CIRCULARQUEUE_H

#include <stdint.h>

#define QUEUE_CAPACITY 40  // Define the capacity of the queue

class CircularQueue{
  private: 
    uint8_t items[QUEUE_CAPACITY];  // Queue array to store elements
    int front;                      // Front index
    int rear;                       // Rear index
    int size;  

  public:
    CircularQueue();
    bool isFull();
    bool isEmpty();
    bool enqueue(uint8_t value);
    uint8_t dequeue();
    uint8_t average();
    int getSize();
};
#endif