#include "CircularQueue.h"

CircularQueue::CircularQueue() {
    this->front = 0;
    this->rear = 0;
    this->size = 0;
}

// Check if the queue is full
bool CircularQueue::isFull() {
    return this->size == QUEUE_CAPACITY;
}

// Check if the queue is empty
bool CircularQueue::isEmpty() {
    return this->size == 0;
}

// Enqueue an element to the queue
bool CircularQueue::enqueue(uint8_t value) {
    if (this->isFull()) {
        return false;
    }

    this->items[this->rear] = value;
    this->rear = (this->rear + 1) % QUEUE_CAPACITY;  // Wrap around
    this->size++;
    return true;
}

// Dequeue an element from the queue
uint8_t CircularQueue::dequeue() {
    if (this->isEmpty()) {
        return 0;
    }

    uint8_t value = this->items[this->front];
    this->front = (this->front + 1) % QUEUE_CAPACITY;  // Wrap around
    this->size--;
    return value;
}

uint8_t CircularQueue::average() {
    if (this->isEmpty()) {
        return 0;
    }

    int sum = 0;
    int index = this->front;
    for (int i = 0; i < this->size; i++) {
        sum += this->items[index];
        index = (index + 1) % QUEUE_CAPACITY;  // Wrap around
    }

    return (uint8_t)((float)sum / this->size);
}

int CircularQueue::getSize(){
    return this->size;
}
