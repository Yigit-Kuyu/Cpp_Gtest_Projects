#include "declerations.h"

template<typename T>
bool YcK<T>::push(T e, int flag) {
    if (top2 - top1 == 1) {
        return false;
    }

    if (flag == 1) {
        data[++top1] = e;
        return true;
    } else if (flag == 2) {
        data[--top2] = e;
        return true;
    } else {
        return false;
    }

}

template<typename T>
bool YcK<T>::empty(int flag) {
    if (flag == 1) {
        return top1 == -1;
    } else if (flag == 2) {
        return top2 == MaxSize;
    } else {
        return false;
    }
}

template<typename T>
bool YcK<T>::pop(int flag) {
    if (empty(flag)) {
        return false;
    }
    if (flag == 1) {
        top1--;
        return true;
    } else if (flag == 2) {
        top2++;
        return true;
    } else {
        return false;
    }
}

template<typename T>
bool YcK<T>::top(T &e, int flag) {
    if (empty(flag)) {
        return false;
    }
    if (flag == 1) {
        e = data[top1];
        return true;
    } else if (flag == 2) {
        e = data[top2];
        return true;
    } else {
        return false;
    }
}
